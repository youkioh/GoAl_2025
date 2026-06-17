#!/usr/bin/env python3
"""
tune_bayes.py

EID 가중치 Bayesian Optimization — 서버 CPU(48코어) 전체 활용.
Optuna TPE sampler + SQLite 스토리지로 프로세스 간 시도 결과를 공유하며,
이전 시도 결과를 학습해 Grid Search보다 적은 trial로 좋은 파라미터를 찾는다.

설치:
    pip install optuna

사용법:
    python3 tune_bayes.py                        # 200 trials, CPU수만큼 병렬
    python3 tune_bayes.py --trials 150           # 150 trials
    python3 tune_bayes.py --workers 24           # 24 병렬 워커
    python3 tune_bayes.py --seeds 3              # 조합당 3회 실행
    python3 tune_bayes.py --resume               # 이전 study 이어서 실행
    python3 tune_bayes.py --resume --trials 100  # 이어서 100 trials 추가
"""

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from multiprocessing import Pool, cpu_count

try:
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
except ImportError:
    sys.exit(
        "ERROR: optuna가 설치되지 않았습니다.\n"
        "  pip install optuna"
    )

# ─── 경로 설정 ────────────────────────────────────────────────────────────────
SRC_DIR     = "/home/new_cloud_cxl/sungsu/GoAl_2025/MRTA"
RESULTS_CSV = "/home/new_cloud_cxl/sungsu/GoAl_2025/tune/tune_bayes_results.csv"
DB_PATH     = "/home/new_cloud_cxl/sungsu/GoAl_2025/tune/tune_bayes.db"
STUDY_NAME  = "mrta_eid_bayes"

SRC_FILES = [
    "main.cpp", "simulator.h", "simulator.cpp",
    "schedular.h", "schedular.cpp", "CMakeLists.txt",
]

# ─── 실험 설정 ────────────────────────────────────────────────────────────────
N_SEEDS = 10
W_FOUND = 1.0
W_DONE  = 2.0

# ─── 연속 탐색 범위 ───────────────────────────────────────────────────────────
# Grid Search의 이산 격자보다 넓게 설정해 BO가 격자 밖도 탐색할 수 있게 한다.
# DRONE_BUDGET_MAX = DRONE_BUDGET_MIN + DRONE_BUDGET_GAP (항상 MAX >= MIN 보장)
PARAM_RANGES = {
    "EID_UNKNOWN_WEIGHT":   (0.3, 3.0),
    "EID_STALENESS_WEIGHT": (0.3, 1.0),
    "EID_DISTANCE_WEIGHT":  (0.05, 0.5),
    "DRONE_BUDGET_MIN":     (0.05, 0.30),
    "DRONE_BUDGET_GAP":     (0.0,  0.25),
}
# ─────────────────────────────────────────────────────────────────────────────


def _patch_defines(content, params):
    """schedular.cpp의 #define 값을 params에 맞게 패치한다."""
    for key, val in params.items():
        formatted = f"{val:.6g}"  # 부동소수점 오차 없이 깔끔하게 출력
        content = re.sub(
            rf'^(#define\s+{re.escape(key)}\s+)\S+',
            lambda m, v=formatted: m.group(1) + v,
            content,
            flags=re.MULTILINE,
        )
    return content


def _parse_output(stdout):
    """시뮬레이터 stdout에서 found/done/total을 파싱한다.
    Task 테이블은 ID 0~9만 출력하므로 'Active task + Completed task'로 found를 계산한다.
    """
    active_m = re.search(r'Active task\s*:\s*(\d+)', stdout)
    done_m   = re.search(r'Completed task\s*:\s*(\d+)', stdout)
    total_m  = re.search(r'Task created\s*:\s*(\d+)', stdout)

    done  = int(done_m.group(1))  if done_m  else 0
    total = int(total_m.group(1)) if total_m else 0
    found = (int(active_m.group(1)) + done) if active_m else done
    return found, done, total


def _compile_and_run(params, n_seeds, trial_number):
    """임시 디렉토리에 소스를 복사·패치·컴파일하고 n_seeds회 실행한다.
    컴파일 실패 시 None, 성공 시 [(found, done, total), ...] 반환.
    """
    tmpdir = tempfile.mkdtemp(prefix=f"bayes_{trial_number:04d}_")
    try:
        # 소스 복사
        for fname in SRC_FILES:
            src = os.path.join(SRC_DIR, fname)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(tmpdir, fname))

        # DRONE_BUDGET_GAP → DRONE_BUDGET_MAX 변환
        patch_params = {k: v for k, v in params.items()
                        if k != "DRONE_BUDGET_GAP"}
        patch_params["DRONE_BUDGET_MAX"] = round(
            params["DRONE_BUDGET_MIN"] + params["DRONE_BUDGET_GAP"], 6
        )

        sched = os.path.join(tmpdir, "schedular.cpp")
        with open(sched) as f:
            code = f.read()
        with open(sched, "w") as f:
            f.write(_patch_defines(code, patch_params))

        build = os.path.join(tmpdir, "build")
        os.makedirs(build)

        r = subprocess.run(["cmake", ".."], cwd=build,
                           capture_output=True, text=True)
        if r.returncode != 0:
            return None

        r = subprocess.run(["make", "-j1"], cwd=build,
                           capture_output=True, text=True)
        if r.returncode != 0:
            return None

        binary = os.path.join(build, "MRTA")
        runs = []
        for _ in range(n_seeds):
            try:
                out = subprocess.run([binary], capture_output=True,
                                     text=True, timeout=600)
                runs.append(_parse_output(out.stdout))
            except subprocess.TimeoutExpired:
                runs.append((0, 0, 0))
        return runs

    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def _make_objective(n_seeds):
    """n_seeds를 클로저로 캡처한 Optuna objective 함수를 반환한다."""

    def objective(trial):
        # 파라미터 제안
        params = {k: round(trial.suggest_float(k, lo, hi), 6)
                  for k, (lo, hi) in PARAM_RANGES.items()}

        runs = _compile_and_run(params, n_seeds, trial.number)
        if runs is None:
            # 컴파일 실패 — surrogate model이 해당 영역을 회피하도록 낮은 점수 반환
            return -100.0

        found_avg = sum(r[0] for r in runs) / len(runs)
        done_avg  = sum(r[1] for r in runs) / len(runs)
        total_avg = sum(r[2] for r in runs) / len(runs)
        score = found_avg * W_FOUND + done_avg * W_DONE

        budget_max = round(params["DRONE_BUDGET_MIN"] + params["DRONE_BUDGET_GAP"], 6)

        # 추가 메트릭 저장 (CSV 출력에 사용)
        trial.set_user_attr("found_avg",       found_avg)
        trial.set_user_attr("done_avg",        done_avg)
        trial.set_user_attr("total_avg",       total_avg)
        trial.set_user_attr("DRONE_BUDGET_MAX", budget_max)
        for i, (f, d, t) in enumerate(runs):
            trial.set_user_attr(f"found_{i}", f)
            trial.set_user_attr(f"done_{i}",  d)
            trial.set_user_attr(f"total_{i}", t)

        print(
            f"[trial {trial.number:3d}] score={score:6.2f} "
            f"found={found_avg:.1f} done={done_avg:.1f}  "
            f"UNK={params['EID_UNKNOWN_WEIGHT']:.3f} "
            f"STA={params['EID_STALENESS_WEIGHT']:.3f} "
            f"DIS={params['EID_DISTANCE_WEIGHT']:.3f} "
            f"BUD=[{params['DRONE_BUDGET_MIN']:.3f},{budget_max:.3f}]",
            flush=True,
        )
        return score

    return objective


def _worker(args):
    """각 프로세스가 실행할 워커: SQLite 스토리지의 study에서 n_trials개 시도."""
    n_trials, n_seeds = args
    study = optuna.load_study(
        study_name=STUDY_NAME,
        storage=f"sqlite:///{DB_PATH}",
    )
    study.optimize(_make_objective(n_seeds), n_trials=n_trials,
                   show_progress_bar=False)


def _std(vals):
    if len(vals) <= 1:
        return 0.0
    avg = sum(vals) / len(vals)
    return (sum((x - avg) ** 2 for x in vals) / len(vals)) ** 0.5


def write_csv(study, n_seeds):
    """완료된 trial을 점수 내림차순으로 CSV에 저장한다."""
    run_cols = (
        [f"found_{i}" for i in range(n_seeds)] +
        [f"done_{i}"  for i in range(n_seeds)] +
        [f"total_{i}" for i in range(n_seeds)]
    )
    headers = [
        "EID_UNKNOWN_WEIGHT", "EID_STALENESS_WEIGHT", "EID_DISTANCE_WEIGHT",
        "DRONE_BUDGET_MIN", "DRONE_BUDGET_MAX",
        "found_avg", "done_avg", "total_avg",
        "score", "found_std", "done_std",
    ] + run_cols

    finished = [t for t in study.trials
                if t.value is not None and t.value > -50]
    finished.sort(key=lambda t: t.value, reverse=True)

    with open(RESULTS_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        for trial in finished:
            a = trial.user_attrs
            p = trial.params
            bmax = a.get("DRONE_BUDGET_MAX",
                         round(p["DRONE_BUDGET_MIN"] + p["DRONE_BUDGET_GAP"], 6))
            fv = [a.get(f"found_{i}", 0) for i in range(n_seeds)]
            dv = [a.get(f"done_{i}",  0) for i in range(n_seeds)]
            tv = [a.get(f"total_{i}", 0) for i in range(n_seeds)]
            row = (
                [f"{p['EID_UNKNOWN_WEIGHT']:.4f}",
                 f"{p['EID_STALENESS_WEIGHT']:.4f}",
                 f"{p['EID_DISTANCE_WEIGHT']:.4f}",
                 f"{p['DRONE_BUDGET_MIN']:.4f}",
                 f"{bmax:.4f}",
                 f"{a.get('found_avg', 0):.2f}",
                 f"{a.get('done_avg',  0):.2f}",
                 f"{a.get('total_avg', 0):.2f}",
                 f"{trial.value:.2f}",
                 f"{_std(fv):.2f}", f"{_std(dv):.2f}"]
                + fv + dv + tv
            )
            w.writerow(row)


def main():
    parser = argparse.ArgumentParser(
        description="EID 가중치 Bayesian Optimization (Optuna TPE)"
    )
    parser.add_argument("--trials",  type=int, default=200,
                        help="총 시도 횟수 (기본값: 200)")
    parser.add_argument("--workers", type=int, default=cpu_count(),
                        help="병렬 워커 수 (기본값: CPU 수)")
    parser.add_argument("--seeds",   type=int, default=N_SEEDS,
                        help="조합당 실행 횟수 (기본값: 5)")
    parser.add_argument("--resume",  action="store_true",
                        help="이전 study를 이어서 실행 (DB 유지)")
    args = parser.parse_args()

    n_workers = min(args.workers, args.trials)
    storage   = f"sqlite:///{DB_PATH}"

    # 새 실행 시 기존 DB 삭제
    if not args.resume and os.path.exists(DB_PATH):
        os.remove(DB_PATH)

    study = optuna.create_study(
        direction="maximize",
        study_name=STUDY_NAME,
        storage=storage,
        load_if_exists=args.resume,
        sampler=optuna.samplers.TPESampler(
            seed=42,
            # 워커가 동시에 시작하므로 첫 배치는 모두 랜덤 탐색이 됨.
            # n_startup_trials 이후부터 TPE(BO)가 이전 결과를 활용한다.
            n_startup_trials=max(30, n_workers),
        ),
    )

    existing = len([t for t in study.trials if t.value is not None])
    if args.resume:
        print(f"이전 study 재개 — 기존 완료 시도: {existing}개")

    print(f"총 시도: {args.trials}개  |  워커: {n_workers}개  |  seeds/시도: {args.seeds}")
    print(f"탐색 공간 (Grid Search보다 넓음):")
    for k, (lo, hi) in PARAM_RANGES.items():
        label = k if k != "DRONE_BUDGET_GAP" else "DRONE_BUDGET_GAP(→MAX=MIN+GAP)"
        print(f"  {label:<35} [{lo}, {hi}]")
    print(f"\nDB 경로:  {DB_PATH}")
    print(f"결과 CSV: {RESULTS_CSV}\n")

    # trials를 workers에게 균등 분배
    base  = args.trials // n_workers
    extra = args.trials % n_workers
    per_worker = [base + (1 if i < extra else 0) for i in range(n_workers)]
    worker_args = [(n, args.seeds) for n in per_worker]

    t_start = time.time()
    with Pool(n_workers) as pool:
        pool.map(_worker, worker_args)
    elapsed = time.time() - t_start

    # 최종 결과 수집 및 저장
    study = optuna.load_study(study_name=STUDY_NAME, storage=storage)
    write_csv(study, args.seeds)

    valid = [t for t in study.trials if t.value is not None and t.value > -50]
    if not valid:
        print("\n유효한 결과가 없습니다.")
        return

    best  = study.best_trial
    bf    = best.user_attrs.get("found_avg", 0)
    bd    = best.user_attrs.get("done_avg",  0)
    bmax  = best.user_attrs.get(
        "DRONE_BUDGET_MAX",
        round(best.params["DRONE_BUDGET_MIN"] + best.params["DRONE_BUDGET_GAP"], 6),
    )

    print(f"\n{'='*60}")
    print(f"총 실험 시간: {elapsed:.0f}s")
    print(f"완료 시도:    {len(valid)}개")
    print(f"최고 점수:    {best.value:.2f}  (found={bf:.1f}, done={bd:.1f})")
    print(f"최적 파라미터 (trial #{best.number}):")
    print(f"  #define EID_UNKNOWN_WEIGHT    {best.params['EID_UNKNOWN_WEIGHT']:.4f}")
    print(f"  #define EID_STALENESS_WEIGHT  {best.params['EID_STALENESS_WEIGHT']:.4f}")
    print(f"  #define EID_DISTANCE_WEIGHT   {best.params['EID_DISTANCE_WEIGHT']:.4f}")
    print(f"  #define DRONE_BUDGET_MIN      {best.params['DRONE_BUDGET_MIN']:.4f}")
    print(f"  #define DRONE_BUDGET_MAX      {bmax:.4f}")
    print(f"{'='*60}")
    print(f"결과 CSV: {RESULTS_CSV}")
    print(f"이어서 실행: python3 tune_bayes.py --resume --trials 100")


if __name__ == "__main__":
    main()
