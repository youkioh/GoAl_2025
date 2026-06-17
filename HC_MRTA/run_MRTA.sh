#!/bin/bash

# 실행 횟수
TEST_COUNT=100
# 코어 최대한 활용
CORE_COUNT=$(nproc)
ITER=$((TEST_COUNT / CORE_COUNT))

# # log, output 파일 초기화
rm -rf build/log_*.txt  # 모든 log 파일 비우기
rm -rf build/output.txt  # output 파일 비우기

# # 각 코어별로 MRTA 실행
for i in $(seq 1 $TEST_COUNT)
do
    build/MRTA >> build/log_$i.txt &
    sleep 1
done
wait

# 로그 파일에서 active와 completed 추출
# Extract Active and Completed task counts from logs
for log_file in build/log_*.txt; do
    # Use awk to directly extract the values and calculate founded
    awk '
    /Active task:/ {
        match($0, /Active task:[[:space:]]*([0-9]+)/, a)
        if (a[1] != "") active = a[1]
    }
    /Completed task/ {
        match($0, /Completed task[[:space:]]*:[[:space:]]*([0-9]+)/, b)
        if (b[1] != "") completed = b[1]
    }
    END {
        if (active != "" && completed != "") 
            print (active+completed) "," completed
    }
    ' "$log_file" >> build/output.txt
done