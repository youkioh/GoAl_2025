#!/bin/bash

# 기존 output.txt 삭제 (선택)
> output.txt  # 파일 비우기 (또는 rm output.txt)

# 100번 실행
for i in {1..100}
do
    ./MRTA >> log.txt
    echo "$i th iteration completed"
done

# 결과 종합
awk '{ active += $1; completed += $2 } END { print "Total active:", active; print "Total completed:", completed; print "Grand total:", active + completed }' output.txt

