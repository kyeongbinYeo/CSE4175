#!/bin/bash

# 컴파일
g++ -O2 -o sender sender_20231566.cc netsim_lib.cc
if [ $? -ne 0 ]; then
    echo "컴파일 실패"
    exit 1
fi

total_cost=0
count=0

run_test() {
    local name=$1
    local input=$2
    local ber=$3
    local seed=$4
    local max_bytes=$5

    echo "=== $name ==="
    result=$(./netsim ./sender --input $input --output out.rx \
             --ber $ber --seed $seed --max_bytes $max_bytes 2>&1)
    echo "$result"

    cost=$(echo "$result" | grep "^cost:" | awk '{print $2}')
    echo "→ cost: $cost"
    echo ""

    total_cost=$((total_cost + cost))
    count=$((count + 1))
}

run_test "Validation 1 (BER=1e-6)" sherlock_holmes.txt 1e-6 1001 386832300
run_test "Validation 2 (BER=1e-5)" cat_bgm.mp3       1e-5 2002 316224000
run_test "Validation 3 (BER=1e-4)" cat_bgm.mp3       1e-4 3003 316224000
run_test "Validation 4 (BER=1e-3)" harry_potter.txt  1e-3 4004 44276800

avg=$((total_cost / count))
echo "=============================="
echo "평균 cost: $avg"
echo "=============================="
