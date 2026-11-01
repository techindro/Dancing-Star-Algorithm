#!/bin/sh
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 solution/solve.cpp -o /tmp/dancing_solve
fail=0
for f in examples/*.in; do
  exp="${f%.in}.out"
  got=$(/tmp/dancing_solve < "$f")
  if [ "$got" = "$(cat "$exp")" ]; then echo "OK   $f"; else echo "FAIL $f (got $got)"; fail=1; fi
  chk=$(python3 benchmarks/verify.py "$f")
  [ "$chk" = "$got" ] || { echo "CHECKER MISMATCH on $f ($chk)"; fail=1; }
done
exit $fail
