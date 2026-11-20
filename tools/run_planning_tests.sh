#!/usr/bin/env bash
# Planning suite (tutorial step 4): POPF on every problem in pddl/problems/.
#   - timeout 60 s per run; solvable problems are run 10 times for the mean planning time
#   - plans saved in pddl/results/<problem>.plan
#   - pddl/results/planning_results.csv: id, solved, makespan, actions, metres, optimum_m,
#     plan_time_ms_mean, runs, crashes
# Usage: bash tools/run_planning_tests.sh            (from anywhere)
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$WS/src/go2_patrol_mission"
DOMAIN="$PKG/pddl/patrol_domain_v2.pddl"
PROBLEMS="$PKG/pddl/problems"
RESULTS="$PKG/pddl/results"
ONTO="$PKG/config/ontology.yaml"
TIMEOUT=60
REPS=10
PY=/usr/bin/python3            # system python (has yaml); venvs in PATH do not

POPF="$(ls /opt/ros/foxy/lib/popf/popf 2>/dev/null)"
[ -x "$POPF" ] || { echo "popf non trovato in /opt/ros/foxy/lib/popf"; exit 1; }
export LD_LIBRARY_PATH="/opt/ros/foxy/lib:${LD_LIBRARY_PATH:-}"
SPEED=$($PY -c "import yaml;print(yaml.safe_load(open('$ONTO'))['robot']['speed'])")

mkdir -p "$RESULTS"
CSV="$RESULTS/planning_results.csv"
echo "id,solved,makespan,actions,metres,optimum_m,plan_time_ms_mean,runs,crashes" > "$CSV"

now_ms() { echo $(( $(date +%s%N) / 1000000 )); }

for pddl in "$PROBLEMS"/p*.pddl; do
  name="$(basename "$pddl" .pddl)"
  id="${name%%_*}"
  plan="$RESULTS/$name.plan"
  optimum=$($PY "$WS/tools/optimal_tours.py" "$ONTO" "${pddl%.pddl}.yaml" | awk '{print $2}')

  total=0; runs=0; crashes=0; solved=no
  for ((i = 1; i <= REPS; i++)); do
    t0=$(now_ms)
    timeout "$TIMEOUT" "$POPF" "$DOMAIN" "$pddl" > "$RESULTS/$name.out" 2>&1
    rc=$?
    t1=$(now_ms)
    runs=$((runs + 1)); total=$((total + t1 - t0))
    # popf exits 1 on ";; Problem unsolvable!" and 124 on timeout: both are normal outcomes.
    # Only a signal (exit code >= 128, e.g. 134/139 core dump) is a crash: count it and retry.
    if [ $rc -ge 128 ] && [ $rc -ne 124 ]; then
      crashes=$((crashes + 1))
      runs=$((runs - 1)); total=$((total - t1 + t0)); i=$((i - 1))
      [ $crashes -ge 5 ] && break
      continue
    fi
    if grep -q '^[0-9.]*: (' "$RESULTS/$name.out"; then
      solved=yes
      [ $i -eq 1 ] && grep '^[0-9.]*: (' "$RESULTS/$name.out" > "$plan"
    else
      # no plan: one run is enough (timeouts would take REPS minutes)
      break
    fi
  done

  if [ "$solved" = yes ]; then
    read -r makespan actions metres < <(awk -v sp="$SPEED" '
      { start = $1 + 0; dur = $NF; gsub(/[\[\]]/, "", dur); end = start + dur
        if (end > ms) ms = end; n++
        if ($2 == "(navigate") m += dur * sp }
      END { printf "%.2f %d %.1f\n", ms, n, m }' "$plan")
  else
    makespan=-; actions=0; metres=-
    rm -f "$plan"
  fi
  mean=$(( runs > 0 ? total / runs : 0 ))
  echo "$id,$solved,$makespan,$actions,$metres,$optimum,$mean,$runs,$crashes" >> "$CSV"
  printf '%-24s solved=%-3s makespan=%-8s actions=%-3s metres=%-6s optimum=%-6s %6s ms (%d run, %d crash)\n' \
    "$name" "$solved" "$makespan" "$actions" "$metres" "$optimum" "$mean" "$runs" "$crashes"
done
echo "Risultati: $CSV"
