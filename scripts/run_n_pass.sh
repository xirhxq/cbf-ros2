#!/bin/bash
# run_n_pass.sh - Run N consecutive fresh-container runs with current config,
# report PASS/FAIL on the 850m comm constraint for each, and overall pass rate.
#
# Usage: N=5 bash scripts/run_n_pass.sh
set -e
N="${N:-5}"
RESULTS_DIR="/tmp/pass_results"
mkdir -p "$RESULTS_DIR"
SUMMARY="$RESULTS_DIR/summary.txt"
: > "$SUMMARY"

echo "=== Running $N consecutive fresh-container runs (current config) ===" | tee -a "$SUMMARY"
python3 -c "import json;d=json.load(open('cbf/config/config.json'));cs=d['cbfs']['without-slack'];print(f'config: comm k={cs[\"comm-fixed\"][\"k\"]} coe={cs[\"comm-fixed\"][\"alpha\"][\"coe\"]} CU={cs[\"comm-fixed\"][\"consider-uncertainty\"]} route1={cs[\"route1\"][\"on\"]} q={d[\"estimator-in-loop\"][\"process-noise-mps\"]}')" | tee -a "$SUMMARY"

for i in $(seq 1 $N); do
  echo "" | tee -a "$SUMMARY"
  echo "=== Run $i/$N ===" | tee -a "$SUMMARY"
  LOG="/tmp/pass_run$i.log"
  bash scripts/run_fresh_container.sh > "$LOG" 2>&1 || true
  # Extract data path
  DATA=$(docker exec cbf-headless bash -lc 'grep -aoE "/home/developer[^ ]*data.json" /tmp/suav.log | tail -1' 2>/dev/null || echo "")
  if [ -z "$DATA" ]; then
    echo "Run $i: CRASH/no data" | tee -a "$SUMMARY"
    continue
  fi
  # Copy data.json out to host for analysis
  HOST_DATA="cbf/data/$(basename $(dirname $DATA))/data.json"
  # Analyze on host (mounted)
  python3 - "$HOST_DATA" "$i" >> "$SUMMARY" << 'PYEOF'
import json, math, sys
path, runidx = sys.argv[1], sys.argv[2]
with open(path) as f: data = json.load(f)
states = data['state']
last_up=0
for i,st in enumerate(states):
    if st.get('update'): last_up=i
edges=set()
for st in states[::30]:
    for r in st['robots']:
        for name in r.get('cbfNoSlack',{}):
            if 'fixedCommCBF(' in name:
                inner=name.split('(')[1].rstrip(')')
                if inner.startswith('base-'): edges.add((r['id'],('base',int(inner.split('-')[1]))))
                elif inner.startswith('#'): edges.add((r['id'],int(inner.lstrip('#'))))
bases={0:(-1550.,-300.),1:(-1550.,0.),2:(-1550.,300.)}
maxd=0; maxe=None; viol=0
for i,st in enumerate(states[:last_up+1]):
    pos={r['id']:(r['state']['x'],r['state']['y']) for r in st['robots']}
    fv=False
    for (a,b) in edges:
        if isinstance(b,tuple):
            if a not in pos: continue
            bx,by=bases[b[1]]; d=math.hypot(pos[a][0]-bx,pos[a][1]-by)
        else:
            if a not in pos or b not in pos: continue
            d=math.hypot(pos[a][0]-pos[b][0],pos[a][1]-pos[b][1])
        if d>maxd: maxd=d;maxe=(a,b)
        if d>850: fv=True
    if fv: viol+=1
minsep=1e9
for st in states[:last_up+1]:
    pos={r['id']:(r['state']['x'],r['state']['y']) for r in st['robots']}
    ids=sorted(pos)
    for x in range(len(ids)):
        for y in range(x+1,len(ids)):
            d=math.hypot(pos[ids[x]][0]-pos[ids[y]][0],pos[ids[x]][1]-pos[ids[y]][1])
            if d<minsep: minsep=d
cov_t = states[last_up]['runtime']
status = "PASS" if maxd<=850 and minsep>=10 else "FAIL"
print(f"Run {runidx}: {status} | max_link={maxd:.2f}m {maxe} | >850={viol}/{last_up+1} | cov@{cov_t}s | minsep={minsep:.2f}m")
PYEOF
done

echo "" | tee -a "$SUMMARY"
echo "=== SUMMARY ===" | tee -a "$SUMMARY"
grep -E "^Run [0-9]+: (PASS|FAIL)" "$SUMMARY"
echo "" | tee -a "$SUMMARY"
PASS=$(grep -c "^Run [0-9]+: PASS" "$SUMMARY" || echo 0)
echo "PASS count: $PASS / $N" | tee -a "$SUMMARY"
