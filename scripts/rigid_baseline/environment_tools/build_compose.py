"""Build the declared Docker environment; no research data or robot commands."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);a=p.parse_args()
out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);start=time.monotonic()
try:
 with (out/'build.log').open('x') as log:
  subprocess.run(['docker','compose','--progress','plain','-f','docker/compose.yaml','build','piakf'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=3040)
 image=json.loads(subprocess.check_output(['docker','image','inspect','piakf:jazzy-mujoco3.3.7'],timeout=15))
 (out/'image.json').write_text(json.dumps(image,indent=2))
 result=dict(passed=True,image_id=image[0]['Id'],wall_seconds=time.monotonic()-start,scientific_calculation=False)
except Exception as e:
 result=dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-start,scientific_calculation=False)
 (out/'metrics.json').write_text(json.dumps(result,indent=2));raise
(out/'metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
