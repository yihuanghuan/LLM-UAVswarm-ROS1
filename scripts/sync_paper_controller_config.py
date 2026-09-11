#!/usr/bin/env python3
"""Generate/check ROS1 controller parameters from the sealed typed policy."""
from pathlib import Path
import argparse,sys,yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/lfs_policy'))
from lfs_policy import load_paper_policy
p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');args=p.parse_args()
config=root/'src/ladrc_controller/config'
params=yaml.safe_load((config/'paper_base.yaml').read_text())
params.update(load_paper_policy(root/'src/lfs_policy/config/lfs_policy.paper_current.yaml').controller.ros_parameters())
params['control_mode']='ladrc_acceleration'
target=config/'ladrc_params.yaml'
if args.check:
    if yaml.safe_load(target.read_text())!=params:raise SystemExit('Controller config differs from frozen policy; run scripts/sync_paper_controller_config.py')
    print('Frozen controller configuration verified')
else:target.write_text('# Generated from sealed paper baseline; no algorithm parameter tuning.\n'+yaml.safe_dump(params,sort_keys=False))
