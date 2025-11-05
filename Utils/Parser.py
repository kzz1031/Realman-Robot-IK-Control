import argparse
import numpy as np

def parse_args_record():
    parser=argparse.ArgumentParser()
    
    def str_to_ndarray(value):
        value = value.strip('"').strip("\\")
        value_list=value.split(',')
        if len(value_list)!=3:
            raise argparse.ArgumentTypeError("The length of pos and ori must be 3")
        return np.array(value_list).astype(float)
    
    def str2bool(v):
        if isinstance(v, bool):
            return v
        if v.lower() in ('yes', 'true', 't', '1'):
            return True
        elif v.lower() in ('no', 'false', 'f', '0'):
            return False
        else:
            raise argparse.ArgumentTypeError('Boolean value expected.')
    
    parser.add_argument("--record_flag",type=str2bool, default=False, help="data collection flag")
    parser.add_argument("--record_video_flag",type=str2bool, default=False, help="record video flag")
    parser.add_argument("--env_random_flag", type=str2bool, default=False, help="env random flag")