#!/usr/bin/python
# usage: python2 bag_batch_process.py --yaml=xxx.yaml --option=filter_bag

import os
import argparse
import yaml

plat_data_pair_list = [
    ('apollo', 'campus_cyt_1_liosam')
]

bag_path_download = '/home/xchu/data/ramlab_dataset/'
bag_path_list = [
    bag_path_download + 'apollo_cyt_1_2022-06-19-14-26-48.bag'
]

algorithm_type_list = [
    # '0',  # liosam
    # '1',  # liomapping
    # '2'  # fastlio
    # '3',  # fasterlio
    # '4',  # liliom
    # '6'  # liosam_loop
    '7'  # fastlio_loop
]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    for i, pd_pair in enumerate(plat_data_pair_list):
        bag_path_dir = bag_path_list[i]
        # for bag_path_dir in bag_path_list:
        for algorithm_type in algorithm_type_list:
            command = 'roslaunch lio_sam_6axis fp_outdoors.launch' + \
                      ' bag_path:=' + bag_path_dir + \
                      ' sequence:=' + pd_pair[1]
            # print("================run algorithm at %s sequence==================" % (
            print(command)
            os.system(command)
