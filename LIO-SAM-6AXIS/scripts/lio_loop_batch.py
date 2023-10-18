#!/usr/bin/python
# usage: python2 bag_batch_process.py --yaml=xxx.yaml --option=filter_bag

import os
import argparse
import yaml

plat_data_pair_list = [
    ('hkust', 'campus')
    # ('hkust', 'campus_day_ouster128')
    # ('hilti', 'exp01'),
    # ('hilti', 'exp02'),
    # ('handheld', 'exp04'),
    # ('handheld', 'exp05'),
    # ('handheld', 'exp06')
    # ('hilti', 'exp07'),
    # ('hilti', 'exp09')
    # ('hilti', 'exp11'),
    # ('hilti', 'exp15')
    # ('hilti', 'exp21')
]

bag_path_download = '/home/xchu/data/ramlab_dataset/'
bag_path_list = [
    bag_path_download + 'hkust_20201105full.bag'
    # bag_path_download + '20220226_campus_road_day_ref.bag'
    # bag_path_download + 'exp01_construction_ground_level.bag',
    # bag_path_download + 'exp02_construction_multilevel.bag',
    # bag_path_download + 'exp_04_construction_upper_level_easy_2_2022-03-03-11-48-59.bag',
    # bag_path_download + 'exp_05_construction_upper_level_easy_2022-03-03-11-46-10.bag',
    # bag_path_download + 'exp_06_construction_upper_level_hard_2022-03-03-12-08-37.bag'
    # bag_path_download + 'exp07_long_corridor.bag',
    # bag_path_download + 'exp09_cupola.bag'
    # bag_path_download + 'exp11_lower_gallery.bag',
    # bag_path_download + 'exp15_attic_to_upper_gallery.bag'
    # bag_path_download + 'exp21_outside_building.bag'
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
            command = 'roslaunch lio_sam_6axis test_vlp16.launch' + \
                      ' bag_path:=' + bag_path_dir + \
                      ' sequence:=' + pd_pair[1]
            # print("================run algorithm at %s sequence==================" % (
            print(command)
            os.system(command)
