python3 -m set-filter 0

python3 -m sim-retest \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/min dist 200/wall wo filter" \
    *_bin_* \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/retest_cart_res_0/wall_wo_filter"
python3 -m sim-retest \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/min dist 200/blanket wo filter" \
    *_bin_* \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/retest_cart_res_0/blanket_wo_filter"

python3 -m set-filter 1

python3 -m sim-retest \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/min dist 200/wall w filter" \
    *_bin_* \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/retest_cart_res_0/wall_w_filter"
python3 -m sim-retest \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/min dist 200/blanket w filter" \
    *_bin_* \
    "/mnt/c/Users/wyatt/Documents/GVSU/Thesis/august_test_data/idc_aug_23/retest_cart_res_0/blanket_w_filter"