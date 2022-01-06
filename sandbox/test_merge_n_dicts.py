"""Small script to test the merge_n_dicts method."""

from openai_ros.common.functions import merge_n_dicts

if __name__ == "__main__":
    dict_1 = {"test": 2, "test1": 3, "test3": 5}
    dict_2 = {"test4": 2, "test1": 5, "test5": 5}
    dict_merged = merge_n_dicts(dict_1, dict_2, order=["test", "test5"])
    print(dict_merged)
