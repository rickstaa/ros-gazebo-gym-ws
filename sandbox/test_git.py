import pygit2
from tqdm import tqdm


# class MyRemoteCallbacks(pygit2.RemoteCallbacks):
#     def __init__(self):
#         super().__init__()
#         self.pbar = tqdm()

#     def transfer_progress(self, statsTransferProgress):
#         # print(statsTransferProgress.received_objects)
#         self.pbar.total = statsTransferProgress.total_objects
#         self.pbar.n = statsTransferProgress.received_objects
#         self.pbar.refresh()


repo_url = "https://bitbucket.org/theconstructcore/spawn_robot_tools.git"
pygit2.clone_repository(
    repo_url, "./" + repo_url.split("/")[-1].replace(".git", ""), bare=True
)
