# Copyright 2022 IMCL, Department of Computing
# Department of Computing, Hong Kong Polytechnic University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class BaseTask():
    """
    Base task interface for team-level
    This is a basic class and should be extended before instantiation.
    """
    def __init__(self):
        pass

    def star_task(self):
        pass

    def stop_task(self):
        pass

    def set_state(self):
        pass

    def get_state(self):
        pass

    def reset(self):
        pass
