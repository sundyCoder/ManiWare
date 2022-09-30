# Copyright 2022 IMCL, Department of Computing
# Department of Computing, Hong Kong Polytechnic University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from threading import Event


class Optimizer():

    def __init__(self, settings: dict=None):
        if settings is not None and not isinstance(settings, dict):
            raise ValueError("settings must be either a dictionary or None")
        
        self.settings = settings if settings is not None else {}
        self._halt_event = None

    def initialize(self, op_type, halt_event: Event=None):
        # store variables
        self.op_type = op_type
        self._halt_event = halt_event
    
    def optimize(self):
        raise NotImplementedError

    def get_result(self):
        raise NotImplementedError
    
    def get_cost(self):
        raise NotImplementedError


