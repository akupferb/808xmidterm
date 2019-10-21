##
##  Distributed under our modified Boost Software License.
##  Version 1.0 (see accompanying file LICENSE)
##
##
##  author     Ryan Bates
##  copyright  Copyright ARL 2019
##  date       10/21/2019
##  version    1.0
##
##              System Test
##
##


#!/bin/sh
#         executable [start][---end position--] [-----obstacle 1--rad] [-----obstacle 2--rad]
./build/app/shell-app 0 0 0 2050.0 30.0 -1200.0 100000.1, 0.2, 0.3 0.5 100000.5, 0.6, 0.7 1.0


# Other tests for the curious:





# Blocked, impossible path

#         executable [start][---end position--] [---obstacle 1--] [---obstacle 2--]
# ./build/app/shell-app 0 0 0 2050.0 30.0 -1200.0 0.1, 0.2, 0.3 0.5 0.5, 0.6, 0.7 1.0




# Path with unspecified obstacles

# ./build/app/shell-app 0 0 0 2050.0 30.0 -1200.0


