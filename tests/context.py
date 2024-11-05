##########################################################################
# This is the suggestted way for importing the main package as stated in 
# https://docs.python-guide.org/writing/structure/
# use this main module in any other individual tests as :
# from context import <package>
###########################################################################


import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import backend
import algorithms
