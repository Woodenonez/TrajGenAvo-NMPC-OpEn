# This is a template of comments, you cannot run it.
# A function type can by vis (visualization), pre (preparation), run, get, set, misc.

import os

'''
File info:
    Name    - [XXX]
    Author  - [XXX]
    Date    - (from)[XXX] -> (to)[XXX]
    Ref     - (literature)[XXX]
            - (website)[XXX]
    Exe     - (executable)[Yes]
File description:
    (What does this file do?)
File content:
    ClassA      <class> - (Basic usage).
    ClassB      <class> - (Basic usage).
    function_A  <func>  - (Basic usage).
    function_B  <func>  - (Basic usage).
Comments:
    (Things worthy of attention.)
'''

def function_A(x:int, y:int):
    '''
    Description:
        (What does this function do?)
    Arguments:
        x <type> - (Description).
        y <type> - (Description).
    Return:
        z <type> - (Description).
    Comments:
        (Things worthy of attention.)
    '''
    z = x+y
    return z

def function_B():
    pass

class ClassA():
    '''
    Description:
        (What does this class do?)
    Arguments:
        x <type> - (Description).
        y <type> - (Description).
    Attributes:
        attr1 <type> - (Description).
        attr2 <type> - (Description).
    Functions
        func1 <type> - (Description).
        func2 <type> - (Description).
    Comments:
        (Things worthy of attention.)
    '''
    def __init__(self, x, y):
        self.attr1 = x
        self.attr2 = y

    def func1(self):
        print('This is function 1.')

    def func2(self):
        print('This is function 2.')

class ClassB():
    pass

if __name__ == '__main__':
    do_something = True # this part for testing, if missing then 'Exe'=No
