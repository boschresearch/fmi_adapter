#!/bin/bash

find . \( -name *.cpp -o -name *.h -o -name *.hpp \) ! -exec clang-format -style=file -i {} \;
