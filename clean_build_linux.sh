#!/bin/bash
find ./ -name "*.user" | xargs rm -f
find ./ -name "*~" | xargs rm -f
find ./ -name "*.pyc" | xargs rm -f

