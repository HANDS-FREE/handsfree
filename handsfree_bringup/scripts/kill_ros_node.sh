#!/usr/bin/env bash

red="\033[1;31m"
green="\033[1;33m"
yellow="\033[1;33m"
color_end="\033[0m"
sleep .3
echo ""
sleep .3
echo ""
sleep .3
echo -e $green"将为你杀死所有隐藏 ros 节点"$color_end
sleep .3
echo -e $green"开始搜索 ros 节点"$color_end
sleep .3
echo -e $green"."$color_end
sleep .3
echo -e $green".."$color_end
sleep .3
echo -e $green"..."$color_end
sleep .3
echo -e $green"...."$color_end
sleep .3
echo -e $green"....."$color_end
sleep .3
echo -e $green"......"$color_end
sleep .3
echo ""
sleep .3
echo ""
sleep .3
echo -e "正在杀死隐藏 ros 节点"
sleep .3
echo ""
ps -ef |grep launch|grep -v grep|awk '{print $2}'|xargs kill -9
echo ""
sleep .3
echo "3秒 后将自动关闭该终端"
sleep .5
ps -ef |grep opt|grep -v grep|awk '{print $2}'|xargs kill -9
echo ""
sleep .5
echo "2秒 后将自动关闭该终端"
sleep .5
ps -ef |grep ".log"|grep -v grep|awk '{print $2}'|xargs kill -9
echo ""
sleep .5
echo "1秒 后将自动关闭该终端"
sleep 1
