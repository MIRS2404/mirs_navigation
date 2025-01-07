# README

## 今後検証すること

bt_sample.xmlいらないんじゃないか？(なくても動きそうだったし... )
nav2_params.yamlのcontroller_server内のvelocity_smootherを消したけどこれで動くか検証する。
同じところにspeed_limit_ratioを追加したがこれが影響していないかも検証する

## 今後修正が必要だとわかっていること

/global_costmapが表示されない問題の解決

## 全ての起動手順
```
cd ~/mirs2404_ws
source install/setup.bash
ros2 launch mirs_navigation mirs_navigation.launch.py

ros2 launch mirs_navigation mirs2404.launch.py
```
