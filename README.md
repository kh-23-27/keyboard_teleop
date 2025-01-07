# keyboard_teleop
[![test](https://github.com/kh-23-27/keyboard_teleop/actions/workflows/test.yml/badge.svg)](https://github.com/kh-23-27/keyboard_teleop/actions/workflows/test.yml)
## 概要
このパッケージはリモートで対象物をキーボード操作するROS2のパッケージです.

## ノード
- keyboard_teleop
    - 押されたキーに応じた速度をトピック'cmd_vel'にパブリッシュします.
## このパッケージを使用する前に
### 使用環境
- Ubuntu 22.04 LTS
- ROS 2 Humble
### セットアップ
- ROS 2インストール  
ROS 2は各自でインストールしてください.
詳細は[インストール](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)を確認してください.   
## 実行例
- 端末１
以下のコマンドを実行します.
```
$ ros2 run keyboard_teleop keyborad_teleop
```
上のコマンドを実行したターミナルに以下の出力がなされます.
```
---------------------------
Moving around:

   q    w    e
   a         d
   z    s    c

j: Increase speed
k: Decrease speed
l: Reset speed to 0.5
---------------------------
```
wを押すと前進、sを押すと後退、aを押すと左に、dを押すと右に動きます.  
qを押すと左上に、eを押すと右上に、zを押すと左下に、cを押すと右下に動きます.

- 端末2
以下のコマンドでcmd_velに送られている速度指令の内容を確認できます.
```
$ ros2 topic echo /cmd_vel
---
linear:
  x: 5.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

# ライセンス
- このパッケージの[test.yml](https://github.com/kh-23-27/keyboard_teleop/blob/main/.github/workflows/test.yml)では, [コンテナ](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2)（by Ryuichi Ueda）を利用しています.
- このパッケージはApache License, Version 2.0に基づき公開されています.
- 詳細は[LICENSE](https://github.com/kh-23-27/keyboard_teleop/blob/main/LICENSE)を確認してください。
- © 2025 Kenta Hirachi
