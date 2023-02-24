# seed_rviz_plugin

## 概要    
本パッケージは[SEED-Mover](https://www.seed-solutions.net/?q=seed_jp/node/73)を自律移動させる際に必要となる、    
移動目標ポイント(waypoint)の追加/編集/削除ができるGUIツールです。
<div align="center">
<img src="https://user-images.githubusercontent.com/12426780/221104629-fdec5f14-ffd7-4547-8626-b44aca94fde2.gif" width="50%">
</div>

## 事前準備    
[task_programmer](https://github.com/seed-solutions/task_programmer)に依存しています。
事前に同パッケージの環境構築を実施して下さい。

## 起動プログラムとRVizの設定    
1. 自律移動に必要なプログラムを起動(下記はシミュレーションで実行する場合の例)
    ```
    roslaunch task_programmer simulation.launch
    ```
2. waypointを管理するノードを起動（1.のlaunchファイルに記載している場合は不要）
    ```
    rosrun seed_rviz_plugin wp_marker_manager_node
    ```
3. RVizの起動
    ```
    roslaunch task_programmer view.launch
    ```
4. RVizの設定追加（事前設定されている場合は不要）
    * InteractiveMarkersの追加    
    ``Add``ボタン->``By topic``から``/wp_marker``の``InteractiveMarkers``を選択->``Show Descriptions``のチェックを外す    

        <img src="https://user-images.githubusercontent.com/12426780/221096064-8eec4a40-49b9-4d0a-a87b-1a74cf3ea002.png" width="50%">

    * PointList Panelの追加    
    ``Panels``->``Add New Panel``->``seed_rviz_plugin``の``PointList``を選択->``PointList``パネルをドラッグ&ドロップでRViz画面左に全面表示    

        <img src="https://user-images.githubusercontent.com/12426780/221096240-a2db9650-fdb9-4d07-9dee-aef3df441c29.png" width="50%">

## 画面説明と操作手順    
<img src="https://user-images.githubusercontent.com/12426780/221097092-1a48fdcc-d0af-4dad-a066-2357c8386d0b.png" width="50%">

* 参照ディレクトリ    
``読込``ボタンを押したときに設定される、ポイントの読込/保存先ディレクトリが表示されます。    
初期設定は``/config/maps``で、``{task_programmerのディレクトリ}/config/maps/waypoints.yaml``に記述されているYAML形式のファイルを扱います。

    参照ディレクトリは[rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure)で管理されており、地図の読込時や切替時に自動で設定されます。

* ポイントリスト    
読み込んだポイント、編集中ポイントの一覧が表示されます。    
表示にチェックを入れるとRViz上にマーカーが表示され、編集にチェックを入れると編集可能となります。    
**(表示にチェックが入っていないと、編集もチェックを入れられません)**    
表示もしくは編集のヘッダーを選択すると、全てのチェックボックスがONもしくはOFFされます。
    
    数値は直接入力かマウススクロール/矢印キーで変更できます。    
編集された名称や値は背景色が変わり、同時にRViz上のマーカーにも反映されます。

    <img src="https://user-images.githubusercontent.com/12426780/221098656-53a9667e-d278-40aa-86a9-b2840291c220.png" width="60%">

* マーカー    
ポイントリストのデータを基に[InteractiveMarker](http://wiki.ros.org/interactive_markers)で表示されます。    

    編集にチェックが入っている場合、マーカー中心部をドラッグ＆ドロップすることで位置を変更でき、    
周りのリングをドラッグ＆ドロップすることで姿勢を変更できます。    
マーカー情報はポイントリストにも反映されます。

    <img src="https://user-images.githubusercontent.com/12426780/221098762-40a46dd6-f286-4cd5-a265-598e34168931.png" width="30%">

* 読込ボタン    
登録されているポイントデータを読み込みます。    
``読込``ボタンを押すと、他のボタンも選択可能になります。

    <img src="https://user-images.githubusercontent.com/12426780/221099155-6434016e-e404-42f5-8038-c82501c3a0a3.png" width="50%">

* 保存ボタン    
ポイントを保存します。    
リストに表示されているポイント情報を``waypoints.yaml``に上書きします。

* 削除ボタン    
ポイントを削除します。    
ボタンを押すと、ポイント名一覧が記載されたポップアップが表示されます。    
任意のポイントを選択して``OK``ボタンを押すと、選択したポイントが削除されます。

    <img src="https://user-images.githubusercontent.com/12426780/221099191-d4090740-ecb5-4fb9-b534-82ac54ceeafd.png" width="10%">

* 行追加ボタン    
ポイントを追加します。    
ボタンを押すと、ロボットの現在位置がポイントリストに追加されます。
