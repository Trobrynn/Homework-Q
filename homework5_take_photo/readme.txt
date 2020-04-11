利用opencv和百度api实现：

首先利用百度api的wave_detect节点进行挥手识别，可以跟踪识别出人所在位置
此时对应的节点间结构关系如rosgraph_wave.png所示
在识别到挥手之后关闭挥手检测节点进行人脸检测，询问是否拍照
此时对应的节点间关系如rosgraph.png所示
确认需要拍照之后通过语音提醒使得使用者处于图片中央并take photo


录屏无挥手直接拍照传至仓库：
https://github.com/Trobrynn/tro-repository/blob/master/take_photo.mkv

挥手加拍照录屏传至仓库：
https://github.com/Trobrynn/tro-repository/blob/master/take_photo_wave.mkv
