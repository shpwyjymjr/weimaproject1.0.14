# calibration

v1.0    使用calibration那一瞬间的深度图像作为标定差值的标准帧，温漂矫正没有加，方差的噪声没有消除，所以均值会随着温度逐步提升，方差也较大

v1.0.10  改进标准帧，先测试10000帧图像的均值作为标准帧的标定效果。

v1.0.11  计算温漂系数，修正对应帧的温漂系数，然后在1w张图像中每隔10帧抽样，取这1000帧数据的均值

v1.0.12  通過最小二乘法計算溫度和距離構成的超定方程，得到溫漂系數，然后在得到溫漂系數之后，計算那個時刻及其以后的10幀數據的溫度均指和深度數據均值，再減去給出的標準距離得到校正矩陣，矯正矩陣也是對應這個溫度均值而言的，之后利用校正矩陣得到的修正數據是基于那個10幀數據的均值的溫度的，所以數據溫漂修正之后得到的是那個溫度均值對應的深度數據，然后減去那個校正矩陣，得到最終的修正數據。

v1.0.13  提交此版本的說明，并且提交上一次的更改,tips:此時上傳的是1.0.12版本，1.0.13版本還未開始寫
         在計算溫漂系數之前，加入篩選功能，丟掉前50幀數據，中間的數據采樣的的時候丟掉溫度波動大于6攝氏度的數據幀，不再存儲每幀的中心點，而是每幀數據的均值。
		 
v1.0.14  使用新的修正方式

