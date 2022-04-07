\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[23,0] + 0 x[23,1] + 0 x[23,2] + 0 x[23,3] + 0 x[23,4]
   + 0 x[23,5] + 0 x[23,6] + 0 x[23,7] + 0 x[23,8] + 0 x[23,9] + 0 x[23,10]
   + 0 x[23,11] + 0 x[23,12] + 0 x[23,13] + 0 x[23,14] + 0 x[23,15]
   + 0 x[23,16] + 0 x[23,17] + 0 x[23,18] + 0 x[23,19] + 0 x[23,20]
   + 0 x[23,21] + 0 x[23,22] + 0 x[23,23] + 0 w[0,0] + 0 w[23,0]
   + 0 w[23,1] + 0 w[23,22] + 0 w[23,23] + 0 v[0,0] + 0 v[1,0] + 0 v[2,0]
   + 0 v[3,0] + 0 v[4,0] + 0 v[5,0] + 0 v[6,0] + 0 v[7,0] + 0 v[8,0]
   + 0 v[9,0] + 0 v[10,0] + 0 v[11,0] + 0 v[12,0] + 0 v[13,0] + 0 v[14,0]
   + 0 v[15,0] + 0 v[16,0] + 0 v[17,0] + 0 v[18,0] + 0 v[19,0] + 0 v[20,0]
   + 0 v[21,0] + 0 v[22,0] + 0 v[23,0] + 0 v[23,1] + 0 v[23,2] + 0 v[23,3]
   + 0 v[23,4] + 0 v[23,5] + 0 v[23,6] + 0 v[23,7] + 0 v[23,8] + 0 v[23,9]
   + 0 v[23,10] + 0 v[23,11] + 0 v[23,12] + 0 v[23,13] + 0 v[23,14]
   + 0 v[23,15] + 0 v[23,16] + 0 v[23,17] + 0 v[23,18] + 0 v[23,19]
   + 0 v[23,20] + 0 v[23,21] + 0 v[23,22] + 0 v[23,23] + 0 service_time[23]
   + start_service[23]
Subject To
 MaxNumberVehiclesStart: x[0,1] + x[0,2] + x[0,3] + x[0,4] + x[0,5]
   + x[0,6] + x[0,7] + x[0,8] + x[0,9] + x[0,10] + x[0,11] + x[0,12]
   + x[0,13] + x[0,14] + x[0,15] + x[0,16] + x[0,17] + x[0,18] + x[0,19]
   + x[0,20] + x[0,21] + x[0,22] + x[0,23] <= 3
 OutFlowVehicles[1]: x[1,1] + x[1,2] + x[1,3] + x[1,4] + x[1,5] + x[1,6]
   + x[1,7] + x[1,8] + x[1,9] + x[1,10] + x[1,11] + x[1,12] + x[1,13]
   + x[1,14] + x[1,15] + x[1,16] + x[1,17] + x[1,18] + x[1,19] + x[1,20]
   + x[1,21] + x[1,22] + x[1,23] = 1
 OutFlowVehicles[2]: x[2,1] + x[2,2] + x[2,3] + x[2,4] + x[2,5] + x[2,6]
   + x[2,7] + x[2,8] + x[2,9] + x[2,10] + x[2,11] + x[2,12] + x[2,13]
   + x[2,14] + x[2,15] + x[2,16] + x[2,17] + x[2,18] + x[2,19] + x[2,20]
   + x[2,21] + x[2,22] + x[2,23] = 1
 OutFlowVehicles[3]: x[3,1] + x[3,2] + x[3,3] + x[3,4] + x[3,5] + x[3,6]
   + x[3,7] + x[3,8] + x[3,9] + x[3,10] + x[3,11] + x[3,12] + x[3,13]
   + x[3,14] + x[3,15] + x[3,16] + x[3,17] + x[3,18] + x[3,19] + x[3,20]
   + x[3,21] + x[3,22] + x[3,23] = 1
 OutFlowVehicles[4]: x[4,1] + x[4,2] + x[4,3] + x[4,4] + x[4,5] + x[4,6]
   + x[4,7] + x[4,8] + x[4,9] + x[4,10] + x[4,11] + x[4,12] + x[4,13]
   + x[4,14] + x[4,15] + x[4,16] + x[4,17] + x[4,18] + x[4,19] + x[4,20]
   + x[4,21] + x[4,22] + x[4,23] = 1
 OutFlowVehicles[5]: x[5,1] + x[5,2] + x[5,3] + x[5,4] + x[5,5] + x[5,6]
   + x[5,7] + x[5,8] + x[5,9] + x[5,10] + x[5,11] + x[5,12] + x[5,13]
   + x[5,14] + x[5,15] + x[5,16] + x[5,17] + x[5,18] + x[5,19] + x[5,20]
   + x[5,21] + x[5,22] + x[5,23] = 1
 OutFlowVehicles[6]: x[6,1] + x[6,2] + x[6,3] + x[6,4] + x[6,5] + x[6,6]
   + x[6,7] + x[6,8] + x[6,9] + x[6,10] + x[6,11] + x[6,12] + x[6,13]
   + x[6,14] + x[6,15] + x[6,16] + x[6,17] + x[6,18] + x[6,19] + x[6,20]
   + x[6,21] + x[6,22] + x[6,23] = 1
 OutFlowVehicles[7]: x[7,1] + x[7,2] + x[7,3] + x[7,4] + x[7,5] + x[7,6]
   + x[7,7] + x[7,8] + x[7,9] + x[7,10] + x[7,11] + x[7,12] + x[7,13]
   + x[7,14] + x[7,15] + x[7,16] + x[7,17] + x[7,18] + x[7,19] + x[7,20]
   + x[7,21] + x[7,22] + x[7,23] = 1
 OutFlowVehicles[8]: x[8,1] + x[8,2] + x[8,3] + x[8,4] + x[8,5] + x[8,6]
   + x[8,7] + x[8,8] + x[8,9] + x[8,10] + x[8,11] + x[8,12] + x[8,13]
   + x[8,14] + x[8,15] + x[8,16] + x[8,17] + x[8,18] + x[8,19] + x[8,20]
   + x[8,21] + x[8,22] + x[8,23] = 1
 OutFlowVehicles[9]: x[9,1] + x[9,2] + x[9,3] + x[9,4] + x[9,5] + x[9,6]
   + x[9,7] + x[9,8] + x[9,9] + x[9,10] + x[9,11] + x[9,12] + x[9,13]
   + x[9,14] + x[9,15] + x[9,16] + x[9,17] + x[9,18] + x[9,19] + x[9,20]
   + x[9,21] + x[9,22] + x[9,23] = 1
 OutFlowVehicles[10]: x[10,1] + x[10,2] + x[10,3] + x[10,4] + x[10,5]
   + x[10,6] + x[10,7] + x[10,8] + x[10,9] + x[10,10] + x[10,11] + x[10,12]
   + x[10,13] + x[10,14] + x[10,15] + x[10,16] + x[10,17] + x[10,18]
   + x[10,19] + x[10,20] + x[10,21] + x[10,22] + x[10,23] = 1
 OutFlowVehicles[11]: x[11,1] + x[11,2] + x[11,3] + x[11,4] + x[11,5]
   + x[11,6] + x[11,7] + x[11,8] + x[11,9] + x[11,10] + x[11,11] + x[11,12]
   + x[11,13] + x[11,14] + x[11,15] + x[11,16] + x[11,17] + x[11,18]
   + x[11,19] + x[11,20] + x[11,21] + x[11,22] + x[11,23] = 1
 OutFlowVehicles[12]: x[12,1] + x[12,2] + x[12,3] + x[12,4] + x[12,5]
   + x[12,6] + x[12,7] + x[12,8] + x[12,9] + x[12,10] + x[12,11] + x[12,12]
   + x[12,13] + x[12,14] + x[12,15] + x[12,16] + x[12,17] + x[12,18]
   + x[12,19] + x[12,20] + x[12,21] + x[12,22] + x[12,23] = 1
 OutFlowVehicles[13]: x[13,1] + x[13,2] + x[13,3] + x[13,4] + x[13,5]
   + x[13,6] + x[13,7] + x[13,8] + x[13,9] + x[13,10] + x[13,11] + x[13,12]
   + x[13,13] + x[13,14] + x[13,15] + x[13,16] + x[13,17] + x[13,18]
   + x[13,19] + x[13,20] + x[13,21] + x[13,22] + x[13,23] = 1
 OutFlowVehicles[14]: x[14,1] + x[14,2] + x[14,3] + x[14,4] + x[14,5]
   + x[14,6] + x[14,7] + x[14,8] + x[14,9] + x[14,10] + x[14,11] + x[14,12]
   + x[14,13] + x[14,14] + x[14,15] + x[14,16] + x[14,17] + x[14,18]
   + x[14,19] + x[14,20] + x[14,21] + x[14,22] + x[14,23] = 1
 OutFlowVehicles[15]: x[15,1] + x[15,2] + x[15,3] + x[15,4] + x[15,5]
   + x[15,6] + x[15,7] + x[15,8] + x[15,9] + x[15,10] + x[15,11] + x[15,12]
   + x[15,13] + x[15,14] + x[15,15] + x[15,16] + x[15,17] + x[15,18]
   + x[15,19] + x[15,20] + x[15,21] + x[15,22] + x[15,23] = 1
 OutFlowVehicles[16]: x[16,1] + x[16,2] + x[16,3] + x[16,4] + x[16,5]
   + x[16,6] + x[16,7] + x[16,8] + x[16,9] + x[16,10] + x[16,11] + x[16,12]
   + x[16,13] + x[16,14] + x[16,15] + x[16,16] + x[16,17] + x[16,18]
   + x[16,19] + x[16,20] + x[16,21] + x[16,22] + x[16,23] = 1
 OutFlowVehicles[17]: x[17,1] + x[17,2] + x[17,3] + x[17,4] + x[17,5]
   + x[17,6] + x[17,7] + x[17,8] + x[17,9] + x[17,10] + x[17,11] + x[17,12]
   + x[17,13] + x[17,14] + x[17,15] + x[17,16] + x[17,17] + x[17,18]
   + x[17,19] + x[17,20] + x[17,21] + x[17,22] + x[17,23] = 1
 OutFlowVehicles[18]: x[18,1] + x[18,2] + x[18,3] + x[18,4] + x[18,5]
   + x[18,6] + x[18,7] + x[18,8] + x[18,9] + x[18,10] + x[18,11] + x[18,12]
   + x[18,13] + x[18,14] + x[18,15] + x[18,16] + x[18,17] + x[18,18]
   + x[18,19] + x[18,20] + x[18,21] + x[18,22] + x[18,23] = 1
 OutFlowVehicles[19]: x[19,1] + x[19,2] + x[19,3] + x[19,4] + x[19,5]
   + x[19,6] + x[19,7] + x[19,8] + x[19,9] + x[19,10] + x[19,11] + x[19,12]
   + x[19,13] + x[19,14] + x[19,15] + x[19,16] + x[19,17] + x[19,18]
   + x[19,19] + x[19,20] + x[19,21] + x[19,22] + x[19,23] = 1
 OutFlowVehicles[20]: x[20,1] + x[20,2] + x[20,3] + x[20,4] + x[20,5]
   + x[20,6] + x[20,7] + x[20,8] + x[20,9] + x[20,10] + x[20,11] + x[20,12]
   + x[20,13] + x[20,14] + x[20,15] + x[20,16] + x[20,17] + x[20,18]
   + x[20,19] + x[20,20] + x[20,21] + x[20,22] + x[20,23] = 1
 OutFlowVehicles[21]: x[21,1] + x[21,2] + x[21,3] + x[21,4] + x[21,5]
   + x[21,6] + x[21,7] + x[21,8] + x[21,9] + x[21,10] + x[21,11] + x[21,12]
   + x[21,13] + x[21,14] + x[21,15] + x[21,16] + x[21,17] + x[21,18]
   + x[21,19] + x[21,20] + x[21,21] + x[21,22] + x[21,23] = 1
 OutFlowVehicles[22]: x[22,1] + x[22,2] + x[22,3] + x[22,4] + x[22,5]
   + x[22,6] + x[22,7] + x[22,8] + x[22,9] + x[22,10] + x[22,11] + x[22,12]
   + x[22,13] + x[22,14] + x[22,15] + x[22,16] + x[22,17] + x[22,18]
   + x[22,19] + x[22,20] + x[22,21] + x[22,22] + x[22,23] = 1
 InFlowVehicles[1]: x[0,1] + x[1,1] + x[2,1] + x[3,1] + x[4,1] + x[5,1]
   + x[6,1] + x[7,1] + x[8,1] + x[9,1] + x[10,1] + x[11,1] + x[12,1]
   + x[13,1] + x[14,1] + x[15,1] + x[16,1] + x[17,1] + x[18,1] + x[19,1]
   + x[20,1] + x[21,1] + x[22,1] = 1
 InFlowVehicles[2]: x[0,2] + x[1,2] + x[2,2] + x[3,2] + x[4,2] + x[5,2]
   + x[6,2] + x[7,2] + x[8,2] + x[9,2] + x[10,2] + x[11,2] + x[12,2]
   + x[13,2] + x[14,2] + x[15,2] + x[16,2] + x[17,2] + x[18,2] + x[19,2]
   + x[20,2] + x[21,2] + x[22,2] = 1
 InFlowVehicles[3]: x[0,3] + x[1,3] + x[2,3] + x[3,3] + x[4,3] + x[5,3]
   + x[6,3] + x[7,3] + x[8,3] + x[9,3] + x[10,3] + x[11,3] + x[12,3]
   + x[13,3] + x[14,3] + x[15,3] + x[16,3] + x[17,3] + x[18,3] + x[19,3]
   + x[20,3] + x[21,3] + x[22,3] = 1
 InFlowVehicles[4]: x[0,4] + x[1,4] + x[2,4] + x[3,4] + x[4,4] + x[5,4]
   + x[6,4] + x[7,4] + x[8,4] + x[9,4] + x[10,4] + x[11,4] + x[12,4]
   + x[13,4] + x[14,4] + x[15,4] + x[16,4] + x[17,4] + x[18,4] + x[19,4]
   + x[20,4] + x[21,4] + x[22,4] = 1
 InFlowVehicles[5]: x[0,5] + x[1,5] + x[2,5] + x[3,5] + x[4,5] + x[5,5]
   + x[6,5] + x[7,5] + x[8,5] + x[9,5] + x[10,5] + x[11,5] + x[12,5]
   + x[13,5] + x[14,5] + x[15,5] + x[16,5] + x[17,5] + x[18,5] + x[19,5]
   + x[20,5] + x[21,5] + x[22,5] = 1
 InFlowVehicles[6]: x[0,6] + x[1,6] + x[2,6] + x[3,6] + x[4,6] + x[5,6]
   + x[6,6] + x[7,6] + x[8,6] + x[9,6] + x[10,6] + x[11,6] + x[12,6]
   + x[13,6] + x[14,6] + x[15,6] + x[16,6] + x[17,6] + x[18,6] + x[19,6]
   + x[20,6] + x[21,6] + x[22,6] = 1
 InFlowVehicles[7]: x[0,7] + x[1,7] + x[2,7] + x[3,7] + x[4,7] + x[5,7]
   + x[6,7] + x[7,7] + x[8,7] + x[9,7] + x[10,7] + x[11,7] + x[12,7]
   + x[13,7] + x[14,7] + x[15,7] + x[16,7] + x[17,7] + x[18,7] + x[19,7]
   + x[20,7] + x[21,7] + x[22,7] = 1
 InFlowVehicles[8]: x[0,8] + x[1,8] + x[2,8] + x[3,8] + x[4,8] + x[5,8]
   + x[6,8] + x[7,8] + x[8,8] + x[9,8] + x[10,8] + x[11,8] + x[12,8]
   + x[13,8] + x[14,8] + x[15,8] + x[16,8] + x[17,8] + x[18,8] + x[19,8]
   + x[20,8] + x[21,8] + x[22,8] = 1
 InFlowVehicles[9]: x[0,9] + x[1,9] + x[2,9] + x[3,9] + x[4,9] + x[5,9]
   + x[6,9] + x[7,9] + x[8,9] + x[9,9] + x[10,9] + x[11,9] + x[12,9]
   + x[13,9] + x[14,9] + x[15,9] + x[16,9] + x[17,9] + x[18,9] + x[19,9]
   + x[20,9] + x[21,9] + x[22,9] = 1
 InFlowVehicles[10]: x[0,10] + x[1,10] + x[2,10] + x[3,10] + x[4,10]
   + x[5,10] + x[6,10] + x[7,10] + x[8,10] + x[9,10] + x[10,10] + x[11,10]
   + x[12,10] + x[13,10] + x[14,10] + x[15,10] + x[16,10] + x[17,10]
   + x[18,10] + x[19,10] + x[20,10] + x[21,10] + x[22,10] = 1
 InFlowVehicles[11]: x[0,11] + x[1,11] + x[2,11] + x[3,11] + x[4,11]
   + x[5,11] + x[6,11] + x[7,11] + x[8,11] + x[9,11] + x[10,11] + x[11,11]
   + x[12,11] + x[13,11] + x[14,11] + x[15,11] + x[16,11] + x[17,11]
   + x[18,11] + x[19,11] + x[20,11] + x[21,11] + x[22,11] = 1
 InFlowVehicles[12]: x[0,12] + x[1,12] + x[2,12] + x[3,12] + x[4,12]
   + x[5,12] + x[6,12] + x[7,12] + x[8,12] + x[9,12] + x[10,12] + x[11,12]
   + x[12,12] + x[13,12] + x[14,12] + x[15,12] + x[16,12] + x[17,12]
   + x[18,12] + x[19,12] + x[20,12] + x[21,12] + x[22,12] = 1
 InFlowVehicles[13]: x[0,13] + x[1,13] + x[2,13] + x[3,13] + x[4,13]
   + x[5,13] + x[6,13] + x[7,13] + x[8,13] + x[9,13] + x[10,13] + x[11,13]
   + x[12,13] + x[13,13] + x[14,13] + x[15,13] + x[16,13] + x[17,13]
   + x[18,13] + x[19,13] + x[20,13] + x[21,13] + x[22,13] = 1
 InFlowVehicles[14]: x[0,14] + x[1,14] + x[2,14] + x[3,14] + x[4,14]
   + x[5,14] + x[6,14] + x[7,14] + x[8,14] + x[9,14] + x[10,14] + x[11,14]
   + x[12,14] + x[13,14] + x[14,14] + x[15,14] + x[16,14] + x[17,14]
   + x[18,14] + x[19,14] + x[20,14] + x[21,14] + x[22,14] = 1
 InFlowVehicles[15]: x[0,15] + x[1,15] + x[2,15] + x[3,15] + x[4,15]
   + x[5,15] + x[6,15] + x[7,15] + x[8,15] + x[9,15] + x[10,15] + x[11,15]
   + x[12,15] + x[13,15] + x[14,15] + x[15,15] + x[16,15] + x[17,15]
   + x[18,15] + x[19,15] + x[20,15] + x[21,15] + x[22,15] = 1
 InFlowVehicles[16]: x[0,16] + x[1,16] + x[2,16] + x[3,16] + x[4,16]
   + x[5,16] + x[6,16] + x[7,16] + x[8,16] + x[9,16] + x[10,16] + x[11,16]
   + x[12,16] + x[13,16] + x[14,16] + x[15,16] + x[16,16] + x[17,16]
   + x[18,16] + x[19,16] + x[20,16] + x[21,16] + x[22,16] = 1
 InFlowVehicles[17]: x[0,17] + x[1,17] + x[2,17] + x[3,17] + x[4,17]
   + x[5,17] + x[6,17] + x[7,17] + x[8,17] + x[9,17] + x[10,17] + x[11,17]
   + x[12,17] + x[13,17] + x[14,17] + x[15,17] + x[16,17] + x[17,17]
   + x[18,17] + x[19,17] + x[20,17] + x[21,17] + x[22,17] = 1
 InFlowVehicles[18]: x[0,18] + x[1,18] + x[2,18] + x[3,18] + x[4,18]
   + x[5,18] + x[6,18] + x[7,18] + x[8,18] + x[9,18] + x[10,18] + x[11,18]
   + x[12,18] + x[13,18] + x[14,18] + x[15,18] + x[16,18] + x[17,18]
   + x[18,18] + x[19,18] + x[20,18] + x[21,18] + x[22,18] = 1
 InFlowVehicles[19]: x[0,19] + x[1,19] + x[2,19] + x[3,19] + x[4,19]
   + x[5,19] + x[6,19] + x[7,19] + x[8,19] + x[9,19] + x[10,19] + x[11,19]
   + x[12,19] + x[13,19] + x[14,19] + x[15,19] + x[16,19] + x[17,19]
   + x[18,19] + x[19,19] + x[20,19] + x[21,19] + x[22,19] = 1
 InFlowVehicles[20]: x[0,20] + x[1,20] + x[2,20] + x[3,20] + x[4,20]
   + x[5,20] + x[6,20] + x[7,20] + x[8,20] + x[9,20] + x[10,20] + x[11,20]
   + x[12,20] + x[13,20] + x[14,20] + x[15,20] + x[16,20] + x[17,20]
   + x[18,20] + x[19,20] + x[20,20] + x[21,20] + x[22,20] = 1
 InFlowVehicles[21]: x[0,21] + x[1,21] + x[2,21] + x[3,21] + x[4,21]
   + x[5,21] + x[6,21] + x[7,21] + x[8,21] + x[9,21] + x[10,21] + x[11,21]
   + x[12,21] + x[13,21] + x[14,21] + x[15,21] + x[16,21] + x[17,21]
   + x[18,21] + x[19,21] + x[20,21] + x[21,21] + x[22,21] = 1
 InFlowVehicles[22]: x[0,22] + x[1,22] + x[2,22] + x[3,22] + x[4,22]
   + x[5,22] + x[6,22] + x[7,22] + x[8,22] + x[9,22] + x[10,22] + x[11,22]
   + x[12,22] + x[13,22] + x[14,22] + x[15,22] + x[16,22] + x[17,22]
   + x[18,22] + x[19,22] + x[20,22] + x[21,22] + x[22,22] = 1
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - start_service[0]
   + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 999999 x[0,2] - start_service[0]
   + start_service[2] >= -999945.4287857366
 NodeArrivalVehicles[0,3]: - 999999 x[0,3] - start_service[0]
   + start_service[3] >= -999962.9193004568
 NodeArrivalVehicles[0,4]: - 999999 x[0,4] - start_service[0]
   + start_service[4] >= -999937.583726213
 NodeArrivalVehicles[0,5]: - 999999 x[0,5] - start_service[0]
   + start_service[5] >= -999947.3853854308
 NodeArrivalVehicles[0,6]: - 999999 x[0,6] - start_service[0]
   + start_service[6] >= -999936.1484321967
 NodeArrivalVehicles[0,7]: - 999999 x[0,7] - start_service[0]
   + start_service[7] >= -999941.126196999
 NodeArrivalVehicles[0,8]: - 999999 x[0,8] - start_service[0]
   + start_service[8] >= -999951.7620508042
 NodeArrivalVehicles[0,9]: - 999999 x[0,9] - start_service[0]
   + start_service[9] >= -999971.4290013848
 NodeArrivalVehicles[0,10]: - 999999 x[0,10] - start_service[0]
   + start_service[10] >= -999934.7445159944
 NodeArrivalVehicles[0,11]: - 999999 x[0,11] - start_service[0]
   + start_service[11] >= -999984.1635755197
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999945.4287857366
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999962.9193004568
 NodeArrivalVehicles[0,14]: - 999999 x[0,14] - start_service[0]
   + start_service[14] >= -999937.583726213
 NodeArrivalVehicles[0,15]: - 999999 x[0,15] - start_service[0]
   + start_service[15] >= -999947.3853854308
 NodeArrivalVehicles[0,16]: - 999999 x[0,16] - start_service[0]
   + start_service[16] >= -999936.1484321967
 NodeArrivalVehicles[0,17]: - 999999 x[0,17] - start_service[0]
   + start_service[17] >= -999941.126196999
 NodeArrivalVehicles[0,18]: - 999999 x[0,18] - start_service[0]
   + start_service[18] >= -999951.7620508042
 NodeArrivalVehicles[0,19]: - 999999 x[0,19] - start_service[0]
   + start_service[19] >= -999971.4290013848
 NodeArrivalVehicles[0,20]: - 999999 x[0,20] - start_service[0]
   + start_service[20] >= -999934.7445159944
 NodeArrivalVehicles[0,21]: - 999999 x[0,21] - start_service[0]
   + start_service[21] >= -999984.1635755197
 NodeArrivalVehicles[0,22]: - 999999 x[0,22] - start_service[0]
   + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999 x[0,23] - start_service[0]
   + start_service[23] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
   + start_service[2] >= -999945.4287857366
 NodeArrivalVehicles[1,3]: - 999999 x[1,3] - start_service[1]
   + start_service[3] >= -999962.9193004568
 NodeArrivalVehicles[1,4]: - 999999 x[1,4] - start_service[1]
   + start_service[4] >= -999937.583726213
 NodeArrivalVehicles[1,5]: - 999999 x[1,5] - start_service[1]
   + start_service[5] >= -999947.3853854308
 NodeArrivalVehicles[1,6]: - 999999 x[1,6] - start_service[1]
   + start_service[6] >= -999936.1484321967
 NodeArrivalVehicles[1,7]: - 999999 x[1,7] - start_service[1]
   + start_service[7] >= -999941.126196999
 NodeArrivalVehicles[1,8]: - 999999 x[1,8] - start_service[1]
   + start_service[8] >= -999951.7620508042
 NodeArrivalVehicles[1,9]: - 999999 x[1,9] - start_service[1]
   + start_service[9] >= -999971.4290013848
 NodeArrivalVehicles[1,10]: - 999999 x[1,10] - start_service[1]
   + start_service[10] >= -999934.7445159944
 NodeArrivalVehicles[1,11]: - 999999 x[1,11] - start_service[1]
   + start_service[11] >= -999984.1635755197
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999945.4287857366
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999962.9193004568
 NodeArrivalVehicles[1,14]: - 999999 x[1,14] - start_service[1]
   + start_service[14] >= -999937.583726213
 NodeArrivalVehicles[1,15]: - 999999 x[1,15] - start_service[1]
   + start_service[15] >= -999947.3853854308
 NodeArrivalVehicles[1,16]: - 999999 x[1,16] - start_service[1]
   + start_service[16] >= -999936.1484321967
 NodeArrivalVehicles[1,17]: - 999999 x[1,17] - start_service[1]
   + start_service[17] >= -999941.126196999
 NodeArrivalVehicles[1,18]: - 999999 x[1,18] - start_service[1]
   + start_service[18] >= -999951.7620508042
 NodeArrivalVehicles[1,19]: - 999999 x[1,19] - start_service[1]
   + start_service[19] >= -999971.4290013848
 NodeArrivalVehicles[1,20]: - 999999 x[1,20] - start_service[1]
   + start_service[20] >= -999934.7445159944
 NodeArrivalVehicles[1,21]: - 999999 x[1,21] - start_service[1]
   + start_service[21] >= -999984.1635755197
 NodeArrivalVehicles[1,22]: - 999999 x[1,22] - start_service[1]
   + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999 x[1,23] - start_service[1]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999945.4287857366
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 999999 x[2,3] - start_service[2]
   + start_service[3] >= -999955.9804138733
 NodeArrivalVehicles[2,4]: - 999999 x[2,4] - start_service[2]
   + start_service[4] >= -999938.8231751657
 NodeArrivalVehicles[2,5]: - 999999 x[2,5] - start_service[2]
   + start_service[5] >= -999926.0251344698
 NodeArrivalVehicles[2,6]: - 999999 x[2,6] - start_service[2]
   + start_service[6] >= -999925.9721696612
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999933.5494026708
 NodeArrivalVehicles[2,8]: - 999999 x[2,8] - start_service[2]
   + start_service[8] >= -999979.7861738935
 NodeArrivalVehicles[2,9]: - 999999 x[2,9] - start_service[2]
   + start_service[9] >= -999931.6046053576
 NodeArrivalVehicles[2,10]: - 999999 x[2,10] - start_service[2]
   + start_service[10] >= -999987.1892001542
 NodeArrivalVehicles[2,11]: - 999999 x[2,11] - start_service[2]
   + start_service[11] >= -999932.5412105162
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - start_service[2]
   + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999955.9804138733
 NodeArrivalVehicles[2,14]: - 999999 x[2,14] - start_service[2]
   + start_service[14] >= -999938.8231751657
 NodeArrivalVehicles[2,15]: - 999999 x[2,15] - start_service[2]
   + start_service[15] >= -999926.0251344698
 NodeArrivalVehicles[2,16]: - 999999 x[2,16] - start_service[2]
   + start_service[16] >= -999925.9721696612
 NodeArrivalVehicles[2,17]: - 999999 x[2,17] - start_service[2]
   + start_service[17] >= -999933.5494026708
 NodeArrivalVehicles[2,18]: - 999999 x[2,18] - start_service[2]
   + start_service[18] >= -999979.7861738935
 NodeArrivalVehicles[2,19]: - 999999 x[2,19] - start_service[2]
   + start_service[19] >= -999931.6046053576
 NodeArrivalVehicles[2,20]: - 999999 x[2,20] - start_service[2]
   + start_service[20] >= -999987.1892001542
 NodeArrivalVehicles[2,21]: - 999999 x[2,21] - start_service[2]
   + start_service[21] >= -999932.5412105162
 NodeArrivalVehicles[2,22]: - 999999 x[2,22] - start_service[2]
   + start_service[22] >= -999945.4287857366
 NodeArrivalVehicles[2,23]: - 999999 x[2,23] - start_service[2]
   + start_service[23] >= -999945.4287857366
 NodeArrivalVehicles[3,1]: - 999999 x[3,1] + start_service[1]
   - start_service[3] >= -999962.9193004568
 NodeArrivalVehicles[3,2]: - 999999 x[3,2] + start_service[2]
   - start_service[3] >= -999955.9804138733
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 999999 x[3,4] - start_service[3]
   + start_service[4] >= -999972.474248387
 NodeArrivalVehicles[3,5]: - 999999 x[3,5] - start_service[3]
   + start_service[5] >= -999916.1872075291
 NodeArrivalVehicles[3,6]: - 999999 x[3,6] - start_service[3]
   + start_service[6] >= -999908.5111139182
 NodeArrivalVehicles[3,7]: - 999999 x[3,7] - start_service[3]
   + start_service[7] >= -999972.6248534749
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999973.4499059909
 NodeArrivalVehicles[3,9]: - 999999 x[3,9] - start_service[3]
   + start_service[9] >= -999936.1977742163
 NodeArrivalVehicles[3,10]: - 999999 x[3,10] - start_service[3]
   + start_service[10] >= -999944.4584358049
 NodeArrivalVehicles[3,11]: - 999999 x[3,11] - start_service[3]
   + start_service[11] >= -999959.1641715461
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - start_service[3]
   + start_service[12] >= -999955.9804138733
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 999999 x[3,14] - start_service[3]
   + start_service[14] >= -999972.474248387
 NodeArrivalVehicles[3,15]: - 999999 x[3,15] - start_service[3]
   + start_service[15] >= -999916.1872075291
 NodeArrivalVehicles[3,16]: - 999999 x[3,16] - start_service[3]
   + start_service[16] >= -999908.5111139182
 NodeArrivalVehicles[3,17]: - 999999 x[3,17] - start_service[3]
   + start_service[17] >= -999972.6248534749
 NodeArrivalVehicles[3,18]: - 999999 x[3,18] - start_service[3]
   + start_service[18] >= -999973.4499059909
 NodeArrivalVehicles[3,19]: - 999999 x[3,19] - start_service[3]
   + start_service[19] >= -999936.1977742163
 NodeArrivalVehicles[3,20]: - 999999 x[3,20] - start_service[3]
   + start_service[20] >= -999944.4584358049
 NodeArrivalVehicles[3,21]: - 999999 x[3,21] - start_service[3]
   + start_service[21] >= -999959.1641715461
 NodeArrivalVehicles[3,22]: - 999999 x[3,22] - start_service[3]
   + start_service[22] >= -999962.9193004568
 NodeArrivalVehicles[3,23]: - 999999 x[3,23] - start_service[3]
   + start_service[23] >= -999962.9193004568
 NodeArrivalVehicles[4,1]: - 999999 x[4,1] + start_service[1]
   - start_service[4] >= -999937.583726213
 NodeArrivalVehicles[4,2]: - 999999 x[4,2] + start_service[2]
   - start_service[4] >= -999938.8231751657
 NodeArrivalVehicles[4,3]: - 999999 x[4,3] + start_service[3]
   - start_service[4] >= -999972.474248387
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 999999 x[4,5] - start_service[4]
   + start_service[5] >= -999889.662152088
 NodeArrivalVehicles[4,6]: - 999999 x[4,6] - start_service[4]
   + start_service[6] >= -999882.2562239703
 NodeArrivalVehicles[4,7]: - 999999 x[4,7] - start_service[4]
   + start_service[7] >= -999988.8635851692
 NodeArrivalVehicles[4,8]: - 999999 x[4,8] - start_service[4]
   + start_service[8] >= -999957.9553616339
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999910.2653522416
 NodeArrivalVehicles[4,10]: - 999999 x[4,10] - start_service[4]
   + start_service[10] >= -999929.0561937442
 NodeArrivalVehicles[4,11]: - 999999 x[4,11] - start_service[4]
   + start_service[11] >= -999937.3674259853
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - start_service[4]
   + start_service[12] >= -999938.8231751657
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - start_service[4]
   + start_service[13] >= -999972.474248387
 NodeArrivalVehicles[4,14]: - 999999 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 999999 x[4,15] - start_service[4]
   + start_service[15] >= -999889.662152088
 NodeArrivalVehicles[4,16]: - 999999 x[4,16] - start_service[4]
   + start_service[16] >= -999882.2562239703
 NodeArrivalVehicles[4,17]: - 999999 x[4,17] - start_service[4]
   + start_service[17] >= -999988.8635851692
 NodeArrivalVehicles[4,18]: - 999999 x[4,18] - start_service[4]
   + start_service[18] >= -999957.9553616339
 NodeArrivalVehicles[4,19]: - 999999 x[4,19] - start_service[4]
   + start_service[19] >= -999910.2653522416
 NodeArrivalVehicles[4,20]: - 999999 x[4,20] - start_service[4]
   + start_service[20] >= -999929.0561937442
 NodeArrivalVehicles[4,21]: - 999999 x[4,21] - start_service[4]
   + start_service[21] >= -999937.3674259853
 NodeArrivalVehicles[4,22]: - 999999 x[4,22] - start_service[4]
   + start_service[22] >= -999937.583726213
 NodeArrivalVehicles[4,23]: - 999999 x[4,23] - start_service[4]
   + start_service[23] >= -999937.583726213
 NodeArrivalVehicles[5,1]: - 999999 x[5,1] + start_service[1]
   - start_service[5] >= -999947.3853854308
 NodeArrivalVehicles[5,2]: - 999999 x[5,2] + start_service[2]
   - start_service[5] >= -999926.0251344698
 NodeArrivalVehicles[5,3]: - 999999 x[5,3] + start_service[3]
   - start_service[5] >= -999916.1872075291
 NodeArrivalVehicles[5,4]: - 999999 x[5,4] + start_service[4]
   - start_service[5] >= -999889.662152088
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 999999 x[5,6] - start_service[5]
   + start_service[6] >= -999983.6081284663
 NodeArrivalVehicles[5,7]: - 999999 x[5,7] - start_service[5]
   + start_service[7] >= -999891.227834211
 NodeArrivalVehicles[5,8]: - 999999 x[5,8] - start_service[5]
   + start_service[8] >= -999918.6764967154
 NodeArrivalVehicles[5,9]: - 999999 x[5,9] - start_service[5]
   + start_service[9] >= -999971.5729838425
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999921.6996578324
 NodeArrivalVehicles[5,11]: - 999999 x[5,11] - start_service[5]
   + start_service[11] >= -999937.5561410894
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - start_service[5]
   + start_service[12] >= -999926.0251344698
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - start_service[5]
   + start_service[13] >= -999916.1872075291
 NodeArrivalVehicles[5,14]: - 999999 x[5,14] - start_service[5]
   + start_service[14] >= -999889.662152088
 NodeArrivalVehicles[5,15]: - 999999 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 999999 x[5,16] - start_service[5]
   + start_service[16] >= -999983.6081284663
 NodeArrivalVehicles[5,17]: - 999999 x[5,17] - start_service[5]
   + start_service[17] >= -999891.227834211
 NodeArrivalVehicles[5,18]: - 999999 x[5,18] - start_service[5]
   + start_service[18] >= -999918.6764967154
 NodeArrivalVehicles[5,19]: - 999999 x[5,19] - start_service[5]
   + start_service[19] >= -999971.5729838425
 NodeArrivalVehicles[5,20]: - 999999 x[5,20] - start_service[5]
   + start_service[20] >= -999921.6996578324
 NodeArrivalVehicles[5,21]: - 999999 x[5,21] - start_service[5]
   + start_service[21] >= -999937.5561410894
 NodeArrivalVehicles[5,22]: - 999999 x[5,22] - start_service[5]
   + start_service[22] >= -999947.3853854308
 NodeArrivalVehicles[5,23]: - 999999 x[5,23] - start_service[5]
   + start_service[23] >= -999947.3853854308
 NodeArrivalVehicles[6,1]: - 999999 x[6,1] + start_service[1]
   - start_service[6] >= -999936.1484321967
 NodeArrivalVehicles[6,2]: - 999999 x[6,2] + start_service[2]
   - start_service[6] >= -999925.9721696612
 NodeArrivalVehicles[6,3]: - 999999 x[6,3] + start_service[3]
   - start_service[6] >= -999908.5111139182
 NodeArrivalVehicles[6,4]: - 999999 x[6,4] + start_service[4]
   - start_service[6] >= -999882.2562239703
 NodeArrivalVehicles[6,5]: - 999999 x[6,5] + start_service[5]
   - start_service[6] >= -999983.6081284663
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 999999 x[6,7] - start_service[6]
   + start_service[7] >= -999882.642633686
 NodeArrivalVehicles[6,8]: - 999999 x[6,8] - start_service[6]
   + start_service[8] >= -999915.2753088792
 NodeArrivalVehicles[6,9]: - 999999 x[6,9] - start_service[6]
   + start_service[9] >= -999957.385467226
 NodeArrivalVehicles[6,10]: - 999999 x[6,10] - start_service[6]
   + start_service[10] >= -999923.9891144931
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999924.7300771383
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - start_service[6]
   + start_service[12] >= -999925.9721696612
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - start_service[6]
   + start_service[13] >= -999908.5111139182
 NodeArrivalVehicles[6,14]: - 999999 x[6,14] - start_service[6]
   + start_service[14] >= -999882.2562239703
 NodeArrivalVehicles[6,15]: - 999999 x[6,15] - start_service[6]
   + start_service[15] >= -999983.6081284663
 NodeArrivalVehicles[6,16]: - 999999 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 999999 x[6,17] - start_service[6]
   + start_service[17] >= -999882.642633686
 NodeArrivalVehicles[6,18]: - 999999 x[6,18] - start_service[6]
   + start_service[18] >= -999915.2753088792
 NodeArrivalVehicles[6,19]: - 999999 x[6,19] - start_service[6]
   + start_service[19] >= -999957.385467226
 NodeArrivalVehicles[6,20]: - 999999 x[6,20] - start_service[6]
   + start_service[20] >= -999923.9891144931
 NodeArrivalVehicles[6,21]: - 999999 x[6,21] - start_service[6]
   + start_service[21] >= -999924.7300771383
 NodeArrivalVehicles[6,22]: - 999999 x[6,22] - start_service[6]
   + start_service[22] >= -999936.1484321967
 NodeArrivalVehicles[6,23]: - 999999 x[6,23] - start_service[6]
   + start_service[23] >= -999936.1484321967
 NodeArrivalVehicles[7,1]: - 999999 x[7,1] + start_service[1]
   - start_service[7] >= -999941.126196999
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999933.5494026708
 NodeArrivalVehicles[7,3]: - 999999 x[7,3] + start_service[3]
   - start_service[7] >= -999972.6248534749
 NodeArrivalVehicles[7,4]: - 999999 x[7,4] + start_service[4]
   - start_service[7] >= -999988.8635851692
 NodeArrivalVehicles[7,5]: - 999999 x[7,5] + start_service[5]
   - start_service[7] >= -999891.227834211
 NodeArrivalVehicles[7,6]: - 999999 x[7,6] + start_service[6]
   - start_service[7] >= -999882.642633686
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 999999 x[7,8] - start_service[7]
   + start_service[8] >= -999952.7111189339
 NodeArrivalVehicles[7,9]: - 999999 x[7,9] - start_service[7]
   + start_service[9] >= -999913.555315655
 NodeArrivalVehicles[7,10]: - 999999 x[7,10] - start_service[7]
   + start_service[10] >= -999923.0335188602
 NodeArrivalVehicles[7,11]: - 999999 x[7,11] - start_service[7]
   + start_service[11] >= -999943.1581189876
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999933.5494026708
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - start_service[7]
   + start_service[13] >= -999972.6248534749
 NodeArrivalVehicles[7,14]: - 999999 x[7,14] - start_service[7]
   + start_service[14] >= -999988.8635851692
 NodeArrivalVehicles[7,15]: - 999999 x[7,15] - start_service[7]
   + start_service[15] >= -999891.227834211
 NodeArrivalVehicles[7,16]: - 999999 x[7,16] - start_service[7]
   + start_service[16] >= -999882.642633686
 NodeArrivalVehicles[7,17]: - 999999 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 999999 x[7,18] - start_service[7]
   + start_service[18] >= -999952.7111189339
 NodeArrivalVehicles[7,19]: - 999999 x[7,19] - start_service[7]
   + start_service[19] >= -999913.555315655
 NodeArrivalVehicles[7,20]: - 999999 x[7,20] - start_service[7]
   + start_service[20] >= -999923.0335188602
 NodeArrivalVehicles[7,21]: - 999999 x[7,21] - start_service[7]
   + start_service[21] >= -999943.1581189876
 NodeArrivalVehicles[7,22]: - 999999 x[7,22] - start_service[7]
   + start_service[22] >= -999941.126196999
 NodeArrivalVehicles[7,23]: - 999999 x[7,23] - start_service[7]
   + start_service[23] >= -999941.126196999
 NodeArrivalVehicles[8,1]: - 999999 x[8,1] + start_service[1]
   - start_service[8] >= -999951.7620508042
 NodeArrivalVehicles[8,2]: - 999999 x[8,2] + start_service[2]
   - start_service[8] >= -999979.7861738935
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999973.4499059909
 NodeArrivalVehicles[8,4]: - 999999 x[8,4] + start_service[4]
   - start_service[8] >= -999957.9553616339
 NodeArrivalVehicles[8,5]: - 999999 x[8,5] + start_service[5]
   - start_service[8] >= -999918.6764967154
 NodeArrivalVehicles[8,6]: - 999999 x[8,6] + start_service[6]
   - start_service[8] >= -999915.2753088792
 NodeArrivalVehicles[8,7]: - 999999 x[8,7] + start_service[7]
   - start_service[8] >= -999952.7111189339
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 999999 x[8,9] - start_service[8]
   + start_service[9] >= -999931.0123467562
 NodeArrivalVehicles[8,10]: - 999999 x[8,10] - start_service[8]
   + start_service[10] >= -999969.1712492406
 NodeArrivalVehicles[8,11]: - 999999 x[8,11] - start_service[8]
   + start_service[11] >= -999941.8023031916
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - start_service[8]
   + start_service[12] >= -999979.7861738935
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - start_service[8]
   + start_service[13] >= -999973.4499059909
 NodeArrivalVehicles[8,14]: - 999999 x[8,14] - start_service[8]
   + start_service[14] >= -999957.9553616339
 NodeArrivalVehicles[8,15]: - 999999 x[8,15] - start_service[8]
   + start_service[15] >= -999918.6764967154
 NodeArrivalVehicles[8,16]: - 999999 x[8,16] - start_service[8]
   + start_service[16] >= -999915.2753088792
 NodeArrivalVehicles[8,17]: - 999999 x[8,17] - start_service[8]
   + start_service[17] >= -999952.7111189339
 NodeArrivalVehicles[8,18]: - 999999 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 999999 x[8,19] - start_service[8]
   + start_service[19] >= -999931.0123467562
 NodeArrivalVehicles[8,20]: - 999999 x[8,20] - start_service[8]
   + start_service[20] >= -999969.1712492406
 NodeArrivalVehicles[8,21]: - 999999 x[8,21] - start_service[8]
   + start_service[21] >= -999941.8023031916
 NodeArrivalVehicles[8,22]: - 999999 x[8,22] - start_service[8]
   + start_service[22] >= -999951.7620508042
 NodeArrivalVehicles[8,23]: - 999999 x[8,23] - start_service[8]
   + start_service[23] >= -999951.7620508042
 NodeArrivalVehicles[9,1]: - 999999 x[9,1] + start_service[1]
   - start_service[9] >= -999971.4290013848
 NodeArrivalVehicles[9,2]: - 999999 x[9,2] + start_service[2]
   - start_service[9] >= -999931.6046053576
 NodeArrivalVehicles[9,3]: - 999999 x[9,3] + start_service[3]
   - start_service[9] >= -999936.1977742163
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999910.2653522416
 NodeArrivalVehicles[9,5]: - 999999 x[9,5] + start_service[5]
   - start_service[9] >= -999971.5729838425
 NodeArrivalVehicles[9,6]: - 999999 x[9,6] + start_service[6]
   - start_service[9] >= -999957.385467226
 NodeArrivalVehicles[9,7]: - 999999 x[9,7] + start_service[7]
   - start_service[9] >= -999913.555315655
 NodeArrivalVehicles[9,8]: - 999999 x[9,8] + start_service[8]
   - start_service[9] >= -999931.0123467562
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 999999 x[9,10] - start_service[9]
   + start_service[10] >= -999923.5677362148
 NodeArrivalVehicles[9,11]: - 999999 x[9,11] - start_service[9]
   + start_service[11] >= -999964.3876103099
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - start_service[9]
   + start_service[12] >= -999931.6046053576
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - start_service[9]
   + start_service[13] >= -999936.1977742163
 NodeArrivalVehicles[9,14]: - 999999 x[9,14] - start_service[9]
   + start_service[14] >= -999910.2653522416
 NodeArrivalVehicles[9,15]: - 999999 x[9,15] - start_service[9]
   + start_service[15] >= -999971.5729838425
 NodeArrivalVehicles[9,16]: - 999999 x[9,16] - start_service[9]
   + start_service[16] >= -999957.385467226
 NodeArrivalVehicles[9,17]: - 999999 x[9,17] - start_service[9]
   + start_service[17] >= -999913.555315655
 NodeArrivalVehicles[9,18]: - 999999 x[9,18] - start_service[9]
   + start_service[18] >= -999931.0123467562
 NodeArrivalVehicles[9,19]: - 999999 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 999999 x[9,20] - start_service[9]
   + start_service[20] >= -999923.5677362148
 NodeArrivalVehicles[9,21]: - 999999 x[9,21] - start_service[9]
   + start_service[21] >= -999964.3876103099
 NodeArrivalVehicles[9,22]: - 999999 x[9,22] - start_service[9]
   + start_service[22] >= -999971.4290013848
 NodeArrivalVehicles[9,23]: - 999999 x[9,23] - start_service[9]
   + start_service[23] >= -999971.4290013848
 NodeArrivalVehicles[10,1]: - 999999 x[10,1] + start_service[1]
   - start_service[10] >= -999934.7445159944
 NodeArrivalVehicles[10,2]: - 999999 x[10,2] + start_service[2]
   - start_service[10] >= -999987.1892001542
 NodeArrivalVehicles[10,3]: - 999999 x[10,3] + start_service[3]
   - start_service[10] >= -999944.4584358049
 NodeArrivalVehicles[10,4]: - 999999 x[10,4] + start_service[4]
   - start_service[10] >= -999929.0561937442
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999921.6996578324
 NodeArrivalVehicles[10,6]: - 999999 x[10,6] + start_service[6]
   - start_service[10] >= -999923.9891144931
 NodeArrivalVehicles[10,7]: - 999999 x[10,7] + start_service[7]
   - start_service[10] >= -999923.0335188602
 NodeArrivalVehicles[10,8]: - 999999 x[10,8] + start_service[8]
   - start_service[10] >= -999969.1712492406
 NodeArrivalVehicles[10,9]: - 999999 x[10,9] + start_service[9]
   - start_service[10] >= -999923.5677362148
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 999999 x[10,11] - start_service[10]
   + start_service[11] >= -999921.3827498653
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - start_service[10]
   + start_service[12] >= -999987.1892001542
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - start_service[10]
   + start_service[13] >= -999944.4584358049
 NodeArrivalVehicles[10,14]: - 999999 x[10,14] - start_service[10]
   + start_service[14] >= -999929.0561937442
 NodeArrivalVehicles[10,15]: - 999999 x[10,15] - start_service[10]
   + start_service[15] >= -999921.6996578324
 NodeArrivalVehicles[10,16]: - 999999 x[10,16] - start_service[10]
   + start_service[16] >= -999923.9891144931
 NodeArrivalVehicles[10,17]: - 999999 x[10,17] - start_service[10]
   + start_service[17] >= -999923.0335188602
 NodeArrivalVehicles[10,18]: - 999999 x[10,18] - start_service[10]
   + start_service[18] >= -999969.1712492406
 NodeArrivalVehicles[10,19]: - 999999 x[10,19] - start_service[10]
   + start_service[19] >= -999923.5677362148
 NodeArrivalVehicles[10,20]: - 999999 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 999999 x[10,21] - start_service[10]
   + start_service[21] >= -999921.3827498653
 NodeArrivalVehicles[10,22]: - 999999 x[10,22] - start_service[10]
   + start_service[22] >= -999934.7445159944
 NodeArrivalVehicles[10,23]: - 999999 x[10,23] - start_service[10]
   + start_service[23] >= -999934.7445159944
 NodeArrivalVehicles[11,1]: - 999999 x[11,1] + start_service[1]
   - start_service[11] >= -999984.1635755197
 NodeArrivalVehicles[11,2]: - 999999 x[11,2] + start_service[2]
   - start_service[11] >= -999932.5412105162
 NodeArrivalVehicles[11,3]: - 999999 x[11,3] + start_service[3]
   - start_service[11] >= -999959.1641715461
 NodeArrivalVehicles[11,4]: - 999999 x[11,4] + start_service[4]
   - start_service[11] >= -999937.3674259853
 NodeArrivalVehicles[11,5]: - 999999 x[11,5] + start_service[5]
   - start_service[11] >= -999937.5561410894
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999924.7300771383
 NodeArrivalVehicles[11,7]: - 999999 x[11,7] + start_service[7]
   - start_service[11] >= -999943.1581189876
 NodeArrivalVehicles[11,8]: - 999999 x[11,8] + start_service[8]
   - start_service[11] >= -999941.8023031916
 NodeArrivalVehicles[11,9]: - 999999 x[11,9] + start_service[9]
   - start_service[11] >= -999964.3876103099
 NodeArrivalVehicles[11,10]: - 999999 x[11,10] + start_service[10]
   - start_service[11] >= -999921.3827498653
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - start_service[11]
   + start_service[12] >= -999932.5412105162
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - start_service[11]
   + start_service[13] >= -999959.1641715461
 NodeArrivalVehicles[11,14]: - 999999 x[11,14] - start_service[11]
   + start_service[14] >= -999937.3674259853
 NodeArrivalVehicles[11,15]: - 999999 x[11,15] - start_service[11]
   + start_service[15] >= -999937.5561410894
 NodeArrivalVehicles[11,16]: - 999999 x[11,16] - start_service[11]
   + start_service[16] >= -999924.7300771383
 NodeArrivalVehicles[11,17]: - 999999 x[11,17] - start_service[11]
   + start_service[17] >= -999943.1581189876
 NodeArrivalVehicles[11,18]: - 999999 x[11,18] - start_service[11]
   + start_service[18] >= -999941.8023031916
 NodeArrivalVehicles[11,19]: - 999999 x[11,19] - start_service[11]
   + start_service[19] >= -999964.3876103099
 NodeArrivalVehicles[11,20]: - 999999 x[11,20] - start_service[11]
   + start_service[20] >= -999921.3827498653
 NodeArrivalVehicles[11,21]: - 999999 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 999999 x[11,22] - start_service[11]
   + start_service[22] >= -999984.1635755197
 NodeArrivalVehicles[11,23]: - 999999 x[11,23] - start_service[11]
   + start_service[23] >= -999984.1635755197
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999945.4287857366
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] + start_service[2]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 999999 x[12,3] + start_service[3]
   - start_service[12] >= -999955.9804138733
 NodeArrivalVehicles[12,4]: - 999999 x[12,4] + start_service[4]
   - start_service[12] >= -999938.8231751657
 NodeArrivalVehicles[12,5]: - 999999 x[12,5] + start_service[5]
   - start_service[12] >= -999926.0251344698
 NodeArrivalVehicles[12,6]: - 999999 x[12,6] + start_service[6]
   - start_service[12] >= -999925.9721696612
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999933.5494026708
 NodeArrivalVehicles[12,8]: - 999999 x[12,8] + start_service[8]
   - start_service[12] >= -999979.7861738935
 NodeArrivalVehicles[12,9]: - 999999 x[12,9] + start_service[9]
   - start_service[12] >= -999931.6046053576
 NodeArrivalVehicles[12,10]: - 999999 x[12,10] + start_service[10]
   - start_service[12] >= -999987.1892001542
 NodeArrivalVehicles[12,11]: - 999999 x[12,11] + start_service[11]
   - start_service[12] >= -999932.5412105162
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999955.9804138733
 NodeArrivalVehicles[12,14]: - 999999 x[12,14] - start_service[12]
   + start_service[14] >= -999938.8231751657
 NodeArrivalVehicles[12,15]: - 999999 x[12,15] - start_service[12]
   + start_service[15] >= -999926.0251344698
 NodeArrivalVehicles[12,16]: - 999999 x[12,16] - start_service[12]
   + start_service[16] >= -999925.9721696612
 NodeArrivalVehicles[12,17]: - 999999 x[12,17] - start_service[12]
   + start_service[17] >= -999933.5494026708
 NodeArrivalVehicles[12,18]: - 999999 x[12,18] - start_service[12]
   + start_service[18] >= -999979.7861738935
 NodeArrivalVehicles[12,19]: - 999999 x[12,19] - start_service[12]
   + start_service[19] >= -999931.6046053576
 NodeArrivalVehicles[12,20]: - 999999 x[12,20] - start_service[12]
   + start_service[20] >= -999987.1892001542
 NodeArrivalVehicles[12,21]: - 999999 x[12,21] - start_service[12]
   + start_service[21] >= -999932.5412105162
 NodeArrivalVehicles[12,22]: - 999999 x[12,22] - start_service[12]
   + start_service[22] >= -999945.4287857366
 NodeArrivalVehicles[12,23]: - 999999 x[12,23] - start_service[12]
   + start_service[23] >= -999945.4287857366
 NodeArrivalVehicles[13,1]: - 999999 x[13,1] + start_service[1]
   - start_service[13] >= -999962.9193004568
 NodeArrivalVehicles[13,2]: - 999999 x[13,2] + start_service[2]
   - start_service[13] >= -999955.9804138733
 NodeArrivalVehicles[13,3]: - 999999 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 999999 x[13,4] + start_service[4]
   - start_service[13] >= -999972.474248387
 NodeArrivalVehicles[13,5]: - 999999 x[13,5] + start_service[5]
   - start_service[13] >= -999916.1872075291
 NodeArrivalVehicles[13,6]: - 999999 x[13,6] + start_service[6]
   - start_service[13] >= -999908.5111139182
 NodeArrivalVehicles[13,7]: - 999999 x[13,7] + start_service[7]
   - start_service[13] >= -999972.6248534749
 NodeArrivalVehicles[13,8]: - 999999 x[13,8] + start_service[8]
   - start_service[13] >= -999973.4499059909
 NodeArrivalVehicles[13,9]: - 999999 x[13,9] + start_service[9]
   - start_service[13] >= -999936.1977742163
 NodeArrivalVehicles[13,10]: - 999999 x[13,10] + start_service[10]
   - start_service[13] >= -999944.4584358049
 NodeArrivalVehicles[13,11]: - 999999 x[13,11] + start_service[11]
   - start_service[13] >= -999959.1641715461
 NodeArrivalVehicles[13,12]: - 999999 x[13,12] + start_service[12]
   - start_service[13] >= -999955.9804138733
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 999999 x[13,14] - start_service[13]
   + start_service[14] >= -999972.474248387
 NodeArrivalVehicles[13,15]: - 999999 x[13,15] - start_service[13]
   + start_service[15] >= -999916.1872075291
 NodeArrivalVehicles[13,16]: - 999999 x[13,16] - start_service[13]
   + start_service[16] >= -999908.5111139182
 NodeArrivalVehicles[13,17]: - 999999 x[13,17] - start_service[13]
   + start_service[17] >= -999972.6248534749
 NodeArrivalVehicles[13,18]: - 999999 x[13,18] - start_service[13]
   + start_service[18] >= -999973.4499059909
 NodeArrivalVehicles[13,19]: - 999999 x[13,19] - start_service[13]
   + start_service[19] >= -999936.1977742163
 NodeArrivalVehicles[13,20]: - 999999 x[13,20] - start_service[13]
   + start_service[20] >= -999944.4584358049
 NodeArrivalVehicles[13,21]: - 999999 x[13,21] - start_service[13]
   + start_service[21] >= -999959.1641715461
 NodeArrivalVehicles[13,22]: - 999999 x[13,22] - start_service[13]
   + start_service[22] >= -999962.9193004568
 NodeArrivalVehicles[13,23]: - 999999 x[13,23] - start_service[13]
   + start_service[23] >= -999962.9193004568
 NodeArrivalVehicles[14,1]: - 999999 x[14,1] + start_service[1]
   - start_service[14] >= -999937.583726213
 NodeArrivalVehicles[14,2]: - 999999 x[14,2] + start_service[2]
   - start_service[14] >= -999938.8231751657
 NodeArrivalVehicles[14,3]: - 999999 x[14,3] + start_service[3]
   - start_service[14] >= -999972.474248387
 NodeArrivalVehicles[14,4]: - 999999 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 999999 x[14,5] + start_service[5]
   - start_service[14] >= -999889.662152088
 NodeArrivalVehicles[14,6]: - 999999 x[14,6] + start_service[6]
   - start_service[14] >= -999882.2562239703
 NodeArrivalVehicles[14,7]: - 999999 x[14,7] + start_service[7]
   - start_service[14] >= -999988.8635851692
 NodeArrivalVehicles[14,8]: - 999999 x[14,8] + start_service[8]
   - start_service[14] >= -999957.9553616339
 NodeArrivalVehicles[14,9]: - 999999 x[14,9] + start_service[9]
   - start_service[14] >= -999910.2653522416
 NodeArrivalVehicles[14,10]: - 999999 x[14,10] + start_service[10]
   - start_service[14] >= -999929.0561937442
 NodeArrivalVehicles[14,11]: - 999999 x[14,11] + start_service[11]
   - start_service[14] >= -999937.3674259853
 NodeArrivalVehicles[14,12]: - 999999 x[14,12] + start_service[12]
   - start_service[14] >= -999938.8231751657
 NodeArrivalVehicles[14,13]: - 999999 x[14,13] + start_service[13]
   - start_service[14] >= -999972.474248387
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 999999 x[14,15] - start_service[14]
   + start_service[15] >= -999889.662152088
 NodeArrivalVehicles[14,16]: - 999999 x[14,16] - start_service[14]
   + start_service[16] >= -999882.2562239703
 NodeArrivalVehicles[14,17]: - 999999 x[14,17] - start_service[14]
   + start_service[17] >= -999988.8635851692
 NodeArrivalVehicles[14,18]: - 999999 x[14,18] - start_service[14]
   + start_service[18] >= -999957.9553616339
 NodeArrivalVehicles[14,19]: - 999999 x[14,19] - start_service[14]
   + start_service[19] >= -999910.2653522416
 NodeArrivalVehicles[14,20]: - 999999 x[14,20] - start_service[14]
   + start_service[20] >= -999929.0561937442
 NodeArrivalVehicles[14,21]: - 999999 x[14,21] - start_service[14]
   + start_service[21] >= -999937.3674259853
 NodeArrivalVehicles[14,22]: - 999999 x[14,22] - start_service[14]
   + start_service[22] >= -999937.583726213
 NodeArrivalVehicles[14,23]: - 999999 x[14,23] - start_service[14]
   + start_service[23] >= -999937.583726213
 NodeArrivalVehicles[15,1]: - 999999 x[15,1] + start_service[1]
   - start_service[15] >= -999947.3853854308
 NodeArrivalVehicles[15,2]: - 999999 x[15,2] + start_service[2]
   - start_service[15] >= -999926.0251344698
 NodeArrivalVehicles[15,3]: - 999999 x[15,3] + start_service[3]
   - start_service[15] >= -999916.1872075291
 NodeArrivalVehicles[15,4]: - 999999 x[15,4] + start_service[4]
   - start_service[15] >= -999889.662152088
 NodeArrivalVehicles[15,5]: - 999999 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 999999 x[15,6] + start_service[6]
   - start_service[15] >= -999983.6081284663
 NodeArrivalVehicles[15,7]: - 999999 x[15,7] + start_service[7]
   - start_service[15] >= -999891.227834211
 NodeArrivalVehicles[15,8]: - 999999 x[15,8] + start_service[8]
   - start_service[15] >= -999918.6764967154
 NodeArrivalVehicles[15,9]: - 999999 x[15,9] + start_service[9]
   - start_service[15] >= -999971.5729838425
 NodeArrivalVehicles[15,10]: - 999999 x[15,10] + start_service[10]
   - start_service[15] >= -999921.6996578324
 NodeArrivalVehicles[15,11]: - 999999 x[15,11] + start_service[11]
   - start_service[15] >= -999937.5561410894
 NodeArrivalVehicles[15,12]: - 999999 x[15,12] + start_service[12]
   - start_service[15] >= -999926.0251344698
 NodeArrivalVehicles[15,13]: - 999999 x[15,13] + start_service[13]
   - start_service[15] >= -999916.1872075291
 NodeArrivalVehicles[15,14]: - 999999 x[15,14] + start_service[14]
   - start_service[15] >= -999889.662152088
 NodeArrivalVehicles[15,15]: - 999999 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 999999 x[15,16] - start_service[15]
   + start_service[16] >= -999983.6081284663
 NodeArrivalVehicles[15,17]: - 999999 x[15,17] - start_service[15]
   + start_service[17] >= -999891.227834211
 NodeArrivalVehicles[15,18]: - 999999 x[15,18] - start_service[15]
   + start_service[18] >= -999918.6764967154
 NodeArrivalVehicles[15,19]: - 999999 x[15,19] - start_service[15]
   + start_service[19] >= -999971.5729838425
 NodeArrivalVehicles[15,20]: - 999999 x[15,20] - start_service[15]
   + start_service[20] >= -999921.6996578324
 NodeArrivalVehicles[15,21]: - 999999 x[15,21] - start_service[15]
   + start_service[21] >= -999937.5561410894
 NodeArrivalVehicles[15,22]: - 999999 x[15,22] - start_service[15]
   + start_service[22] >= -999947.3853854308
 NodeArrivalVehicles[15,23]: - 999999 x[15,23] - start_service[15]
   + start_service[23] >= -999947.3853854308
 NodeArrivalVehicles[16,1]: - 999999 x[16,1] + start_service[1]
   - start_service[16] >= -999936.1484321967
 NodeArrivalVehicles[16,2]: - 999999 x[16,2] + start_service[2]
   - start_service[16] >= -999925.9721696612
 NodeArrivalVehicles[16,3]: - 999999 x[16,3] + start_service[3]
   - start_service[16] >= -999908.5111139182
 NodeArrivalVehicles[16,4]: - 999999 x[16,4] + start_service[4]
   - start_service[16] >= -999882.2562239703
 NodeArrivalVehicles[16,5]: - 999999 x[16,5] + start_service[5]
   - start_service[16] >= -999983.6081284663
 NodeArrivalVehicles[16,6]: - 999999 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 999999 x[16,7] + start_service[7]
   - start_service[16] >= -999882.642633686
 NodeArrivalVehicles[16,8]: - 999999 x[16,8] + start_service[8]
   - start_service[16] >= -999915.2753088792
 NodeArrivalVehicles[16,9]: - 999999 x[16,9] + start_service[9]
   - start_service[16] >= -999957.385467226
 NodeArrivalVehicles[16,10]: - 999999 x[16,10] + start_service[10]
   - start_service[16] >= -999923.9891144931
 NodeArrivalVehicles[16,11]: - 999999 x[16,11] + start_service[11]
   - start_service[16] >= -999924.7300771383
 NodeArrivalVehicles[16,12]: - 999999 x[16,12] + start_service[12]
   - start_service[16] >= -999925.9721696612
 NodeArrivalVehicles[16,13]: - 999999 x[16,13] + start_service[13]
   - start_service[16] >= -999908.5111139182
 NodeArrivalVehicles[16,14]: - 999999 x[16,14] + start_service[14]
   - start_service[16] >= -999882.2562239703
 NodeArrivalVehicles[16,15]: - 999999 x[16,15] + start_service[15]
   - start_service[16] >= -999983.6081284663
 NodeArrivalVehicles[16,16]: - 999999 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 999999 x[16,17] - start_service[16]
   + start_service[17] >= -999882.642633686
 NodeArrivalVehicles[16,18]: - 999999 x[16,18] - start_service[16]
   + start_service[18] >= -999915.2753088792
 NodeArrivalVehicles[16,19]: - 999999 x[16,19] - start_service[16]
   + start_service[19] >= -999957.385467226
 NodeArrivalVehicles[16,20]: - 999999 x[16,20] - start_service[16]
   + start_service[20] >= -999923.9891144931
 NodeArrivalVehicles[16,21]: - 999999 x[16,21] - start_service[16]
   + start_service[21] >= -999924.7300771383
 NodeArrivalVehicles[16,22]: - 999999 x[16,22] - start_service[16]
   + start_service[22] >= -999936.1484321967
 NodeArrivalVehicles[16,23]: - 999999 x[16,23] - start_service[16]
   + start_service[23] >= -999936.1484321967
 NodeArrivalVehicles[17,1]: - 999999 x[17,1] + start_service[1]
   - start_service[17] >= -999941.126196999
 NodeArrivalVehicles[17,2]: - 999999 x[17,2] + start_service[2]
   - start_service[17] >= -999933.5494026708
 NodeArrivalVehicles[17,3]: - 999999 x[17,3] + start_service[3]
   - start_service[17] >= -999972.6248534749
 NodeArrivalVehicles[17,4]: - 999999 x[17,4] + start_service[4]
   - start_service[17] >= -999988.8635851692
 NodeArrivalVehicles[17,5]: - 999999 x[17,5] + start_service[5]
   - start_service[17] >= -999891.227834211
 NodeArrivalVehicles[17,6]: - 999999 x[17,6] + start_service[6]
   - start_service[17] >= -999882.642633686
 NodeArrivalVehicles[17,7]: - 999999 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 999999 x[17,8] + start_service[8]
   - start_service[17] >= -999952.7111189339
 NodeArrivalVehicles[17,9]: - 999999 x[17,9] + start_service[9]
   - start_service[17] >= -999913.555315655
 NodeArrivalVehicles[17,10]: - 999999 x[17,10] + start_service[10]
   - start_service[17] >= -999923.0335188602
 NodeArrivalVehicles[17,11]: - 999999 x[17,11] + start_service[11]
   - start_service[17] >= -999943.1581189876
 NodeArrivalVehicles[17,12]: - 999999 x[17,12] + start_service[12]
   - start_service[17] >= -999933.5494026708
 NodeArrivalVehicles[17,13]: - 999999 x[17,13] + start_service[13]
   - start_service[17] >= -999972.6248534749
 NodeArrivalVehicles[17,14]: - 999999 x[17,14] + start_service[14]
   - start_service[17] >= -999988.8635851692
 NodeArrivalVehicles[17,15]: - 999999 x[17,15] + start_service[15]
   - start_service[17] >= -999891.227834211
 NodeArrivalVehicles[17,16]: - 999999 x[17,16] + start_service[16]
   - start_service[17] >= -999882.642633686
 NodeArrivalVehicles[17,17]: - 999999 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 999999 x[17,18] - start_service[17]
   + start_service[18] >= -999952.7111189339
 NodeArrivalVehicles[17,19]: - 999999 x[17,19] - start_service[17]
   + start_service[19] >= -999913.555315655
 NodeArrivalVehicles[17,20]: - 999999 x[17,20] - start_service[17]
   + start_service[20] >= -999923.0335188602
 NodeArrivalVehicles[17,21]: - 999999 x[17,21] - start_service[17]
   + start_service[21] >= -999943.1581189876
 NodeArrivalVehicles[17,22]: - 999999 x[17,22] - start_service[17]
   + start_service[22] >= -999941.126196999
 NodeArrivalVehicles[17,23]: - 999999 x[17,23] - start_service[17]
   + start_service[23] >= -999941.126196999
 NodeArrivalVehicles[18,1]: - 999999 x[18,1] + start_service[1]
   - start_service[18] >= -999951.7620508042
 NodeArrivalVehicles[18,2]: - 999999 x[18,2] + start_service[2]
   - start_service[18] >= -999979.7861738935
 NodeArrivalVehicles[18,3]: - 999999 x[18,3] + start_service[3]
   - start_service[18] >= -999973.4499059909
 NodeArrivalVehicles[18,4]: - 999999 x[18,4] + start_service[4]
   - start_service[18] >= -999957.9553616339
 NodeArrivalVehicles[18,5]: - 999999 x[18,5] + start_service[5]
   - start_service[18] >= -999918.6764967154
 NodeArrivalVehicles[18,6]: - 999999 x[18,6] + start_service[6]
   - start_service[18] >= -999915.2753088792
 NodeArrivalVehicles[18,7]: - 999999 x[18,7] + start_service[7]
   - start_service[18] >= -999952.7111189339
 NodeArrivalVehicles[18,8]: - 999999 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 999999 x[18,9] + start_service[9]
   - start_service[18] >= -999931.0123467562
 NodeArrivalVehicles[18,10]: - 999999 x[18,10] + start_service[10]
   - start_service[18] >= -999969.1712492406
 NodeArrivalVehicles[18,11]: - 999999 x[18,11] + start_service[11]
   - start_service[18] >= -999941.8023031916
 NodeArrivalVehicles[18,12]: - 999999 x[18,12] + start_service[12]
   - start_service[18] >= -999979.7861738935
 NodeArrivalVehicles[18,13]: - 999999 x[18,13] + start_service[13]
   - start_service[18] >= -999973.4499059909
 NodeArrivalVehicles[18,14]: - 999999 x[18,14] + start_service[14]
   - start_service[18] >= -999957.9553616339
 NodeArrivalVehicles[18,15]: - 999999 x[18,15] + start_service[15]
   - start_service[18] >= -999918.6764967154
 NodeArrivalVehicles[18,16]: - 999999 x[18,16] + start_service[16]
   - start_service[18] >= -999915.2753088792
 NodeArrivalVehicles[18,17]: - 999999 x[18,17] + start_service[17]
   - start_service[18] >= -999952.7111189339
 NodeArrivalVehicles[18,18]: - 999999 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 999999 x[18,19] - start_service[18]
   + start_service[19] >= -999931.0123467562
 NodeArrivalVehicles[18,20]: - 999999 x[18,20] - start_service[18]
   + start_service[20] >= -999969.1712492406
 NodeArrivalVehicles[18,21]: - 999999 x[18,21] - start_service[18]
   + start_service[21] >= -999941.8023031916
 NodeArrivalVehicles[18,22]: - 999999 x[18,22] - start_service[18]
   + start_service[22] >= -999951.7620508042
 NodeArrivalVehicles[18,23]: - 999999 x[18,23] - start_service[18]
   + start_service[23] >= -999951.7620508042
 NodeArrivalVehicles[19,1]: - 999999 x[19,1] + start_service[1]
   - start_service[19] >= -999971.4290013848
 NodeArrivalVehicles[19,2]: - 999999 x[19,2] + start_service[2]
   - start_service[19] >= -999931.6046053576
 NodeArrivalVehicles[19,3]: - 999999 x[19,3] + start_service[3]
   - start_service[19] >= -999936.1977742163
 NodeArrivalVehicles[19,4]: - 999999 x[19,4] + start_service[4]
   - start_service[19] >= -999910.2653522416
 NodeArrivalVehicles[19,5]: - 999999 x[19,5] + start_service[5]
   - start_service[19] >= -999971.5729838425
 NodeArrivalVehicles[19,6]: - 999999 x[19,6] + start_service[6]
   - start_service[19] >= -999957.385467226
 NodeArrivalVehicles[19,7]: - 999999 x[19,7] + start_service[7]
   - start_service[19] >= -999913.555315655
 NodeArrivalVehicles[19,8]: - 999999 x[19,8] + start_service[8]
   - start_service[19] >= -999931.0123467562
 NodeArrivalVehicles[19,9]: - 999999 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 999999 x[19,10] + start_service[10]
   - start_service[19] >= -999923.5677362148
 NodeArrivalVehicles[19,11]: - 999999 x[19,11] + start_service[11]
   - start_service[19] >= -999964.3876103099
 NodeArrivalVehicles[19,12]: - 999999 x[19,12] + start_service[12]
   - start_service[19] >= -999931.6046053576
 NodeArrivalVehicles[19,13]: - 999999 x[19,13] + start_service[13]
   - start_service[19] >= -999936.1977742163
 NodeArrivalVehicles[19,14]: - 999999 x[19,14] + start_service[14]
   - start_service[19] >= -999910.2653522416
 NodeArrivalVehicles[19,15]: - 999999 x[19,15] + start_service[15]
   - start_service[19] >= -999971.5729838425
 NodeArrivalVehicles[19,16]: - 999999 x[19,16] + start_service[16]
   - start_service[19] >= -999957.385467226
 NodeArrivalVehicles[19,17]: - 999999 x[19,17] + start_service[17]
   - start_service[19] >= -999913.555315655
 NodeArrivalVehicles[19,18]: - 999999 x[19,18] + start_service[18]
   - start_service[19] >= -999931.0123467562
 NodeArrivalVehicles[19,19]: - 999999 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 999999 x[19,20] - start_service[19]
   + start_service[20] >= -999923.5677362148
 NodeArrivalVehicles[19,21]: - 999999 x[19,21] - start_service[19]
   + start_service[21] >= -999964.3876103099
 NodeArrivalVehicles[19,22]: - 999999 x[19,22] - start_service[19]
   + start_service[22] >= -999971.4290013848
 NodeArrivalVehicles[19,23]: - 999999 x[19,23] - start_service[19]
   + start_service[23] >= -999971.4290013848
 NodeArrivalVehicles[20,1]: - 999999 x[20,1] + start_service[1]
   - start_service[20] >= -999934.7445159944
 NodeArrivalVehicles[20,2]: - 999999 x[20,2] + start_service[2]
   - start_service[20] >= -999987.1892001542
 NodeArrivalVehicles[20,3]: - 999999 x[20,3] + start_service[3]
   - start_service[20] >= -999944.4584358049
 NodeArrivalVehicles[20,4]: - 999999 x[20,4] + start_service[4]
   - start_service[20] >= -999929.0561937442
 NodeArrivalVehicles[20,5]: - 999999 x[20,5] + start_service[5]
   - start_service[20] >= -999921.6996578324
 NodeArrivalVehicles[20,6]: - 999999 x[20,6] + start_service[6]
   - start_service[20] >= -999923.9891144931
 NodeArrivalVehicles[20,7]: - 999999 x[20,7] + start_service[7]
   - start_service[20] >= -999923.0335188602
 NodeArrivalVehicles[20,8]: - 999999 x[20,8] + start_service[8]
   - start_service[20] >= -999969.1712492406
 NodeArrivalVehicles[20,9]: - 999999 x[20,9] + start_service[9]
   - start_service[20] >= -999923.5677362148
 NodeArrivalVehicles[20,10]: - 999999 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 999999 x[20,11] + start_service[11]
   - start_service[20] >= -999921.3827498653
 NodeArrivalVehicles[20,12]: - 999999 x[20,12] + start_service[12]
   - start_service[20] >= -999987.1892001542
 NodeArrivalVehicles[20,13]: - 999999 x[20,13] + start_service[13]
   - start_service[20] >= -999944.4584358049
 NodeArrivalVehicles[20,14]: - 999999 x[20,14] + start_service[14]
   - start_service[20] >= -999929.0561937442
 NodeArrivalVehicles[20,15]: - 999999 x[20,15] + start_service[15]
   - start_service[20] >= -999921.6996578324
 NodeArrivalVehicles[20,16]: - 999999 x[20,16] + start_service[16]
   - start_service[20] >= -999923.9891144931
 NodeArrivalVehicles[20,17]: - 999999 x[20,17] + start_service[17]
   - start_service[20] >= -999923.0335188602
 NodeArrivalVehicles[20,18]: - 999999 x[20,18] + start_service[18]
   - start_service[20] >= -999969.1712492406
 NodeArrivalVehicles[20,19]: - 999999 x[20,19] + start_service[19]
   - start_service[20] >= -999923.5677362148
 NodeArrivalVehicles[20,20]: - 999999 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 999999 x[20,21] - start_service[20]
   + start_service[21] >= -999921.3827498653
 NodeArrivalVehicles[20,22]: - 999999 x[20,22] - start_service[20]
   + start_service[22] >= -999934.7445159944
 NodeArrivalVehicles[20,23]: - 999999 x[20,23] - start_service[20]
   + start_service[23] >= -999934.7445159944
 NodeArrivalVehicles[21,1]: - 999999 x[21,1] + start_service[1]
   - start_service[21] >= -999984.1635755197
 NodeArrivalVehicles[21,2]: - 999999 x[21,2] + start_service[2]
   - start_service[21] >= -999932.5412105162
 NodeArrivalVehicles[21,3]: - 999999 x[21,3] + start_service[3]
   - start_service[21] >= -999959.1641715461
 NodeArrivalVehicles[21,4]: - 999999 x[21,4] + start_service[4]
   - start_service[21] >= -999937.3674259853
 NodeArrivalVehicles[21,5]: - 999999 x[21,5] + start_service[5]
   - start_service[21] >= -999937.5561410894
 NodeArrivalVehicles[21,6]: - 999999 x[21,6] + start_service[6]
   - start_service[21] >= -999924.7300771383
 NodeArrivalVehicles[21,7]: - 999999 x[21,7] + start_service[7]
   - start_service[21] >= -999943.1581189876
 NodeArrivalVehicles[21,8]: - 999999 x[21,8] + start_service[8]
   - start_service[21] >= -999941.8023031916
 NodeArrivalVehicles[21,9]: - 999999 x[21,9] + start_service[9]
   - start_service[21] >= -999964.3876103099
 NodeArrivalVehicles[21,10]: - 999999 x[21,10] + start_service[10]
   - start_service[21] >= -999921.3827498653
 NodeArrivalVehicles[21,11]: - 999999 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 999999 x[21,12] + start_service[12]
   - start_service[21] >= -999932.5412105162
 NodeArrivalVehicles[21,13]: - 999999 x[21,13] + start_service[13]
   - start_service[21] >= -999959.1641715461
 NodeArrivalVehicles[21,14]: - 999999 x[21,14] + start_service[14]
   - start_service[21] >= -999937.3674259853
 NodeArrivalVehicles[21,15]: - 999999 x[21,15] + start_service[15]
   - start_service[21] >= -999937.5561410894
 NodeArrivalVehicles[21,16]: - 999999 x[21,16] + start_service[16]
   - start_service[21] >= -999924.7300771383
 NodeArrivalVehicles[21,17]: - 999999 x[21,17] + start_service[17]
   - start_service[21] >= -999943.1581189876
 NodeArrivalVehicles[21,18]: - 999999 x[21,18] + start_service[18]
   - start_service[21] >= -999941.8023031916
 NodeArrivalVehicles[21,19]: - 999999 x[21,19] + start_service[19]
   - start_service[21] >= -999964.3876103099
 NodeArrivalVehicles[21,20]: - 999999 x[21,20] + start_service[20]
   - start_service[21] >= -999921.3827498653
 NodeArrivalVehicles[21,21]: - 999999 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 999999 x[21,22] - start_service[21]
   + start_service[22] >= -999984.1635755197
 NodeArrivalVehicles[21,23]: - 999999 x[21,23] - start_service[21]
   + start_service[23] >= -999984.1635755197
 NodeArrivalVehicles[22,1]: - 999999 x[22,1] + start_service[1]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 999999 x[22,2] + start_service[2]
   - start_service[22] >= -999945.4287857366
 NodeArrivalVehicles[22,3]: - 999999 x[22,3] + start_service[3]
   - start_service[22] >= -999962.9193004568
 NodeArrivalVehicles[22,4]: - 999999 x[22,4] + start_service[4]
   - start_service[22] >= -999937.583726213
 NodeArrivalVehicles[22,5]: - 999999 x[22,5] + start_service[5]
   - start_service[22] >= -999947.3853854308
 NodeArrivalVehicles[22,6]: - 999999 x[22,6] + start_service[6]
   - start_service[22] >= -999936.1484321967
 NodeArrivalVehicles[22,7]: - 999999 x[22,7] + start_service[7]
   - start_service[22] >= -999941.126196999
 NodeArrivalVehicles[22,8]: - 999999 x[22,8] + start_service[8]
   - start_service[22] >= -999951.7620508042
 NodeArrivalVehicles[22,9]: - 999999 x[22,9] + start_service[9]
   - start_service[22] >= -999971.4290013848
 NodeArrivalVehicles[22,10]: - 999999 x[22,10] + start_service[10]
   - start_service[22] >= -999934.7445159944
 NodeArrivalVehicles[22,11]: - 999999 x[22,11] + start_service[11]
   - start_service[22] >= -999984.1635755197
 NodeArrivalVehicles[22,12]: - 999999 x[22,12] + start_service[12]
   - start_service[22] >= -999945.4287857366
 NodeArrivalVehicles[22,13]: - 999999 x[22,13] + start_service[13]
   - start_service[22] >= -999962.9193004568
 NodeArrivalVehicles[22,14]: - 999999 x[22,14] + start_service[14]
   - start_service[22] >= -999937.583726213
 NodeArrivalVehicles[22,15]: - 999999 x[22,15] + start_service[15]
   - start_service[22] >= -999947.3853854308
 NodeArrivalVehicles[22,16]: - 999999 x[22,16] + start_service[16]
   - start_service[22] >= -999936.1484321967
 NodeArrivalVehicles[22,17]: - 999999 x[22,17] + start_service[17]
   - start_service[22] >= -999941.126196999
 NodeArrivalVehicles[22,18]: - 999999 x[22,18] + start_service[18]
   - start_service[22] >= -999951.7620508042
 NodeArrivalVehicles[22,19]: - 999999 x[22,19] + start_service[19]
   - start_service[22] >= -999971.4290013848
 NodeArrivalVehicles[22,20]: - 999999 x[22,20] + start_service[20]
   - start_service[22] >= -999934.7445159944
 NodeArrivalVehicles[22,21]: - 999999 x[22,21] + start_service[21]
   - start_service[22] >= -999984.1635755197
 NodeArrivalVehicles[22,22]: - 999999 x[22,22] >= -999999
 NodeArrivalVehicles[22,23]: - 999999 x[22,23] - start_service[22]
   + start_service[23] >= -999999
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[0,14] + w[0,15] + w[0,16] + w[0,17] + w[0,18] + w[0,19]
   + w[0,20] + w[0,21] + w[0,22] + w[0,23] <= 6
 TotalRobotStarts: w[0,2] + w[0,3] + w[0,4] + w[0,5] + w[0,6] + w[0,7]
   + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12] + w[0,13] + w[0,14]
   + w[0,15] + w[0,16] + w[0,17] + w[0,18] + w[0,19] + w[0,20] + w[0,21]
   + w[0,22] + w[0,23] + w[1,2] + w[1,3] + w[1,4] + w[1,5] + w[1,6]
   + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12] + w[1,13]
   + w[1,14] + w[1,15] + w[1,16] + w[1,17] + w[1,18] + w[1,19] + w[1,20]
   + w[1,21] + w[1,22] + w[1,23] <= 7
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] + w[1,14] + w[1,15] + w[1,16] + w[1,17] + w[1,18] + w[1,19]
   + w[1,20] + w[1,21] + w[1,22] + w[1,23] <= 1
 RobotsFinishAtVehicleDepot: w[0,23] + w[1,23] + w[2,23] + w[3,23]
   + w[4,23] + w[5,23] + w[6,23] + w[7,23] + w[8,23] + w[9,23] + w[10,23]
   + w[11,23] + w[12,23] + w[13,23] + w[14,23] + w[15,23] + w[16,23]
   + w[17,23] + w[18,23] + w[19,23] + w[20,23] + w[21,23] + w[22,23] <= 6
 TotalRobotFinishes: w[0,22] + w[0,23] + w[1,22] + w[1,23] + w[2,22]
   + w[2,23] + w[3,22] + w[3,23] + w[4,22] + w[4,23] + w[5,22] + w[5,23]
   + w[6,22] + w[6,23] + w[7,22] + w[7,23] + w[8,22] + w[8,23] + w[9,22]
   + w[9,23] + w[10,22] + w[10,23] + w[11,22] + w[11,23] + w[12,22]
   + w[12,23] + w[13,22] + w[13,23] + w[14,22] + w[14,23] + w[15,22]
   + w[15,23] + w[16,22] + w[16,23] + w[17,22] + w[17,23] + w[18,22]
   + w[18,23] + w[19,22] + w[19,23] + w[20,22] + w[20,23] + w[21,22]
   + w[21,23] <= 7
 FlowConservationRobots[2]: w[0,2] + w[1,2] - w[2,1] - w[2,3] - w[2,4]
   - w[2,5] - w[2,6] - w[2,7] - w[2,8] - w[2,9] - w[2,10] - w[2,11]
   - w[2,12] - w[2,13] - w[2,14] - w[2,15] - w[2,16] - w[2,17] - w[2,18]
   - w[2,19] - w[2,20] - w[2,21] - w[2,22] - w[2,23] + w[3,2] + w[4,2]
   + w[5,2] + w[6,2] + w[7,2] + w[8,2] + w[9,2] + w[10,2] + w[11,2]
   + w[12,2] + w[13,2] + w[14,2] + w[15,2] + w[16,2] + w[17,2] + w[18,2]
   + w[19,2] + w[20,2] + w[21,2] + w[22,2] = 0
 FlowConservationRobots[3]: w[0,3] + w[1,3] + w[2,3] - w[3,1] - w[3,2]
   - w[3,4] - w[3,5] - w[3,6] - w[3,7] - w[3,8] - w[3,9] - w[3,10]
   - w[3,11] - w[3,12] - w[3,13] - w[3,14] - w[3,15] - w[3,16] - w[3,17]
   - w[3,18] - w[3,19] - w[3,20] - w[3,21] - w[3,22] - w[3,23] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] + w[13,3] + w[14,3] + w[15,3] + w[16,3] + w[17,3] + w[18,3]
   + w[19,3] + w[20,3] + w[21,3] + w[22,3] = 0
 FlowConservationRobots[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] - w[4,1]
   - w[4,2] - w[4,3] - w[4,5] - w[4,6] - w[4,7] - w[4,8] - w[4,9] - w[4,10]
   - w[4,11] - w[4,12] - w[4,13] - w[4,14] - w[4,15] - w[4,16] - w[4,17]
   - w[4,18] - w[4,19] - w[4,20] - w[4,21] - w[4,22] - w[4,23] + w[5,4]
   + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4] + w[12,4]
   + w[13,4] + w[14,4] + w[15,4] + w[16,4] + w[17,4] + w[18,4] + w[19,4]
   + w[20,4] + w[21,4] + w[22,4] = 0
 FlowConservationRobots[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   - w[5,1] - w[5,2] - w[5,3] - w[5,4] - w[5,6] - w[5,7] - w[5,8] - w[5,9]
   - w[5,10] - w[5,11] - w[5,12] - w[5,13] - w[5,14] - w[5,15] - w[5,16]
   - w[5,17] - w[5,18] - w[5,19] - w[5,20] - w[5,21] - w[5,22] - w[5,23]
   + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5] + w[12,5]
   + w[13,5] + w[14,5] + w[15,5] + w[16,5] + w[17,5] + w[18,5] + w[19,5]
   + w[20,5] + w[21,5] + w[22,5] = 0
 FlowConservationRobots[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] - w[6,1] - w[6,2] - w[6,3] - w[6,4] - w[6,5] - w[6,7] - w[6,8]
   - w[6,9] - w[6,10] - w[6,11] - w[6,12] - w[6,13] - w[6,14] - w[6,15]
   - w[6,16] - w[6,17] - w[6,18] - w[6,19] - w[6,20] - w[6,21] - w[6,22]
   - w[6,23] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6] + w[12,6]
   + w[13,6] + w[14,6] + w[15,6] + w[16,6] + w[17,6] + w[18,6] + w[19,6]
   + w[20,6] + w[21,6] + w[22,6] = 0
 FlowConservationRobots[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] - w[7,1] - w[7,2] - w[7,3] - w[7,4] - w[7,5] - w[7,6]
   - w[7,8] - w[7,9] - w[7,10] - w[7,11] - w[7,12] - w[7,13] - w[7,14]
   - w[7,15] - w[7,16] - w[7,17] - w[7,18] - w[7,19] - w[7,20] - w[7,21]
   - w[7,22] - w[7,23] + w[8,7] + w[9,7] + w[10,7] + w[11,7] + w[12,7]
   + w[13,7] + w[14,7] + w[15,7] + w[16,7] + w[17,7] + w[18,7] + w[19,7]
   + w[20,7] + w[21,7] + w[22,7] = 0
 FlowConservationRobots[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] - w[8,1] - w[8,2] - w[8,3] - w[8,4] - w[8,5]
   - w[8,6] - w[8,7] - w[8,9] - w[8,10] - w[8,11] - w[8,12] - w[8,13]
   - w[8,14] - w[8,15] - w[8,16] - w[8,17] - w[8,18] - w[8,19] - w[8,20]
   - w[8,21] - w[8,22] - w[8,23] + w[9,8] + w[10,8] + w[11,8] + w[12,8]
   + w[13,8] + w[14,8] + w[15,8] + w[16,8] + w[17,8] + w[18,8] + w[19,8]
   + w[20,8] + w[21,8] + w[22,8] = 0
 FlowConservationRobots[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] - w[9,1] - w[9,2] - w[9,3] - w[9,4]
   - w[9,5] - w[9,6] - w[9,7] - w[9,8] - w[9,10] - w[9,11] - w[9,12]
   - w[9,13] - w[9,14] - w[9,15] - w[9,16] - w[9,17] - w[9,18] - w[9,19]
   - w[9,20] - w[9,21] - w[9,22] - w[9,23] + w[10,9] + w[11,9] + w[12,9]
   + w[13,9] + w[14,9] + w[15,9] + w[16,9] + w[17,9] + w[18,9] + w[19,9]
   + w[20,9] + w[21,9] + w[22,9] = 0
 FlowConservationRobots[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] - w[10,1]
   - w[10,2] - w[10,3] - w[10,4] - w[10,5] - w[10,6] - w[10,7] - w[10,8]
   - w[10,9] - w[10,11] - w[10,12] - w[10,13] - w[10,14] - w[10,15]
   - w[10,16] - w[10,17] - w[10,18] - w[10,19] - w[10,20] - w[10,21]
   - w[10,22] - w[10,23] + w[11,10] + w[12,10] + w[13,10] + w[14,10]
   + w[15,10] + w[16,10] + w[17,10] + w[18,10] + w[19,10] + w[20,10]
   + w[21,10] + w[22,10] = 0
 FlowConservationRobots[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   - w[11,1] - w[11,2] - w[11,3] - w[11,4] - w[11,5] - w[11,6] - w[11,7]
   - w[11,8] - w[11,9] - w[11,10] - w[11,12] - w[11,13] - w[11,14]
   - w[11,15] - w[11,16] - w[11,17] - w[11,18] - w[11,19] - w[11,20]
   - w[11,21] - w[11,22] - w[11,23] + w[12,11] + w[13,11] + w[14,11]
   + w[15,11] + w[16,11] + w[17,11] + w[18,11] + w[19,11] + w[20,11]
   + w[21,11] + w[22,11] = 0
 FlowConservationRobots[12]: w[0,12] + w[1,12] + w[2,12] + w[3,12]
   + w[4,12] + w[5,12] + w[6,12] + w[7,12] + w[8,12] + w[9,12] + w[10,12]
   + w[11,12] - w[12,1] - w[12,2] - w[12,3] - w[12,4] - w[12,5] - w[12,6]
   - w[12,7] - w[12,8] - w[12,9] - w[12,10] - w[12,11] - w[12,13]
   - w[12,14] - w[12,15] - w[12,16] - w[12,17] - w[12,18] - w[12,19]
   - w[12,20] - w[12,21] - w[12,22] - w[12,23] + w[13,12] + w[14,12]
   + w[15,12] + w[16,12] + w[17,12] + w[18,12] + w[19,12] + w[20,12]
   + w[21,12] + w[22,12] = 0
 FlowConservationRobots[13]: w[0,13] + w[1,13] + w[2,13] + w[3,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] - w[13,1] - w[13,2] - w[13,3] - w[13,4] - w[13,5]
   - w[13,6] - w[13,7] - w[13,8] - w[13,9] - w[13,10] - w[13,11] - w[13,12]
   - w[13,14] - w[13,15] - w[13,16] - w[13,17] - w[13,18] - w[13,19]
   - w[13,20] - w[13,21] - w[13,22] - w[13,23] + w[14,13] + w[15,13]
   + w[16,13] + w[17,13] + w[18,13] + w[19,13] + w[20,13] + w[21,13]
   + w[22,13] = 0
 FlowConservationRobots[14]: w[0,14] + w[1,14] + w[2,14] + w[3,14]
   + w[4,14] + w[5,14] + w[6,14] + w[7,14] + w[8,14] + w[9,14] + w[10,14]
   + w[11,14] + w[12,14] + w[13,14] - w[14,1] - w[14,2] - w[14,3] - w[14,4]
   - w[14,5] - w[14,6] - w[14,7] - w[14,8] - w[14,9] - w[14,10] - w[14,11]
   - w[14,12] - w[14,13] - w[14,15] - w[14,16] - w[14,17] - w[14,18]
   - w[14,19] - w[14,20] - w[14,21] - w[14,22] - w[14,23] + w[15,14]
   + w[16,14] + w[17,14] + w[18,14] + w[19,14] + w[20,14] + w[21,14]
   + w[22,14] = 0
 FlowConservationRobots[15]: w[0,15] + w[1,15] + w[2,15] + w[3,15]
   + w[4,15] + w[5,15] + w[6,15] + w[7,15] + w[8,15] + w[9,15] + w[10,15]
   + w[11,15] + w[12,15] + w[13,15] + w[14,15] - w[15,1] - w[15,2]
   - w[15,3] - w[15,4] - w[15,5] - w[15,6] - w[15,7] - w[15,8] - w[15,9]
   - w[15,10] - w[15,11] - w[15,12] - w[15,13] - w[15,14] - w[15,16]
   - w[15,17] - w[15,18] - w[15,19] - w[15,20] - w[15,21] - w[15,22]
   - w[15,23] + w[16,15] + w[17,15] + w[18,15] + w[19,15] + w[20,15]
   + w[21,15] + w[22,15] = 0
 FlowConservationRobots[16]: w[0,16] + w[1,16] + w[2,16] + w[3,16]
   + w[4,16] + w[5,16] + w[6,16] + w[7,16] + w[8,16] + w[9,16] + w[10,16]
   + w[11,16] + w[12,16] + w[13,16] + w[14,16] + w[15,16] - w[16,1]
   - w[16,2] - w[16,3] - w[16,4] - w[16,5] - w[16,6] - w[16,7] - w[16,8]
   - w[16,9] - w[16,10] - w[16,11] - w[16,12] - w[16,13] - w[16,14]
   - w[16,15] - w[16,17] - w[16,18] - w[16,19] - w[16,20] - w[16,21]
   - w[16,22] - w[16,23] + w[17,16] + w[18,16] + w[19,16] + w[20,16]
   + w[21,16] + w[22,16] = 0
 FlowConservationRobots[17]: w[0,17] + w[1,17] + w[2,17] + w[3,17]
   + w[4,17] + w[5,17] + w[6,17] + w[7,17] + w[8,17] + w[9,17] + w[10,17]
   + w[11,17] + w[12,17] + w[13,17] + w[14,17] + w[15,17] + w[16,17]
   - w[17,1] - w[17,2] - w[17,3] - w[17,4] - w[17,5] - w[17,6] - w[17,7]
   - w[17,8] - w[17,9] - w[17,10] - w[17,11] - w[17,12] - w[17,13]
   - w[17,14] - w[17,15] - w[17,16] - w[17,18] - w[17,19] - w[17,20]
   - w[17,21] - w[17,22] - w[17,23] + w[18,17] + w[19,17] + w[20,17]
   + w[21,17] + w[22,17] = 0
 FlowConservationRobots[18]: w[0,18] + w[1,18] + w[2,18] + w[3,18]
   + w[4,18] + w[5,18] + w[6,18] + w[7,18] + w[8,18] + w[9,18] + w[10,18]
   + w[11,18] + w[12,18] + w[13,18] + w[14,18] + w[15,18] + w[16,18]
   + w[17,18] - w[18,1] - w[18,2] - w[18,3] - w[18,4] - w[18,5] - w[18,6]
   - w[18,7] - w[18,8] - w[18,9] - w[18,10] - w[18,11] - w[18,12]
   - w[18,13] - w[18,14] - w[18,15] - w[18,16] - w[18,17] - w[18,19]
   - w[18,20] - w[18,21] - w[18,22] - w[18,23] + w[19,18] + w[20,18]
   + w[21,18] + w[22,18] = 0
 FlowConservationRobots[19]: w[0,19] + w[1,19] + w[2,19] + w[3,19]
   + w[4,19] + w[5,19] + w[6,19] + w[7,19] + w[8,19] + w[9,19] + w[10,19]
   + w[11,19] + w[12,19] + w[13,19] + w[14,19] + w[15,19] + w[16,19]
   + w[17,19] + w[18,19] - w[19,1] - w[19,2] - w[19,3] - w[19,4] - w[19,5]
   - w[19,6] - w[19,7] - w[19,8] - w[19,9] - w[19,10] - w[19,11] - w[19,12]
   - w[19,13] - w[19,14] - w[19,15] - w[19,16] - w[19,17] - w[19,18]
   - w[19,20] - w[19,21] - w[19,22] - w[19,23] + w[20,19] + w[21,19]
   + w[22,19] = 0
 FlowConservationRobots[20]: w[0,20] + w[1,20] + w[2,20] + w[3,20]
   + w[4,20] + w[5,20] + w[6,20] + w[7,20] + w[8,20] + w[9,20] + w[10,20]
   + w[11,20] + w[12,20] + w[13,20] + w[14,20] + w[15,20] + w[16,20]
   + w[17,20] + w[18,20] + w[19,20] - w[20,1] - w[20,2] - w[20,3] - w[20,4]
   - w[20,5] - w[20,6] - w[20,7] - w[20,8] - w[20,9] - w[20,10] - w[20,11]
   - w[20,12] - w[20,13] - w[20,14] - w[20,15] - w[20,16] - w[20,17]
   - w[20,18] - w[20,19] - w[20,21] - w[20,22] - w[20,23] + w[21,20]
   + w[22,20] = 0
 FlowConservationRobots[21]: w[0,21] + w[1,21] + w[2,21] + w[3,21]
   + w[4,21] + w[5,21] + w[6,21] + w[7,21] + w[8,21] + w[9,21] + w[10,21]
   + w[11,21] + w[12,21] + w[13,21] + w[14,21] + w[15,21] + w[16,21]
   + w[17,21] + w[18,21] + w[19,21] + w[20,21] - w[21,1] - w[21,2]
   - w[21,3] - w[21,4] - w[21,5] - w[21,6] - w[21,7] - w[21,8] - w[21,9]
   - w[21,10] - w[21,11] - w[21,12] - w[21,13] - w[21,14] - w[21,15]
   - w[21,16] - w[21,17] - w[21,18] - w[21,19] - w[21,20] - w[21,22]
   - w[21,23] + w[22,21] = 0
 MaxNumberSupportOnArc[0,1]: w[0,1] - 2 v[0,1] <= 0
 MaxNumberSupportOnArc[0,2]: w[0,2] - 2 v[0,2] <= 0
 MaxNumberSupportOnArc[0,3]: w[0,3] - 2 v[0,3] <= 0
 MaxNumberSupportOnArc[0,4]: w[0,4] - 2 v[0,4] <= 0
 MaxNumberSupportOnArc[0,5]: w[0,5] - 2 v[0,5] <= 0
 MaxNumberSupportOnArc[0,6]: w[0,6] - 2 v[0,6] <= 0
 MaxNumberSupportOnArc[0,7]: w[0,7] - 2 v[0,7] <= 0
 MaxNumberSupportOnArc[0,8]: w[0,8] - 2 v[0,8] <= 0
 MaxNumberSupportOnArc[0,9]: w[0,9] - 2 v[0,9] <= 0
 MaxNumberSupportOnArc[0,10]: w[0,10] - 2 v[0,10] <= 0
 MaxNumberSupportOnArc[0,11]: w[0,11] - 2 v[0,11] <= 0
 MaxNumberSupportOnArc[0,12]: w[0,12] - 2 v[0,12] <= 0
 MaxNumberSupportOnArc[0,13]: w[0,13] - 2 v[0,13] <= 0
 MaxNumberSupportOnArc[0,14]: w[0,14] - 2 v[0,14] <= 0
 MaxNumberSupportOnArc[0,15]: w[0,15] - 2 v[0,15] <= 0
 MaxNumberSupportOnArc[0,16]: w[0,16] - 2 v[0,16] <= 0
 MaxNumberSupportOnArc[0,17]: w[0,17] - 2 v[0,17] <= 0
 MaxNumberSupportOnArc[0,18]: w[0,18] - 2 v[0,18] <= 0
 MaxNumberSupportOnArc[0,19]: w[0,19] - 2 v[0,19] <= 0
 MaxNumberSupportOnArc[0,20]: w[0,20] - 2 v[0,20] <= 0
 MaxNumberSupportOnArc[0,21]: w[0,21] - 2 v[0,21] <= 0
 MaxNumberSupportOnArc[0,22]: w[0,22] - 2 v[0,22] <= 0
 MaxNumberSupportOnArc[0,23]: w[0,23] - 2 v[0,23] <= 0
 MaxNumberSupportOnArc[1,1]: w[1,1] - 2 v[1,1] <= 0
 MaxNumberSupportOnArc[1,2]: w[1,2] - 2 v[1,2] <= 0
 MaxNumberSupportOnArc[1,3]: w[1,3] - 2 v[1,3] <= 0
 MaxNumberSupportOnArc[1,4]: w[1,4] - 2 v[1,4] <= 0
 MaxNumberSupportOnArc[1,5]: w[1,5] - 2 v[1,5] <= 0
 MaxNumberSupportOnArc[1,6]: w[1,6] - 2 v[1,6] <= 0
 MaxNumberSupportOnArc[1,7]: w[1,7] - 2 v[1,7] <= 0
 MaxNumberSupportOnArc[1,8]: w[1,8] - 2 v[1,8] <= 0
 MaxNumberSupportOnArc[1,9]: w[1,9] - 2 v[1,9] <= 0
 MaxNumberSupportOnArc[1,10]: w[1,10] - 2 v[1,10] <= 0
 MaxNumberSupportOnArc[1,11]: w[1,11] - 2 v[1,11] <= 0
 MaxNumberSupportOnArc[1,12]: w[1,12] - 2 v[1,12] <= 0
 MaxNumberSupportOnArc[1,13]: w[1,13] - 2 v[1,13] <= 0
 MaxNumberSupportOnArc[1,14]: w[1,14] - 2 v[1,14] <= 0
 MaxNumberSupportOnArc[1,15]: w[1,15] - 2 v[1,15] <= 0
 MaxNumberSupportOnArc[1,16]: w[1,16] - 2 v[1,16] <= 0
 MaxNumberSupportOnArc[1,17]: w[1,17] - 2 v[1,17] <= 0
 MaxNumberSupportOnArc[1,18]: w[1,18] - 2 v[1,18] <= 0
 MaxNumberSupportOnArc[1,19]: w[1,19] - 2 v[1,19] <= 0
 MaxNumberSupportOnArc[1,20]: w[1,20] - 2 v[1,20] <= 0
 MaxNumberSupportOnArc[1,21]: w[1,21] - 2 v[1,21] <= 0
 MaxNumberSupportOnArc[1,22]: w[1,22] - 2 v[1,22] <= 0
 MaxNumberSupportOnArc[1,23]: w[1,23] - 2 v[1,23] <= 0
 MaxNumberSupportOnArc[2,1]: w[2,1] - 2 v[2,1] <= 0
 MaxNumberSupportOnArc[2,2]: w[2,2] - 2 v[2,2] <= 0
 MaxNumberSupportOnArc[2,3]: w[2,3] - 2 v[2,3] <= 0
 MaxNumberSupportOnArc[2,4]: w[2,4] - 2 v[2,4] <= 0
 MaxNumberSupportOnArc[2,5]: w[2,5] - 2 v[2,5] <= 0
 MaxNumberSupportOnArc[2,6]: w[2,6] - 2 v[2,6] <= 0
 MaxNumberSupportOnArc[2,7]: w[2,7] - 2 v[2,7] <= 0
 MaxNumberSupportOnArc[2,8]: w[2,8] - 2 v[2,8] <= 0
 MaxNumberSupportOnArc[2,9]: w[2,9] - 2 v[2,9] <= 0
 MaxNumberSupportOnArc[2,10]: w[2,10] - 2 v[2,10] <= 0
 MaxNumberSupportOnArc[2,11]: w[2,11] - 2 v[2,11] <= 0
 MaxNumberSupportOnArc[2,12]: w[2,12] - 2 v[2,12] <= 0
 MaxNumberSupportOnArc[2,13]: w[2,13] - 2 v[2,13] <= 0
 MaxNumberSupportOnArc[2,14]: w[2,14] - 2 v[2,14] <= 0
 MaxNumberSupportOnArc[2,15]: w[2,15] - 2 v[2,15] <= 0
 MaxNumberSupportOnArc[2,16]: w[2,16] - 2 v[2,16] <= 0
 MaxNumberSupportOnArc[2,17]: w[2,17] - 2 v[2,17] <= 0
 MaxNumberSupportOnArc[2,18]: w[2,18] - 2 v[2,18] <= 0
 MaxNumberSupportOnArc[2,19]: w[2,19] - 2 v[2,19] <= 0
 MaxNumberSupportOnArc[2,20]: w[2,20] - 2 v[2,20] <= 0
 MaxNumberSupportOnArc[2,21]: w[2,21] - 2 v[2,21] <= 0
 MaxNumberSupportOnArc[2,22]: w[2,22] - 2 v[2,22] <= 0
 MaxNumberSupportOnArc[2,23]: w[2,23] - 2 v[2,23] <= 0
 MaxNumberSupportOnArc[3,1]: w[3,1] - 2 v[3,1] <= 0
 MaxNumberSupportOnArc[3,2]: w[3,2] - 2 v[3,2] <= 0
 MaxNumberSupportOnArc[3,3]: w[3,3] - 2 v[3,3] <= 0
 MaxNumberSupportOnArc[3,4]: w[3,4] - 2 v[3,4] <= 0
 MaxNumberSupportOnArc[3,5]: w[3,5] - 2 v[3,5] <= 0
 MaxNumberSupportOnArc[3,6]: w[3,6] - 2 v[3,6] <= 0
 MaxNumberSupportOnArc[3,7]: w[3,7] - 2 v[3,7] <= 0
 MaxNumberSupportOnArc[3,8]: w[3,8] - 2 v[3,8] <= 0
 MaxNumberSupportOnArc[3,9]: w[3,9] - 2 v[3,9] <= 0
 MaxNumberSupportOnArc[3,10]: w[3,10] - 2 v[3,10] <= 0
 MaxNumberSupportOnArc[3,11]: w[3,11] - 2 v[3,11] <= 0
 MaxNumberSupportOnArc[3,12]: w[3,12] - 2 v[3,12] <= 0
 MaxNumberSupportOnArc[3,13]: w[3,13] - 2 v[3,13] <= 0
 MaxNumberSupportOnArc[3,14]: w[3,14] - 2 v[3,14] <= 0
 MaxNumberSupportOnArc[3,15]: w[3,15] - 2 v[3,15] <= 0
 MaxNumberSupportOnArc[3,16]: w[3,16] - 2 v[3,16] <= 0
 MaxNumberSupportOnArc[3,17]: w[3,17] - 2 v[3,17] <= 0
 MaxNumberSupportOnArc[3,18]: w[3,18] - 2 v[3,18] <= 0
 MaxNumberSupportOnArc[3,19]: w[3,19] - 2 v[3,19] <= 0
 MaxNumberSupportOnArc[3,20]: w[3,20] - 2 v[3,20] <= 0
 MaxNumberSupportOnArc[3,21]: w[3,21] - 2 v[3,21] <= 0
 MaxNumberSupportOnArc[3,22]: w[3,22] - 2 v[3,22] <= 0
 MaxNumberSupportOnArc[3,23]: w[3,23] - 2 v[3,23] <= 0
 MaxNumberSupportOnArc[4,1]: w[4,1] - 2 v[4,1] <= 0
 MaxNumberSupportOnArc[4,2]: w[4,2] - 2 v[4,2] <= 0
 MaxNumberSupportOnArc[4,3]: w[4,3] - 2 v[4,3] <= 0
 MaxNumberSupportOnArc[4,4]: w[4,4] - 2 v[4,4] <= 0
 MaxNumberSupportOnArc[4,5]: w[4,5] - 2 v[4,5] <= 0
 MaxNumberSupportOnArc[4,6]: w[4,6] - 2 v[4,6] <= 0
 MaxNumberSupportOnArc[4,7]: w[4,7] - 2 v[4,7] <= 0
 MaxNumberSupportOnArc[4,8]: w[4,8] - 2 v[4,8] <= 0
 MaxNumberSupportOnArc[4,9]: w[4,9] - 2 v[4,9] <= 0
 MaxNumberSupportOnArc[4,10]: w[4,10] - 2 v[4,10] <= 0
 MaxNumberSupportOnArc[4,11]: w[4,11] - 2 v[4,11] <= 0
 MaxNumberSupportOnArc[4,12]: w[4,12] - 2 v[4,12] <= 0
 MaxNumberSupportOnArc[4,13]: w[4,13] - 2 v[4,13] <= 0
 MaxNumberSupportOnArc[4,14]: w[4,14] - 2 v[4,14] <= 0
 MaxNumberSupportOnArc[4,15]: w[4,15] - 2 v[4,15] <= 0
 MaxNumberSupportOnArc[4,16]: w[4,16] - 2 v[4,16] <= 0
 MaxNumberSupportOnArc[4,17]: w[4,17] - 2 v[4,17] <= 0
 MaxNumberSupportOnArc[4,18]: w[4,18] - 2 v[4,18] <= 0
 MaxNumberSupportOnArc[4,19]: w[4,19] - 2 v[4,19] <= 0
 MaxNumberSupportOnArc[4,20]: w[4,20] - 2 v[4,20] <= 0
 MaxNumberSupportOnArc[4,21]: w[4,21] - 2 v[4,21] <= 0
 MaxNumberSupportOnArc[4,22]: w[4,22] - 2 v[4,22] <= 0
 MaxNumberSupportOnArc[4,23]: w[4,23] - 2 v[4,23] <= 0
 MaxNumberSupportOnArc[5,1]: w[5,1] - 2 v[5,1] <= 0
 MaxNumberSupportOnArc[5,2]: w[5,2] - 2 v[5,2] <= 0
 MaxNumberSupportOnArc[5,3]: w[5,3] - 2 v[5,3] <= 0
 MaxNumberSupportOnArc[5,4]: w[5,4] - 2 v[5,4] <= 0
 MaxNumberSupportOnArc[5,5]: w[5,5] - 2 v[5,5] <= 0
 MaxNumberSupportOnArc[5,6]: w[5,6] - 2 v[5,6] <= 0
 MaxNumberSupportOnArc[5,7]: w[5,7] - 2 v[5,7] <= 0
 MaxNumberSupportOnArc[5,8]: w[5,8] - 2 v[5,8] <= 0
 MaxNumberSupportOnArc[5,9]: w[5,9] - 2 v[5,9] <= 0
 MaxNumberSupportOnArc[5,10]: w[5,10] - 2 v[5,10] <= 0
 MaxNumberSupportOnArc[5,11]: w[5,11] - 2 v[5,11] <= 0
 MaxNumberSupportOnArc[5,12]: w[5,12] - 2 v[5,12] <= 0
 MaxNumberSupportOnArc[5,13]: w[5,13] - 2 v[5,13] <= 0
 MaxNumberSupportOnArc[5,14]: w[5,14] - 2 v[5,14] <= 0
 MaxNumberSupportOnArc[5,15]: w[5,15] - 2 v[5,15] <= 0
 MaxNumberSupportOnArc[5,16]: w[5,16] - 2 v[5,16] <= 0
 MaxNumberSupportOnArc[5,17]: w[5,17] - 2 v[5,17] <= 0
 MaxNumberSupportOnArc[5,18]: w[5,18] - 2 v[5,18] <= 0
 MaxNumberSupportOnArc[5,19]: w[5,19] - 2 v[5,19] <= 0
 MaxNumberSupportOnArc[5,20]: w[5,20] - 2 v[5,20] <= 0
 MaxNumberSupportOnArc[5,21]: w[5,21] - 2 v[5,21] <= 0
 MaxNumberSupportOnArc[5,22]: w[5,22] - 2 v[5,22] <= 0
 MaxNumberSupportOnArc[5,23]: w[5,23] - 2 v[5,23] <= 0
 MaxNumberSupportOnArc[6,1]: w[6,1] - 2 v[6,1] <= 0
 MaxNumberSupportOnArc[6,2]: w[6,2] - 2 v[6,2] <= 0
 MaxNumberSupportOnArc[6,3]: w[6,3] - 2 v[6,3] <= 0
 MaxNumberSupportOnArc[6,4]: w[6,4] - 2 v[6,4] <= 0
 MaxNumberSupportOnArc[6,5]: w[6,5] - 2 v[6,5] <= 0
 MaxNumberSupportOnArc[6,6]: w[6,6] - 2 v[6,6] <= 0
 MaxNumberSupportOnArc[6,7]: w[6,7] - 2 v[6,7] <= 0
 MaxNumberSupportOnArc[6,8]: w[6,8] - 2 v[6,8] <= 0
 MaxNumberSupportOnArc[6,9]: w[6,9] - 2 v[6,9] <= 0
 MaxNumberSupportOnArc[6,10]: w[6,10] - 2 v[6,10] <= 0
 MaxNumberSupportOnArc[6,11]: w[6,11] - 2 v[6,11] <= 0
 MaxNumberSupportOnArc[6,12]: w[6,12] - 2 v[6,12] <= 0
 MaxNumberSupportOnArc[6,13]: w[6,13] - 2 v[6,13] <= 0
 MaxNumberSupportOnArc[6,14]: w[6,14] - 2 v[6,14] <= 0
 MaxNumberSupportOnArc[6,15]: w[6,15] - 2 v[6,15] <= 0
 MaxNumberSupportOnArc[6,16]: w[6,16] - 2 v[6,16] <= 0
 MaxNumberSupportOnArc[6,17]: w[6,17] - 2 v[6,17] <= 0
 MaxNumberSupportOnArc[6,18]: w[6,18] - 2 v[6,18] <= 0
 MaxNumberSupportOnArc[6,19]: w[6,19] - 2 v[6,19] <= 0
 MaxNumberSupportOnArc[6,20]: w[6,20] - 2 v[6,20] <= 0
 MaxNumberSupportOnArc[6,21]: w[6,21] - 2 v[6,21] <= 0
 MaxNumberSupportOnArc[6,22]: w[6,22] - 2 v[6,22] <= 0
 MaxNumberSupportOnArc[6,23]: w[6,23] - 2 v[6,23] <= 0
 MaxNumberSupportOnArc[7,1]: w[7,1] - 2 v[7,1] <= 0
 MaxNumberSupportOnArc[7,2]: w[7,2] - 2 v[7,2] <= 0
 MaxNumberSupportOnArc[7,3]: w[7,3] - 2 v[7,3] <= 0
 MaxNumberSupportOnArc[7,4]: w[7,4] - 2 v[7,4] <= 0
 MaxNumberSupportOnArc[7,5]: w[7,5] - 2 v[7,5] <= 0
 MaxNumberSupportOnArc[7,6]: w[7,6] - 2 v[7,6] <= 0
 MaxNumberSupportOnArc[7,7]: w[7,7] - 2 v[7,7] <= 0
 MaxNumberSupportOnArc[7,8]: w[7,8] - 2 v[7,8] <= 0
 MaxNumberSupportOnArc[7,9]: w[7,9] - 2 v[7,9] <= 0
 MaxNumberSupportOnArc[7,10]: w[7,10] - 2 v[7,10] <= 0
 MaxNumberSupportOnArc[7,11]: w[7,11] - 2 v[7,11] <= 0
 MaxNumberSupportOnArc[7,12]: w[7,12] - 2 v[7,12] <= 0
 MaxNumberSupportOnArc[7,13]: w[7,13] - 2 v[7,13] <= 0
 MaxNumberSupportOnArc[7,14]: w[7,14] - 2 v[7,14] <= 0
 MaxNumberSupportOnArc[7,15]: w[7,15] - 2 v[7,15] <= 0
 MaxNumberSupportOnArc[7,16]: w[7,16] - 2 v[7,16] <= 0
 MaxNumberSupportOnArc[7,17]: w[7,17] - 2 v[7,17] <= 0
 MaxNumberSupportOnArc[7,18]: w[7,18] - 2 v[7,18] <= 0
 MaxNumberSupportOnArc[7,19]: w[7,19] - 2 v[7,19] <= 0
 MaxNumberSupportOnArc[7,20]: w[7,20] - 2 v[7,20] <= 0
 MaxNumberSupportOnArc[7,21]: w[7,21] - 2 v[7,21] <= 0
 MaxNumberSupportOnArc[7,22]: w[7,22] - 2 v[7,22] <= 0
 MaxNumberSupportOnArc[7,23]: w[7,23] - 2 v[7,23] <= 0
 MaxNumberSupportOnArc[8,1]: w[8,1] - 2 v[8,1] <= 0
 MaxNumberSupportOnArc[8,2]: w[8,2] - 2 v[8,2] <= 0
 MaxNumberSupportOnArc[8,3]: w[8,3] - 2 v[8,3] <= 0
 MaxNumberSupportOnArc[8,4]: w[8,4] - 2 v[8,4] <= 0
 MaxNumberSupportOnArc[8,5]: w[8,5] - 2 v[8,5] <= 0
 MaxNumberSupportOnArc[8,6]: w[8,6] - 2 v[8,6] <= 0
 MaxNumberSupportOnArc[8,7]: w[8,7] - 2 v[8,7] <= 0
 MaxNumberSupportOnArc[8,8]: w[8,8] - 2 v[8,8] <= 0
 MaxNumberSupportOnArc[8,9]: w[8,9] - 2 v[8,9] <= 0
 MaxNumberSupportOnArc[8,10]: w[8,10] - 2 v[8,10] <= 0
 MaxNumberSupportOnArc[8,11]: w[8,11] - 2 v[8,11] <= 0
 MaxNumberSupportOnArc[8,12]: w[8,12] - 2 v[8,12] <= 0
 MaxNumberSupportOnArc[8,13]: w[8,13] - 2 v[8,13] <= 0
 MaxNumberSupportOnArc[8,14]: w[8,14] - 2 v[8,14] <= 0
 MaxNumberSupportOnArc[8,15]: w[8,15] - 2 v[8,15] <= 0
 MaxNumberSupportOnArc[8,16]: w[8,16] - 2 v[8,16] <= 0
 MaxNumberSupportOnArc[8,17]: w[8,17] - 2 v[8,17] <= 0
 MaxNumberSupportOnArc[8,18]: w[8,18] - 2 v[8,18] <= 0
 MaxNumberSupportOnArc[8,19]: w[8,19] - 2 v[8,19] <= 0
 MaxNumberSupportOnArc[8,20]: w[8,20] - 2 v[8,20] <= 0
 MaxNumberSupportOnArc[8,21]: w[8,21] - 2 v[8,21] <= 0
 MaxNumberSupportOnArc[8,22]: w[8,22] - 2 v[8,22] <= 0
 MaxNumberSupportOnArc[8,23]: w[8,23] - 2 v[8,23] <= 0
 MaxNumberSupportOnArc[9,1]: w[9,1] - 2 v[9,1] <= 0
 MaxNumberSupportOnArc[9,2]: w[9,2] - 2 v[9,2] <= 0
 MaxNumberSupportOnArc[9,3]: w[9,3] - 2 v[9,3] <= 0
 MaxNumberSupportOnArc[9,4]: w[9,4] - 2 v[9,4] <= 0
 MaxNumberSupportOnArc[9,5]: w[9,5] - 2 v[9,5] <= 0
 MaxNumberSupportOnArc[9,6]: w[9,6] - 2 v[9,6] <= 0
 MaxNumberSupportOnArc[9,7]: w[9,7] - 2 v[9,7] <= 0
 MaxNumberSupportOnArc[9,8]: w[9,8] - 2 v[9,8] <= 0
 MaxNumberSupportOnArc[9,9]: w[9,9] - 2 v[9,9] <= 0
 MaxNumberSupportOnArc[9,10]: w[9,10] - 2 v[9,10] <= 0
 MaxNumberSupportOnArc[9,11]: w[9,11] - 2 v[9,11] <= 0
 MaxNumberSupportOnArc[9,12]: w[9,12] - 2 v[9,12] <= 0
 MaxNumberSupportOnArc[9,13]: w[9,13] - 2 v[9,13] <= 0
 MaxNumberSupportOnArc[9,14]: w[9,14] - 2 v[9,14] <= 0
 MaxNumberSupportOnArc[9,15]: w[9,15] - 2 v[9,15] <= 0
 MaxNumberSupportOnArc[9,16]: w[9,16] - 2 v[9,16] <= 0
 MaxNumberSupportOnArc[9,17]: w[9,17] - 2 v[9,17] <= 0
 MaxNumberSupportOnArc[9,18]: w[9,18] - 2 v[9,18] <= 0
 MaxNumberSupportOnArc[9,19]: w[9,19] - 2 v[9,19] <= 0
 MaxNumberSupportOnArc[9,20]: w[9,20] - 2 v[9,20] <= 0
 MaxNumberSupportOnArc[9,21]: w[9,21] - 2 v[9,21] <= 0
 MaxNumberSupportOnArc[9,22]: w[9,22] - 2 v[9,22] <= 0
 MaxNumberSupportOnArc[9,23]: w[9,23] - 2 v[9,23] <= 0
 MaxNumberSupportOnArc[10,1]: w[10,1] - 2 v[10,1] <= 0
 MaxNumberSupportOnArc[10,2]: w[10,2] - 2 v[10,2] <= 0
 MaxNumberSupportOnArc[10,3]: w[10,3] - 2 v[10,3] <= 0
 MaxNumberSupportOnArc[10,4]: w[10,4] - 2 v[10,4] <= 0
 MaxNumberSupportOnArc[10,5]: w[10,5] - 2 v[10,5] <= 0
 MaxNumberSupportOnArc[10,6]: w[10,6] - 2 v[10,6] <= 0
 MaxNumberSupportOnArc[10,7]: w[10,7] - 2 v[10,7] <= 0
 MaxNumberSupportOnArc[10,8]: w[10,8] - 2 v[10,8] <= 0
 MaxNumberSupportOnArc[10,9]: w[10,9] - 2 v[10,9] <= 0
 MaxNumberSupportOnArc[10,10]: w[10,10] - 2 v[10,10] <= 0
 MaxNumberSupportOnArc[10,11]: w[10,11] - 2 v[10,11] <= 0
 MaxNumberSupportOnArc[10,12]: w[10,12] - 2 v[10,12] <= 0
 MaxNumberSupportOnArc[10,13]: w[10,13] - 2 v[10,13] <= 0
 MaxNumberSupportOnArc[10,14]: w[10,14] - 2 v[10,14] <= 0
 MaxNumberSupportOnArc[10,15]: w[10,15] - 2 v[10,15] <= 0
 MaxNumberSupportOnArc[10,16]: w[10,16] - 2 v[10,16] <= 0
 MaxNumberSupportOnArc[10,17]: w[10,17] - 2 v[10,17] <= 0
 MaxNumberSupportOnArc[10,18]: w[10,18] - 2 v[10,18] <= 0
 MaxNumberSupportOnArc[10,19]: w[10,19] - 2 v[10,19] <= 0
 MaxNumberSupportOnArc[10,20]: w[10,20] - 2 v[10,20] <= 0
 MaxNumberSupportOnArc[10,21]: w[10,21] - 2 v[10,21] <= 0
 MaxNumberSupportOnArc[10,22]: w[10,22] - 2 v[10,22] <= 0
 MaxNumberSupportOnArc[10,23]: w[10,23] - 2 v[10,23] <= 0
 MaxNumberSupportOnArc[11,1]: w[11,1] - 2 v[11,1] <= 0
 MaxNumberSupportOnArc[11,2]: w[11,2] - 2 v[11,2] <= 0
 MaxNumberSupportOnArc[11,3]: w[11,3] - 2 v[11,3] <= 0
 MaxNumberSupportOnArc[11,4]: w[11,4] - 2 v[11,4] <= 0
 MaxNumberSupportOnArc[11,5]: w[11,5] - 2 v[11,5] <= 0
 MaxNumberSupportOnArc[11,6]: w[11,6] - 2 v[11,6] <= 0
 MaxNumberSupportOnArc[11,7]: w[11,7] - 2 v[11,7] <= 0
 MaxNumberSupportOnArc[11,8]: w[11,8] - 2 v[11,8] <= 0
 MaxNumberSupportOnArc[11,9]: w[11,9] - 2 v[11,9] <= 0
 MaxNumberSupportOnArc[11,10]: w[11,10] - 2 v[11,10] <= 0
 MaxNumberSupportOnArc[11,11]: w[11,11] - 2 v[11,11] <= 0
 MaxNumberSupportOnArc[11,12]: w[11,12] - 2 v[11,12] <= 0
 MaxNumberSupportOnArc[11,13]: w[11,13] - 2 v[11,13] <= 0
 MaxNumberSupportOnArc[11,14]: w[11,14] - 2 v[11,14] <= 0
 MaxNumberSupportOnArc[11,15]: w[11,15] - 2 v[11,15] <= 0
 MaxNumberSupportOnArc[11,16]: w[11,16] - 2 v[11,16] <= 0
 MaxNumberSupportOnArc[11,17]: w[11,17] - 2 v[11,17] <= 0
 MaxNumberSupportOnArc[11,18]: w[11,18] - 2 v[11,18] <= 0
 MaxNumberSupportOnArc[11,19]: w[11,19] - 2 v[11,19] <= 0
 MaxNumberSupportOnArc[11,20]: w[11,20] - 2 v[11,20] <= 0
 MaxNumberSupportOnArc[11,21]: w[11,21] - 2 v[11,21] <= 0
 MaxNumberSupportOnArc[11,22]: w[11,22] - 2 v[11,22] <= 0
 MaxNumberSupportOnArc[11,23]: w[11,23] - 2 v[11,23] <= 0
 MaxNumberSupportOnArc[12,1]: w[12,1] - 2 v[12,1] <= 0
 MaxNumberSupportOnArc[12,2]: w[12,2] - 2 v[12,2] <= 0
 MaxNumberSupportOnArc[12,3]: w[12,3] - 2 v[12,3] <= 0
 MaxNumberSupportOnArc[12,4]: w[12,4] - 2 v[12,4] <= 0
 MaxNumberSupportOnArc[12,5]: w[12,5] - 2 v[12,5] <= 0
 MaxNumberSupportOnArc[12,6]: w[12,6] - 2 v[12,6] <= 0
 MaxNumberSupportOnArc[12,7]: w[12,7] - 2 v[12,7] <= 0
 MaxNumberSupportOnArc[12,8]: w[12,8] - 2 v[12,8] <= 0
 MaxNumberSupportOnArc[12,9]: w[12,9] - 2 v[12,9] <= 0
 MaxNumberSupportOnArc[12,10]: w[12,10] - 2 v[12,10] <= 0
 MaxNumberSupportOnArc[12,11]: w[12,11] - 2 v[12,11] <= 0
 MaxNumberSupportOnArc[12,12]: w[12,12] - 2 v[12,12] <= 0
 MaxNumberSupportOnArc[12,13]: w[12,13] - 2 v[12,13] <= 0
 MaxNumberSupportOnArc[12,14]: w[12,14] - 2 v[12,14] <= 0
 MaxNumberSupportOnArc[12,15]: w[12,15] - 2 v[12,15] <= 0
 MaxNumberSupportOnArc[12,16]: w[12,16] - 2 v[12,16] <= 0
 MaxNumberSupportOnArc[12,17]: w[12,17] - 2 v[12,17] <= 0
 MaxNumberSupportOnArc[12,18]: w[12,18] - 2 v[12,18] <= 0
 MaxNumberSupportOnArc[12,19]: w[12,19] - 2 v[12,19] <= 0
 MaxNumberSupportOnArc[12,20]: w[12,20] - 2 v[12,20] <= 0
 MaxNumberSupportOnArc[12,21]: w[12,21] - 2 v[12,21] <= 0
 MaxNumberSupportOnArc[12,22]: w[12,22] - 2 v[12,22] <= 0
 MaxNumberSupportOnArc[12,23]: w[12,23] - 2 v[12,23] <= 0
 MaxNumberSupportOnArc[13,1]: w[13,1] - 2 v[13,1] <= 0
 MaxNumberSupportOnArc[13,2]: w[13,2] - 2 v[13,2] <= 0
 MaxNumberSupportOnArc[13,3]: w[13,3] - 2 v[13,3] <= 0
 MaxNumberSupportOnArc[13,4]: w[13,4] - 2 v[13,4] <= 0
 MaxNumberSupportOnArc[13,5]: w[13,5] - 2 v[13,5] <= 0
 MaxNumberSupportOnArc[13,6]: w[13,6] - 2 v[13,6] <= 0
 MaxNumberSupportOnArc[13,7]: w[13,7] - 2 v[13,7] <= 0
 MaxNumberSupportOnArc[13,8]: w[13,8] - 2 v[13,8] <= 0
 MaxNumberSupportOnArc[13,9]: w[13,9] - 2 v[13,9] <= 0
 MaxNumberSupportOnArc[13,10]: w[13,10] - 2 v[13,10] <= 0
 MaxNumberSupportOnArc[13,11]: w[13,11] - 2 v[13,11] <= 0
 MaxNumberSupportOnArc[13,12]: w[13,12] - 2 v[13,12] <= 0
 MaxNumberSupportOnArc[13,13]: w[13,13] - 2 v[13,13] <= 0
 MaxNumberSupportOnArc[13,14]: w[13,14] - 2 v[13,14] <= 0
 MaxNumberSupportOnArc[13,15]: w[13,15] - 2 v[13,15] <= 0
 MaxNumberSupportOnArc[13,16]: w[13,16] - 2 v[13,16] <= 0
 MaxNumberSupportOnArc[13,17]: w[13,17] - 2 v[13,17] <= 0
 MaxNumberSupportOnArc[13,18]: w[13,18] - 2 v[13,18] <= 0
 MaxNumberSupportOnArc[13,19]: w[13,19] - 2 v[13,19] <= 0
 MaxNumberSupportOnArc[13,20]: w[13,20] - 2 v[13,20] <= 0
 MaxNumberSupportOnArc[13,21]: w[13,21] - 2 v[13,21] <= 0
 MaxNumberSupportOnArc[13,22]: w[13,22] - 2 v[13,22] <= 0
 MaxNumberSupportOnArc[13,23]: w[13,23] - 2 v[13,23] <= 0
 MaxNumberSupportOnArc[14,1]: w[14,1] - 2 v[14,1] <= 0
 MaxNumberSupportOnArc[14,2]: w[14,2] - 2 v[14,2] <= 0
 MaxNumberSupportOnArc[14,3]: w[14,3] - 2 v[14,3] <= 0
 MaxNumberSupportOnArc[14,4]: w[14,4] - 2 v[14,4] <= 0
 MaxNumberSupportOnArc[14,5]: w[14,5] - 2 v[14,5] <= 0
 MaxNumberSupportOnArc[14,6]: w[14,6] - 2 v[14,6] <= 0
 MaxNumberSupportOnArc[14,7]: w[14,7] - 2 v[14,7] <= 0
 MaxNumberSupportOnArc[14,8]: w[14,8] - 2 v[14,8] <= 0
 MaxNumberSupportOnArc[14,9]: w[14,9] - 2 v[14,9] <= 0
 MaxNumberSupportOnArc[14,10]: w[14,10] - 2 v[14,10] <= 0
 MaxNumberSupportOnArc[14,11]: w[14,11] - 2 v[14,11] <= 0
 MaxNumberSupportOnArc[14,12]: w[14,12] - 2 v[14,12] <= 0
 MaxNumberSupportOnArc[14,13]: w[14,13] - 2 v[14,13] <= 0
 MaxNumberSupportOnArc[14,14]: w[14,14] - 2 v[14,14] <= 0
 MaxNumberSupportOnArc[14,15]: w[14,15] - 2 v[14,15] <= 0
 MaxNumberSupportOnArc[14,16]: w[14,16] - 2 v[14,16] <= 0
 MaxNumberSupportOnArc[14,17]: w[14,17] - 2 v[14,17] <= 0
 MaxNumberSupportOnArc[14,18]: w[14,18] - 2 v[14,18] <= 0
 MaxNumberSupportOnArc[14,19]: w[14,19] - 2 v[14,19] <= 0
 MaxNumberSupportOnArc[14,20]: w[14,20] - 2 v[14,20] <= 0
 MaxNumberSupportOnArc[14,21]: w[14,21] - 2 v[14,21] <= 0
 MaxNumberSupportOnArc[14,22]: w[14,22] - 2 v[14,22] <= 0
 MaxNumberSupportOnArc[14,23]: w[14,23] - 2 v[14,23] <= 0
 MaxNumberSupportOnArc[15,1]: w[15,1] - 2 v[15,1] <= 0
 MaxNumberSupportOnArc[15,2]: w[15,2] - 2 v[15,2] <= 0
 MaxNumberSupportOnArc[15,3]: w[15,3] - 2 v[15,3] <= 0
 MaxNumberSupportOnArc[15,4]: w[15,4] - 2 v[15,4] <= 0
 MaxNumberSupportOnArc[15,5]: w[15,5] - 2 v[15,5] <= 0
 MaxNumberSupportOnArc[15,6]: w[15,6] - 2 v[15,6] <= 0
 MaxNumberSupportOnArc[15,7]: w[15,7] - 2 v[15,7] <= 0
 MaxNumberSupportOnArc[15,8]: w[15,8] - 2 v[15,8] <= 0
 MaxNumberSupportOnArc[15,9]: w[15,9] - 2 v[15,9] <= 0
 MaxNumberSupportOnArc[15,10]: w[15,10] - 2 v[15,10] <= 0
 MaxNumberSupportOnArc[15,11]: w[15,11] - 2 v[15,11] <= 0
 MaxNumberSupportOnArc[15,12]: w[15,12] - 2 v[15,12] <= 0
 MaxNumberSupportOnArc[15,13]: w[15,13] - 2 v[15,13] <= 0
 MaxNumberSupportOnArc[15,14]: w[15,14] - 2 v[15,14] <= 0
 MaxNumberSupportOnArc[15,15]: w[15,15] - 2 v[15,15] <= 0
 MaxNumberSupportOnArc[15,16]: w[15,16] - 2 v[15,16] <= 0
 MaxNumberSupportOnArc[15,17]: w[15,17] - 2 v[15,17] <= 0
 MaxNumberSupportOnArc[15,18]: w[15,18] - 2 v[15,18] <= 0
 MaxNumberSupportOnArc[15,19]: w[15,19] - 2 v[15,19] <= 0
 MaxNumberSupportOnArc[15,20]: w[15,20] - 2 v[15,20] <= 0
 MaxNumberSupportOnArc[15,21]: w[15,21] - 2 v[15,21] <= 0
 MaxNumberSupportOnArc[15,22]: w[15,22] - 2 v[15,22] <= 0
 MaxNumberSupportOnArc[15,23]: w[15,23] - 2 v[15,23] <= 0
 MaxNumberSupportOnArc[16,1]: w[16,1] - 2 v[16,1] <= 0
 MaxNumberSupportOnArc[16,2]: w[16,2] - 2 v[16,2] <= 0
 MaxNumberSupportOnArc[16,3]: w[16,3] - 2 v[16,3] <= 0
 MaxNumberSupportOnArc[16,4]: w[16,4] - 2 v[16,4] <= 0
 MaxNumberSupportOnArc[16,5]: w[16,5] - 2 v[16,5] <= 0
 MaxNumberSupportOnArc[16,6]: w[16,6] - 2 v[16,6] <= 0
 MaxNumberSupportOnArc[16,7]: w[16,7] - 2 v[16,7] <= 0
 MaxNumberSupportOnArc[16,8]: w[16,8] - 2 v[16,8] <= 0
 MaxNumberSupportOnArc[16,9]: w[16,9] - 2 v[16,9] <= 0
 MaxNumberSupportOnArc[16,10]: w[16,10] - 2 v[16,10] <= 0
 MaxNumberSupportOnArc[16,11]: w[16,11] - 2 v[16,11] <= 0
 MaxNumberSupportOnArc[16,12]: w[16,12] - 2 v[16,12] <= 0
 MaxNumberSupportOnArc[16,13]: w[16,13] - 2 v[16,13] <= 0
 MaxNumberSupportOnArc[16,14]: w[16,14] - 2 v[16,14] <= 0
 MaxNumberSupportOnArc[16,15]: w[16,15] - 2 v[16,15] <= 0
 MaxNumberSupportOnArc[16,16]: w[16,16] - 2 v[16,16] <= 0
 MaxNumberSupportOnArc[16,17]: w[16,17] - 2 v[16,17] <= 0
 MaxNumberSupportOnArc[16,18]: w[16,18] - 2 v[16,18] <= 0
 MaxNumberSupportOnArc[16,19]: w[16,19] - 2 v[16,19] <= 0
 MaxNumberSupportOnArc[16,20]: w[16,20] - 2 v[16,20] <= 0
 MaxNumberSupportOnArc[16,21]: w[16,21] - 2 v[16,21] <= 0
 MaxNumberSupportOnArc[16,22]: w[16,22] - 2 v[16,22] <= 0
 MaxNumberSupportOnArc[16,23]: w[16,23] - 2 v[16,23] <= 0
 MaxNumberSupportOnArc[17,1]: w[17,1] - 2 v[17,1] <= 0
 MaxNumberSupportOnArc[17,2]: w[17,2] - 2 v[17,2] <= 0
 MaxNumberSupportOnArc[17,3]: w[17,3] - 2 v[17,3] <= 0
 MaxNumberSupportOnArc[17,4]: w[17,4] - 2 v[17,4] <= 0
 MaxNumberSupportOnArc[17,5]: w[17,5] - 2 v[17,5] <= 0
 MaxNumberSupportOnArc[17,6]: w[17,6] - 2 v[17,6] <= 0
 MaxNumberSupportOnArc[17,7]: w[17,7] - 2 v[17,7] <= 0
 MaxNumberSupportOnArc[17,8]: w[17,8] - 2 v[17,8] <= 0
 MaxNumberSupportOnArc[17,9]: w[17,9] - 2 v[17,9] <= 0
 MaxNumberSupportOnArc[17,10]: w[17,10] - 2 v[17,10] <= 0
 MaxNumberSupportOnArc[17,11]: w[17,11] - 2 v[17,11] <= 0
 MaxNumberSupportOnArc[17,12]: w[17,12] - 2 v[17,12] <= 0
 MaxNumberSupportOnArc[17,13]: w[17,13] - 2 v[17,13] <= 0
 MaxNumberSupportOnArc[17,14]: w[17,14] - 2 v[17,14] <= 0
 MaxNumberSupportOnArc[17,15]: w[17,15] - 2 v[17,15] <= 0
 MaxNumberSupportOnArc[17,16]: w[17,16] - 2 v[17,16] <= 0
 MaxNumberSupportOnArc[17,17]: w[17,17] - 2 v[17,17] <= 0
 MaxNumberSupportOnArc[17,18]: w[17,18] - 2 v[17,18] <= 0
 MaxNumberSupportOnArc[17,19]: w[17,19] - 2 v[17,19] <= 0
 MaxNumberSupportOnArc[17,20]: w[17,20] - 2 v[17,20] <= 0
 MaxNumberSupportOnArc[17,21]: w[17,21] - 2 v[17,21] <= 0
 MaxNumberSupportOnArc[17,22]: w[17,22] - 2 v[17,22] <= 0
 MaxNumberSupportOnArc[17,23]: w[17,23] - 2 v[17,23] <= 0
 MaxNumberSupportOnArc[18,1]: w[18,1] - 2 v[18,1] <= 0
 MaxNumberSupportOnArc[18,2]: w[18,2] - 2 v[18,2] <= 0
 MaxNumberSupportOnArc[18,3]: w[18,3] - 2 v[18,3] <= 0
 MaxNumberSupportOnArc[18,4]: w[18,4] - 2 v[18,4] <= 0
 MaxNumberSupportOnArc[18,5]: w[18,5] - 2 v[18,5] <= 0
 MaxNumberSupportOnArc[18,6]: w[18,6] - 2 v[18,6] <= 0
 MaxNumberSupportOnArc[18,7]: w[18,7] - 2 v[18,7] <= 0
 MaxNumberSupportOnArc[18,8]: w[18,8] - 2 v[18,8] <= 0
 MaxNumberSupportOnArc[18,9]: w[18,9] - 2 v[18,9] <= 0
 MaxNumberSupportOnArc[18,10]: w[18,10] - 2 v[18,10] <= 0
 MaxNumberSupportOnArc[18,11]: w[18,11] - 2 v[18,11] <= 0
 MaxNumberSupportOnArc[18,12]: w[18,12] - 2 v[18,12] <= 0
 MaxNumberSupportOnArc[18,13]: w[18,13] - 2 v[18,13] <= 0
 MaxNumberSupportOnArc[18,14]: w[18,14] - 2 v[18,14] <= 0
 MaxNumberSupportOnArc[18,15]: w[18,15] - 2 v[18,15] <= 0
 MaxNumberSupportOnArc[18,16]: w[18,16] - 2 v[18,16] <= 0
 MaxNumberSupportOnArc[18,17]: w[18,17] - 2 v[18,17] <= 0
 MaxNumberSupportOnArc[18,18]: w[18,18] - 2 v[18,18] <= 0
 MaxNumberSupportOnArc[18,19]: w[18,19] - 2 v[18,19] <= 0
 MaxNumberSupportOnArc[18,20]: w[18,20] - 2 v[18,20] <= 0
 MaxNumberSupportOnArc[18,21]: w[18,21] - 2 v[18,21] <= 0
 MaxNumberSupportOnArc[18,22]: w[18,22] - 2 v[18,22] <= 0
 MaxNumberSupportOnArc[18,23]: w[18,23] - 2 v[18,23] <= 0
 MaxNumberSupportOnArc[19,1]: w[19,1] - 2 v[19,1] <= 0
 MaxNumberSupportOnArc[19,2]: w[19,2] - 2 v[19,2] <= 0
 MaxNumberSupportOnArc[19,3]: w[19,3] - 2 v[19,3] <= 0
 MaxNumberSupportOnArc[19,4]: w[19,4] - 2 v[19,4] <= 0
 MaxNumberSupportOnArc[19,5]: w[19,5] - 2 v[19,5] <= 0
 MaxNumberSupportOnArc[19,6]: w[19,6] - 2 v[19,6] <= 0
 MaxNumberSupportOnArc[19,7]: w[19,7] - 2 v[19,7] <= 0
 MaxNumberSupportOnArc[19,8]: w[19,8] - 2 v[19,8] <= 0
 MaxNumberSupportOnArc[19,9]: w[19,9] - 2 v[19,9] <= 0
 MaxNumberSupportOnArc[19,10]: w[19,10] - 2 v[19,10] <= 0
 MaxNumberSupportOnArc[19,11]: w[19,11] - 2 v[19,11] <= 0
 MaxNumberSupportOnArc[19,12]: w[19,12] - 2 v[19,12] <= 0
 MaxNumberSupportOnArc[19,13]: w[19,13] - 2 v[19,13] <= 0
 MaxNumberSupportOnArc[19,14]: w[19,14] - 2 v[19,14] <= 0
 MaxNumberSupportOnArc[19,15]: w[19,15] - 2 v[19,15] <= 0
 MaxNumberSupportOnArc[19,16]: w[19,16] - 2 v[19,16] <= 0
 MaxNumberSupportOnArc[19,17]: w[19,17] - 2 v[19,17] <= 0
 MaxNumberSupportOnArc[19,18]: w[19,18] - 2 v[19,18] <= 0
 MaxNumberSupportOnArc[19,19]: w[19,19] - 2 v[19,19] <= 0
 MaxNumberSupportOnArc[19,20]: w[19,20] - 2 v[19,20] <= 0
 MaxNumberSupportOnArc[19,21]: w[19,21] - 2 v[19,21] <= 0
 MaxNumberSupportOnArc[19,22]: w[19,22] - 2 v[19,22] <= 0
 MaxNumberSupportOnArc[19,23]: w[19,23] - 2 v[19,23] <= 0
 MaxNumberSupportOnArc[20,1]: w[20,1] - 2 v[20,1] <= 0
 MaxNumberSupportOnArc[20,2]: w[20,2] - 2 v[20,2] <= 0
 MaxNumberSupportOnArc[20,3]: w[20,3] - 2 v[20,3] <= 0
 MaxNumberSupportOnArc[20,4]: w[20,4] - 2 v[20,4] <= 0
 MaxNumberSupportOnArc[20,5]: w[20,5] - 2 v[20,5] <= 0
 MaxNumberSupportOnArc[20,6]: w[20,6] - 2 v[20,6] <= 0
 MaxNumberSupportOnArc[20,7]: w[20,7] - 2 v[20,7] <= 0
 MaxNumberSupportOnArc[20,8]: w[20,8] - 2 v[20,8] <= 0
 MaxNumberSupportOnArc[20,9]: w[20,9] - 2 v[20,9] <= 0
 MaxNumberSupportOnArc[20,10]: w[20,10] - 2 v[20,10] <= 0
 MaxNumberSupportOnArc[20,11]: w[20,11] - 2 v[20,11] <= 0
 MaxNumberSupportOnArc[20,12]: w[20,12] - 2 v[20,12] <= 0
 MaxNumberSupportOnArc[20,13]: w[20,13] - 2 v[20,13] <= 0
 MaxNumberSupportOnArc[20,14]: w[20,14] - 2 v[20,14] <= 0
 MaxNumberSupportOnArc[20,15]: w[20,15] - 2 v[20,15] <= 0
 MaxNumberSupportOnArc[20,16]: w[20,16] - 2 v[20,16] <= 0
 MaxNumberSupportOnArc[20,17]: w[20,17] - 2 v[20,17] <= 0
 MaxNumberSupportOnArc[20,18]: w[20,18] - 2 v[20,18] <= 0
 MaxNumberSupportOnArc[20,19]: w[20,19] - 2 v[20,19] <= 0
 MaxNumberSupportOnArc[20,20]: w[20,20] - 2 v[20,20] <= 0
 MaxNumberSupportOnArc[20,21]: w[20,21] - 2 v[20,21] <= 0
 MaxNumberSupportOnArc[20,22]: w[20,22] - 2 v[20,22] <= 0
 MaxNumberSupportOnArc[20,23]: w[20,23] - 2 v[20,23] <= 0
 MaxNumberSupportOnArc[21,1]: w[21,1] - 2 v[21,1] <= 0
 MaxNumberSupportOnArc[21,2]: w[21,2] - 2 v[21,2] <= 0
 MaxNumberSupportOnArc[21,3]: w[21,3] - 2 v[21,3] <= 0
 MaxNumberSupportOnArc[21,4]: w[21,4] - 2 v[21,4] <= 0
 MaxNumberSupportOnArc[21,5]: w[21,5] - 2 v[21,5] <= 0
 MaxNumberSupportOnArc[21,6]: w[21,6] - 2 v[21,6] <= 0
 MaxNumberSupportOnArc[21,7]: w[21,7] - 2 v[21,7] <= 0
 MaxNumberSupportOnArc[21,8]: w[21,8] - 2 v[21,8] <= 0
 MaxNumberSupportOnArc[21,9]: w[21,9] - 2 v[21,9] <= 0
 MaxNumberSupportOnArc[21,10]: w[21,10] - 2 v[21,10] <= 0
 MaxNumberSupportOnArc[21,11]: w[21,11] - 2 v[21,11] <= 0
 MaxNumberSupportOnArc[21,12]: w[21,12] - 2 v[21,12] <= 0
 MaxNumberSupportOnArc[21,13]: w[21,13] - 2 v[21,13] <= 0
 MaxNumberSupportOnArc[21,14]: w[21,14] - 2 v[21,14] <= 0
 MaxNumberSupportOnArc[21,15]: w[21,15] - 2 v[21,15] <= 0
 MaxNumberSupportOnArc[21,16]: w[21,16] - 2 v[21,16] <= 0
 MaxNumberSupportOnArc[21,17]: w[21,17] - 2 v[21,17] <= 0
 MaxNumberSupportOnArc[21,18]: w[21,18] - 2 v[21,18] <= 0
 MaxNumberSupportOnArc[21,19]: w[21,19] - 2 v[21,19] <= 0
 MaxNumberSupportOnArc[21,20]: w[21,20] - 2 v[21,20] <= 0
 MaxNumberSupportOnArc[21,21]: w[21,21] - 2 v[21,21] <= 0
 MaxNumberSupportOnArc[21,22]: w[21,22] - 2 v[21,22] <= 0
 MaxNumberSupportOnArc[21,23]: w[21,23] - 2 v[21,23] <= 0
 MaxNumberSupportOnArc[22,1]: w[22,1] - 2 v[22,1] <= 0
 MaxNumberSupportOnArc[22,2]: w[22,2] - 2 v[22,2] <= 0
 MaxNumberSupportOnArc[22,3]: w[22,3] - 2 v[22,3] <= 0
 MaxNumberSupportOnArc[22,4]: w[22,4] - 2 v[22,4] <= 0
 MaxNumberSupportOnArc[22,5]: w[22,5] - 2 v[22,5] <= 0
 MaxNumberSupportOnArc[22,6]: w[22,6] - 2 v[22,6] <= 0
 MaxNumberSupportOnArc[22,7]: w[22,7] - 2 v[22,7] <= 0
 MaxNumberSupportOnArc[22,8]: w[22,8] - 2 v[22,8] <= 0
 MaxNumberSupportOnArc[22,9]: w[22,9] - 2 v[22,9] <= 0
 MaxNumberSupportOnArc[22,10]: w[22,10] - 2 v[22,10] <= 0
 MaxNumberSupportOnArc[22,11]: w[22,11] - 2 v[22,11] <= 0
 MaxNumberSupportOnArc[22,12]: w[22,12] - 2 v[22,12] <= 0
 MaxNumberSupportOnArc[22,13]: w[22,13] - 2 v[22,13] <= 0
 MaxNumberSupportOnArc[22,14]: w[22,14] - 2 v[22,14] <= 0
 MaxNumberSupportOnArc[22,15]: w[22,15] - 2 v[22,15] <= 0
 MaxNumberSupportOnArc[22,16]: w[22,16] - 2 v[22,16] <= 0
 MaxNumberSupportOnArc[22,17]: w[22,17] - 2 v[22,17] <= 0
 MaxNumberSupportOnArc[22,18]: w[22,18] - 2 v[22,18] <= 0
 MaxNumberSupportOnArc[22,19]: w[22,19] - 2 v[22,19] <= 0
 MaxNumberSupportOnArc[22,20]: w[22,20] - 2 v[22,20] <= 0
 MaxNumberSupportOnArc[22,21]: w[22,21] - 2 v[22,21] <= 0
 MaxNumberSupportOnArc[22,22]: w[22,22] - 2 v[22,22] <= 0
 MaxNumberSupportOnArc[22,23]: w[22,23] - 2 v[22,23] <= 0
 ArcUsedByRobot[0,1]: w[0,1] - v[0,1] >= 0
 ArcUsedByRobot[0,2]: w[0,2] - v[0,2] >= 0
 ArcUsedByRobot[0,3]: w[0,3] - v[0,3] >= 0
 ArcUsedByRobot[0,4]: w[0,4] - v[0,4] >= 0
 ArcUsedByRobot[0,5]: w[0,5] - v[0,5] >= 0
 ArcUsedByRobot[0,6]: w[0,6] - v[0,6] >= 0
 ArcUsedByRobot[0,7]: w[0,7] - v[0,7] >= 0
 ArcUsedByRobot[0,8]: w[0,8] - v[0,8] >= 0
 ArcUsedByRobot[0,9]: w[0,9] - v[0,9] >= 0
 ArcUsedByRobot[0,10]: w[0,10] - v[0,10] >= 0
 ArcUsedByRobot[0,11]: w[0,11] - v[0,11] >= 0
 ArcUsedByRobot[0,12]: w[0,12] - v[0,12] >= 0
 ArcUsedByRobot[0,13]: w[0,13] - v[0,13] >= 0
 ArcUsedByRobot[0,14]: w[0,14] - v[0,14] >= 0
 ArcUsedByRobot[0,15]: w[0,15] - v[0,15] >= 0
 ArcUsedByRobot[0,16]: w[0,16] - v[0,16] >= 0
 ArcUsedByRobot[0,17]: w[0,17] - v[0,17] >= 0
 ArcUsedByRobot[0,18]: w[0,18] - v[0,18] >= 0
 ArcUsedByRobot[0,19]: w[0,19] - v[0,19] >= 0
 ArcUsedByRobot[0,20]: w[0,20] - v[0,20] >= 0
 ArcUsedByRobot[0,21]: w[0,21] - v[0,21] >= 0
 ArcUsedByRobot[0,22]: w[0,22] - v[0,22] >= 0
 ArcUsedByRobot[0,23]: w[0,23] - v[0,23] >= 0
 ArcUsedByRobot[1,1]: w[1,1] - v[1,1] >= 0
 ArcUsedByRobot[1,2]: w[1,2] - v[1,2] >= 0
 ArcUsedByRobot[1,3]: w[1,3] - v[1,3] >= 0
 ArcUsedByRobot[1,4]: w[1,4] - v[1,4] >= 0
 ArcUsedByRobot[1,5]: w[1,5] - v[1,5] >= 0
 ArcUsedByRobot[1,6]: w[1,6] - v[1,6] >= 0
 ArcUsedByRobot[1,7]: w[1,7] - v[1,7] >= 0
 ArcUsedByRobot[1,8]: w[1,8] - v[1,8] >= 0
 ArcUsedByRobot[1,9]: w[1,9] - v[1,9] >= 0
 ArcUsedByRobot[1,10]: w[1,10] - v[1,10] >= 0
 ArcUsedByRobot[1,11]: w[1,11] - v[1,11] >= 0
 ArcUsedByRobot[1,12]: w[1,12] - v[1,12] >= 0
 ArcUsedByRobot[1,13]: w[1,13] - v[1,13] >= 0
 ArcUsedByRobot[1,14]: w[1,14] - v[1,14] >= 0
 ArcUsedByRobot[1,15]: w[1,15] - v[1,15] >= 0
 ArcUsedByRobot[1,16]: w[1,16] - v[1,16] >= 0
 ArcUsedByRobot[1,17]: w[1,17] - v[1,17] >= 0
 ArcUsedByRobot[1,18]: w[1,18] - v[1,18] >= 0
 ArcUsedByRobot[1,19]: w[1,19] - v[1,19] >= 0
 ArcUsedByRobot[1,20]: w[1,20] - v[1,20] >= 0
 ArcUsedByRobot[1,21]: w[1,21] - v[1,21] >= 0
 ArcUsedByRobot[1,22]: w[1,22] - v[1,22] >= 0
 ArcUsedByRobot[1,23]: w[1,23] - v[1,23] >= 0
 ArcUsedByRobot[2,1]: w[2,1] - v[2,1] >= 0
 ArcUsedByRobot[2,2]: w[2,2] - v[2,2] >= 0
 ArcUsedByRobot[2,3]: w[2,3] - v[2,3] >= 0
 ArcUsedByRobot[2,4]: w[2,4] - v[2,4] >= 0
 ArcUsedByRobot[2,5]: w[2,5] - v[2,5] >= 0
 ArcUsedByRobot[2,6]: w[2,6] - v[2,6] >= 0
 ArcUsedByRobot[2,7]: w[2,7] - v[2,7] >= 0
 ArcUsedByRobot[2,8]: w[2,8] - v[2,8] >= 0
 ArcUsedByRobot[2,9]: w[2,9] - v[2,9] >= 0
 ArcUsedByRobot[2,10]: w[2,10] - v[2,10] >= 0
 ArcUsedByRobot[2,11]: w[2,11] - v[2,11] >= 0
 ArcUsedByRobot[2,12]: w[2,12] - v[2,12] >= 0
 ArcUsedByRobot[2,13]: w[2,13] - v[2,13] >= 0
 ArcUsedByRobot[2,14]: w[2,14] - v[2,14] >= 0
 ArcUsedByRobot[2,15]: w[2,15] - v[2,15] >= 0
 ArcUsedByRobot[2,16]: w[2,16] - v[2,16] >= 0
 ArcUsedByRobot[2,17]: w[2,17] - v[2,17] >= 0
 ArcUsedByRobot[2,18]: w[2,18] - v[2,18] >= 0
 ArcUsedByRobot[2,19]: w[2,19] - v[2,19] >= 0
 ArcUsedByRobot[2,20]: w[2,20] - v[2,20] >= 0
 ArcUsedByRobot[2,21]: w[2,21] - v[2,21] >= 0
 ArcUsedByRobot[2,22]: w[2,22] - v[2,22] >= 0
 ArcUsedByRobot[2,23]: w[2,23] - v[2,23] >= 0
 ArcUsedByRobot[3,1]: w[3,1] - v[3,1] >= 0
 ArcUsedByRobot[3,2]: w[3,2] - v[3,2] >= 0
 ArcUsedByRobot[3,3]: w[3,3] - v[3,3] >= 0
 ArcUsedByRobot[3,4]: w[3,4] - v[3,4] >= 0
 ArcUsedByRobot[3,5]: w[3,5] - v[3,5] >= 0
 ArcUsedByRobot[3,6]: w[3,6] - v[3,6] >= 0
 ArcUsedByRobot[3,7]: w[3,7] - v[3,7] >= 0
 ArcUsedByRobot[3,8]: w[3,8] - v[3,8] >= 0
 ArcUsedByRobot[3,9]: w[3,9] - v[3,9] >= 0
 ArcUsedByRobot[3,10]: w[3,10] - v[3,10] >= 0
 ArcUsedByRobot[3,11]: w[3,11] - v[3,11] >= 0
 ArcUsedByRobot[3,12]: w[3,12] - v[3,12] >= 0
 ArcUsedByRobot[3,13]: w[3,13] - v[3,13] >= 0
 ArcUsedByRobot[3,14]: w[3,14] - v[3,14] >= 0
 ArcUsedByRobot[3,15]: w[3,15] - v[3,15] >= 0
 ArcUsedByRobot[3,16]: w[3,16] - v[3,16] >= 0
 ArcUsedByRobot[3,17]: w[3,17] - v[3,17] >= 0
 ArcUsedByRobot[3,18]: w[3,18] - v[3,18] >= 0
 ArcUsedByRobot[3,19]: w[3,19] - v[3,19] >= 0
 ArcUsedByRobot[3,20]: w[3,20] - v[3,20] >= 0
 ArcUsedByRobot[3,21]: w[3,21] - v[3,21] >= 0
 ArcUsedByRobot[3,22]: w[3,22] - v[3,22] >= 0
 ArcUsedByRobot[3,23]: w[3,23] - v[3,23] >= 0
 ArcUsedByRobot[4,1]: w[4,1] - v[4,1] >= 0
 ArcUsedByRobot[4,2]: w[4,2] - v[4,2] >= 0
 ArcUsedByRobot[4,3]: w[4,3] - v[4,3] >= 0
 ArcUsedByRobot[4,4]: w[4,4] - v[4,4] >= 0
 ArcUsedByRobot[4,5]: w[4,5] - v[4,5] >= 0
 ArcUsedByRobot[4,6]: w[4,6] - v[4,6] >= 0
 ArcUsedByRobot[4,7]: w[4,7] - v[4,7] >= 0
 ArcUsedByRobot[4,8]: w[4,8] - v[4,8] >= 0
 ArcUsedByRobot[4,9]: w[4,9] - v[4,9] >= 0
 ArcUsedByRobot[4,10]: w[4,10] - v[4,10] >= 0
 ArcUsedByRobot[4,11]: w[4,11] - v[4,11] >= 0
 ArcUsedByRobot[4,12]: w[4,12] - v[4,12] >= 0
 ArcUsedByRobot[4,13]: w[4,13] - v[4,13] >= 0
 ArcUsedByRobot[4,14]: w[4,14] - v[4,14] >= 0
 ArcUsedByRobot[4,15]: w[4,15] - v[4,15] >= 0
 ArcUsedByRobot[4,16]: w[4,16] - v[4,16] >= 0
 ArcUsedByRobot[4,17]: w[4,17] - v[4,17] >= 0
 ArcUsedByRobot[4,18]: w[4,18] - v[4,18] >= 0
 ArcUsedByRobot[4,19]: w[4,19] - v[4,19] >= 0
 ArcUsedByRobot[4,20]: w[4,20] - v[4,20] >= 0
 ArcUsedByRobot[4,21]: w[4,21] - v[4,21] >= 0
 ArcUsedByRobot[4,22]: w[4,22] - v[4,22] >= 0
 ArcUsedByRobot[4,23]: w[4,23] - v[4,23] >= 0
 ArcUsedByRobot[5,1]: w[5,1] - v[5,1] >= 0
 ArcUsedByRobot[5,2]: w[5,2] - v[5,2] >= 0
 ArcUsedByRobot[5,3]: w[5,3] - v[5,3] >= 0
 ArcUsedByRobot[5,4]: w[5,4] - v[5,4] >= 0
 ArcUsedByRobot[5,5]: w[5,5] - v[5,5] >= 0
 ArcUsedByRobot[5,6]: w[5,6] - v[5,6] >= 0
 ArcUsedByRobot[5,7]: w[5,7] - v[5,7] >= 0
 ArcUsedByRobot[5,8]: w[5,8] - v[5,8] >= 0
 ArcUsedByRobot[5,9]: w[5,9] - v[5,9] >= 0
 ArcUsedByRobot[5,10]: w[5,10] - v[5,10] >= 0
 ArcUsedByRobot[5,11]: w[5,11] - v[5,11] >= 0
 ArcUsedByRobot[5,12]: w[5,12] - v[5,12] >= 0
 ArcUsedByRobot[5,13]: w[5,13] - v[5,13] >= 0
 ArcUsedByRobot[5,14]: w[5,14] - v[5,14] >= 0
 ArcUsedByRobot[5,15]: w[5,15] - v[5,15] >= 0
 ArcUsedByRobot[5,16]: w[5,16] - v[5,16] >= 0
 ArcUsedByRobot[5,17]: w[5,17] - v[5,17] >= 0
 ArcUsedByRobot[5,18]: w[5,18] - v[5,18] >= 0
 ArcUsedByRobot[5,19]: w[5,19] - v[5,19] >= 0
 ArcUsedByRobot[5,20]: w[5,20] - v[5,20] >= 0
 ArcUsedByRobot[5,21]: w[5,21] - v[5,21] >= 0
 ArcUsedByRobot[5,22]: w[5,22] - v[5,22] >= 0
 ArcUsedByRobot[5,23]: w[5,23] - v[5,23] >= 0
 ArcUsedByRobot[6,1]: w[6,1] - v[6,1] >= 0
 ArcUsedByRobot[6,2]: w[6,2] - v[6,2] >= 0
 ArcUsedByRobot[6,3]: w[6,3] - v[6,3] >= 0
 ArcUsedByRobot[6,4]: w[6,4] - v[6,4] >= 0
 ArcUsedByRobot[6,5]: w[6,5] - v[6,5] >= 0
 ArcUsedByRobot[6,6]: w[6,6] - v[6,6] >= 0
 ArcUsedByRobot[6,7]: w[6,7] - v[6,7] >= 0
 ArcUsedByRobot[6,8]: w[6,8] - v[6,8] >= 0
 ArcUsedByRobot[6,9]: w[6,9] - v[6,9] >= 0
 ArcUsedByRobot[6,10]: w[6,10] - v[6,10] >= 0
 ArcUsedByRobot[6,11]: w[6,11] - v[6,11] >= 0
 ArcUsedByRobot[6,12]: w[6,12] - v[6,12] >= 0
 ArcUsedByRobot[6,13]: w[6,13] - v[6,13] >= 0
 ArcUsedByRobot[6,14]: w[6,14] - v[6,14] >= 0
 ArcUsedByRobot[6,15]: w[6,15] - v[6,15] >= 0
 ArcUsedByRobot[6,16]: w[6,16] - v[6,16] >= 0
 ArcUsedByRobot[6,17]: w[6,17] - v[6,17] >= 0
 ArcUsedByRobot[6,18]: w[6,18] - v[6,18] >= 0
 ArcUsedByRobot[6,19]: w[6,19] - v[6,19] >= 0
 ArcUsedByRobot[6,20]: w[6,20] - v[6,20] >= 0
 ArcUsedByRobot[6,21]: w[6,21] - v[6,21] >= 0
 ArcUsedByRobot[6,22]: w[6,22] - v[6,22] >= 0
 ArcUsedByRobot[6,23]: w[6,23] - v[6,23] >= 0
 ArcUsedByRobot[7,1]: w[7,1] - v[7,1] >= 0
 ArcUsedByRobot[7,2]: w[7,2] - v[7,2] >= 0
 ArcUsedByRobot[7,3]: w[7,3] - v[7,3] >= 0
 ArcUsedByRobot[7,4]: w[7,4] - v[7,4] >= 0
 ArcUsedByRobot[7,5]: w[7,5] - v[7,5] >= 0
 ArcUsedByRobot[7,6]: w[7,6] - v[7,6] >= 0
 ArcUsedByRobot[7,7]: w[7,7] - v[7,7] >= 0
 ArcUsedByRobot[7,8]: w[7,8] - v[7,8] >= 0
 ArcUsedByRobot[7,9]: w[7,9] - v[7,9] >= 0
 ArcUsedByRobot[7,10]: w[7,10] - v[7,10] >= 0
 ArcUsedByRobot[7,11]: w[7,11] - v[7,11] >= 0
 ArcUsedByRobot[7,12]: w[7,12] - v[7,12] >= 0
 ArcUsedByRobot[7,13]: w[7,13] - v[7,13] >= 0
 ArcUsedByRobot[7,14]: w[7,14] - v[7,14] >= 0
 ArcUsedByRobot[7,15]: w[7,15] - v[7,15] >= 0
 ArcUsedByRobot[7,16]: w[7,16] - v[7,16] >= 0
 ArcUsedByRobot[7,17]: w[7,17] - v[7,17] >= 0
 ArcUsedByRobot[7,18]: w[7,18] - v[7,18] >= 0
 ArcUsedByRobot[7,19]: w[7,19] - v[7,19] >= 0
 ArcUsedByRobot[7,20]: w[7,20] - v[7,20] >= 0
 ArcUsedByRobot[7,21]: w[7,21] - v[7,21] >= 0
 ArcUsedByRobot[7,22]: w[7,22] - v[7,22] >= 0
 ArcUsedByRobot[7,23]: w[7,23] - v[7,23] >= 0
 ArcUsedByRobot[8,1]: w[8,1] - v[8,1] >= 0
 ArcUsedByRobot[8,2]: w[8,2] - v[8,2] >= 0
 ArcUsedByRobot[8,3]: w[8,3] - v[8,3] >= 0
 ArcUsedByRobot[8,4]: w[8,4] - v[8,4] >= 0
 ArcUsedByRobot[8,5]: w[8,5] - v[8,5] >= 0
 ArcUsedByRobot[8,6]: w[8,6] - v[8,6] >= 0
 ArcUsedByRobot[8,7]: w[8,7] - v[8,7] >= 0
 ArcUsedByRobot[8,8]: w[8,8] - v[8,8] >= 0
 ArcUsedByRobot[8,9]: w[8,9] - v[8,9] >= 0
 ArcUsedByRobot[8,10]: w[8,10] - v[8,10] >= 0
 ArcUsedByRobot[8,11]: w[8,11] - v[8,11] >= 0
 ArcUsedByRobot[8,12]: w[8,12] - v[8,12] >= 0
 ArcUsedByRobot[8,13]: w[8,13] - v[8,13] >= 0
 ArcUsedByRobot[8,14]: w[8,14] - v[8,14] >= 0
 ArcUsedByRobot[8,15]: w[8,15] - v[8,15] >= 0
 ArcUsedByRobot[8,16]: w[8,16] - v[8,16] >= 0
 ArcUsedByRobot[8,17]: w[8,17] - v[8,17] >= 0
 ArcUsedByRobot[8,18]: w[8,18] - v[8,18] >= 0
 ArcUsedByRobot[8,19]: w[8,19] - v[8,19] >= 0
 ArcUsedByRobot[8,20]: w[8,20] - v[8,20] >= 0
 ArcUsedByRobot[8,21]: w[8,21] - v[8,21] >= 0
 ArcUsedByRobot[8,22]: w[8,22] - v[8,22] >= 0
 ArcUsedByRobot[8,23]: w[8,23] - v[8,23] >= 0
 ArcUsedByRobot[9,1]: w[9,1] - v[9,1] >= 0
 ArcUsedByRobot[9,2]: w[9,2] - v[9,2] >= 0
 ArcUsedByRobot[9,3]: w[9,3] - v[9,3] >= 0
 ArcUsedByRobot[9,4]: w[9,4] - v[9,4] >= 0
 ArcUsedByRobot[9,5]: w[9,5] - v[9,5] >= 0
 ArcUsedByRobot[9,6]: w[9,6] - v[9,6] >= 0
 ArcUsedByRobot[9,7]: w[9,7] - v[9,7] >= 0
 ArcUsedByRobot[9,8]: w[9,8] - v[9,8] >= 0
 ArcUsedByRobot[9,9]: w[9,9] - v[9,9] >= 0
 ArcUsedByRobot[9,10]: w[9,10] - v[9,10] >= 0
 ArcUsedByRobot[9,11]: w[9,11] - v[9,11] >= 0
 ArcUsedByRobot[9,12]: w[9,12] - v[9,12] >= 0
 ArcUsedByRobot[9,13]: w[9,13] - v[9,13] >= 0
 ArcUsedByRobot[9,14]: w[9,14] - v[9,14] >= 0
 ArcUsedByRobot[9,15]: w[9,15] - v[9,15] >= 0
 ArcUsedByRobot[9,16]: w[9,16] - v[9,16] >= 0
 ArcUsedByRobot[9,17]: w[9,17] - v[9,17] >= 0
 ArcUsedByRobot[9,18]: w[9,18] - v[9,18] >= 0
 ArcUsedByRobot[9,19]: w[9,19] - v[9,19] >= 0
 ArcUsedByRobot[9,20]: w[9,20] - v[9,20] >= 0
 ArcUsedByRobot[9,21]: w[9,21] - v[9,21] >= 0
 ArcUsedByRobot[9,22]: w[9,22] - v[9,22] >= 0
 ArcUsedByRobot[9,23]: w[9,23] - v[9,23] >= 0
 ArcUsedByRobot[10,1]: w[10,1] - v[10,1] >= 0
 ArcUsedByRobot[10,2]: w[10,2] - v[10,2] >= 0
 ArcUsedByRobot[10,3]: w[10,3] - v[10,3] >= 0
 ArcUsedByRobot[10,4]: w[10,4] - v[10,4] >= 0
 ArcUsedByRobot[10,5]: w[10,5] - v[10,5] >= 0
 ArcUsedByRobot[10,6]: w[10,6] - v[10,6] >= 0
 ArcUsedByRobot[10,7]: w[10,7] - v[10,7] >= 0
 ArcUsedByRobot[10,8]: w[10,8] - v[10,8] >= 0
 ArcUsedByRobot[10,9]: w[10,9] - v[10,9] >= 0
 ArcUsedByRobot[10,10]: w[10,10] - v[10,10] >= 0
 ArcUsedByRobot[10,11]: w[10,11] - v[10,11] >= 0
 ArcUsedByRobot[10,12]: w[10,12] - v[10,12] >= 0
 ArcUsedByRobot[10,13]: w[10,13] - v[10,13] >= 0
 ArcUsedByRobot[10,14]: w[10,14] - v[10,14] >= 0
 ArcUsedByRobot[10,15]: w[10,15] - v[10,15] >= 0
 ArcUsedByRobot[10,16]: w[10,16] - v[10,16] >= 0
 ArcUsedByRobot[10,17]: w[10,17] - v[10,17] >= 0
 ArcUsedByRobot[10,18]: w[10,18] - v[10,18] >= 0
 ArcUsedByRobot[10,19]: w[10,19] - v[10,19] >= 0
 ArcUsedByRobot[10,20]: w[10,20] - v[10,20] >= 0
 ArcUsedByRobot[10,21]: w[10,21] - v[10,21] >= 0
 ArcUsedByRobot[10,22]: w[10,22] - v[10,22] >= 0
 ArcUsedByRobot[10,23]: w[10,23] - v[10,23] >= 0
 ArcUsedByRobot[11,1]: w[11,1] - v[11,1] >= 0
 ArcUsedByRobot[11,2]: w[11,2] - v[11,2] >= 0
 ArcUsedByRobot[11,3]: w[11,3] - v[11,3] >= 0
 ArcUsedByRobot[11,4]: w[11,4] - v[11,4] >= 0
 ArcUsedByRobot[11,5]: w[11,5] - v[11,5] >= 0
 ArcUsedByRobot[11,6]: w[11,6] - v[11,6] >= 0
 ArcUsedByRobot[11,7]: w[11,7] - v[11,7] >= 0
 ArcUsedByRobot[11,8]: w[11,8] - v[11,8] >= 0
 ArcUsedByRobot[11,9]: w[11,9] - v[11,9] >= 0
 ArcUsedByRobot[11,10]: w[11,10] - v[11,10] >= 0
 ArcUsedByRobot[11,11]: w[11,11] - v[11,11] >= 0
 ArcUsedByRobot[11,12]: w[11,12] - v[11,12] >= 0
 ArcUsedByRobot[11,13]: w[11,13] - v[11,13] >= 0
 ArcUsedByRobot[11,14]: w[11,14] - v[11,14] >= 0
 ArcUsedByRobot[11,15]: w[11,15] - v[11,15] >= 0
 ArcUsedByRobot[11,16]: w[11,16] - v[11,16] >= 0
 ArcUsedByRobot[11,17]: w[11,17] - v[11,17] >= 0
 ArcUsedByRobot[11,18]: w[11,18] - v[11,18] >= 0
 ArcUsedByRobot[11,19]: w[11,19] - v[11,19] >= 0
 ArcUsedByRobot[11,20]: w[11,20] - v[11,20] >= 0
 ArcUsedByRobot[11,21]: w[11,21] - v[11,21] >= 0
 ArcUsedByRobot[11,22]: w[11,22] - v[11,22] >= 0
 ArcUsedByRobot[11,23]: w[11,23] - v[11,23] >= 0
 ArcUsedByRobot[12,1]: w[12,1] - v[12,1] >= 0
 ArcUsedByRobot[12,2]: w[12,2] - v[12,2] >= 0
 ArcUsedByRobot[12,3]: w[12,3] - v[12,3] >= 0
 ArcUsedByRobot[12,4]: w[12,4] - v[12,4] >= 0
 ArcUsedByRobot[12,5]: w[12,5] - v[12,5] >= 0
 ArcUsedByRobot[12,6]: w[12,6] - v[12,6] >= 0
 ArcUsedByRobot[12,7]: w[12,7] - v[12,7] >= 0
 ArcUsedByRobot[12,8]: w[12,8] - v[12,8] >= 0
 ArcUsedByRobot[12,9]: w[12,9] - v[12,9] >= 0
 ArcUsedByRobot[12,10]: w[12,10] - v[12,10] >= 0
 ArcUsedByRobot[12,11]: w[12,11] - v[12,11] >= 0
 ArcUsedByRobot[12,12]: w[12,12] - v[12,12] >= 0
 ArcUsedByRobot[12,13]: w[12,13] - v[12,13] >= 0
 ArcUsedByRobot[12,14]: w[12,14] - v[12,14] >= 0
 ArcUsedByRobot[12,15]: w[12,15] - v[12,15] >= 0
 ArcUsedByRobot[12,16]: w[12,16] - v[12,16] >= 0
 ArcUsedByRobot[12,17]: w[12,17] - v[12,17] >= 0
 ArcUsedByRobot[12,18]: w[12,18] - v[12,18] >= 0
 ArcUsedByRobot[12,19]: w[12,19] - v[12,19] >= 0
 ArcUsedByRobot[12,20]: w[12,20] - v[12,20] >= 0
 ArcUsedByRobot[12,21]: w[12,21] - v[12,21] >= 0
 ArcUsedByRobot[12,22]: w[12,22] - v[12,22] >= 0
 ArcUsedByRobot[12,23]: w[12,23] - v[12,23] >= 0
 ArcUsedByRobot[13,1]: w[13,1] - v[13,1] >= 0
 ArcUsedByRobot[13,2]: w[13,2] - v[13,2] >= 0
 ArcUsedByRobot[13,3]: w[13,3] - v[13,3] >= 0
 ArcUsedByRobot[13,4]: w[13,4] - v[13,4] >= 0
 ArcUsedByRobot[13,5]: w[13,5] - v[13,5] >= 0
 ArcUsedByRobot[13,6]: w[13,6] - v[13,6] >= 0
 ArcUsedByRobot[13,7]: w[13,7] - v[13,7] >= 0
 ArcUsedByRobot[13,8]: w[13,8] - v[13,8] >= 0
 ArcUsedByRobot[13,9]: w[13,9] - v[13,9] >= 0
 ArcUsedByRobot[13,10]: w[13,10] - v[13,10] >= 0
 ArcUsedByRobot[13,11]: w[13,11] - v[13,11] >= 0
 ArcUsedByRobot[13,12]: w[13,12] - v[13,12] >= 0
 ArcUsedByRobot[13,13]: w[13,13] - v[13,13] >= 0
 ArcUsedByRobot[13,14]: w[13,14] - v[13,14] >= 0
 ArcUsedByRobot[13,15]: w[13,15] - v[13,15] >= 0
 ArcUsedByRobot[13,16]: w[13,16] - v[13,16] >= 0
 ArcUsedByRobot[13,17]: w[13,17] - v[13,17] >= 0
 ArcUsedByRobot[13,18]: w[13,18] - v[13,18] >= 0
 ArcUsedByRobot[13,19]: w[13,19] - v[13,19] >= 0
 ArcUsedByRobot[13,20]: w[13,20] - v[13,20] >= 0
 ArcUsedByRobot[13,21]: w[13,21] - v[13,21] >= 0
 ArcUsedByRobot[13,22]: w[13,22] - v[13,22] >= 0
 ArcUsedByRobot[13,23]: w[13,23] - v[13,23] >= 0
 ArcUsedByRobot[14,1]: w[14,1] - v[14,1] >= 0
 ArcUsedByRobot[14,2]: w[14,2] - v[14,2] >= 0
 ArcUsedByRobot[14,3]: w[14,3] - v[14,3] >= 0
 ArcUsedByRobot[14,4]: w[14,4] - v[14,4] >= 0
 ArcUsedByRobot[14,5]: w[14,5] - v[14,5] >= 0
 ArcUsedByRobot[14,6]: w[14,6] - v[14,6] >= 0
 ArcUsedByRobot[14,7]: w[14,7] - v[14,7] >= 0
 ArcUsedByRobot[14,8]: w[14,8] - v[14,8] >= 0
 ArcUsedByRobot[14,9]: w[14,9] - v[14,9] >= 0
 ArcUsedByRobot[14,10]: w[14,10] - v[14,10] >= 0
 ArcUsedByRobot[14,11]: w[14,11] - v[14,11] >= 0
 ArcUsedByRobot[14,12]: w[14,12] - v[14,12] >= 0
 ArcUsedByRobot[14,13]: w[14,13] - v[14,13] >= 0
 ArcUsedByRobot[14,14]: w[14,14] - v[14,14] >= 0
 ArcUsedByRobot[14,15]: w[14,15] - v[14,15] >= 0
 ArcUsedByRobot[14,16]: w[14,16] - v[14,16] >= 0
 ArcUsedByRobot[14,17]: w[14,17] - v[14,17] >= 0
 ArcUsedByRobot[14,18]: w[14,18] - v[14,18] >= 0
 ArcUsedByRobot[14,19]: w[14,19] - v[14,19] >= 0
 ArcUsedByRobot[14,20]: w[14,20] - v[14,20] >= 0
 ArcUsedByRobot[14,21]: w[14,21] - v[14,21] >= 0
 ArcUsedByRobot[14,22]: w[14,22] - v[14,22] >= 0
 ArcUsedByRobot[14,23]: w[14,23] - v[14,23] >= 0
 ArcUsedByRobot[15,1]: w[15,1] - v[15,1] >= 0
 ArcUsedByRobot[15,2]: w[15,2] - v[15,2] >= 0
 ArcUsedByRobot[15,3]: w[15,3] - v[15,3] >= 0
 ArcUsedByRobot[15,4]: w[15,4] - v[15,4] >= 0
 ArcUsedByRobot[15,5]: w[15,5] - v[15,5] >= 0
 ArcUsedByRobot[15,6]: w[15,6] - v[15,6] >= 0
 ArcUsedByRobot[15,7]: w[15,7] - v[15,7] >= 0
 ArcUsedByRobot[15,8]: w[15,8] - v[15,8] >= 0
 ArcUsedByRobot[15,9]: w[15,9] - v[15,9] >= 0
 ArcUsedByRobot[15,10]: w[15,10] - v[15,10] >= 0
 ArcUsedByRobot[15,11]: w[15,11] - v[15,11] >= 0
 ArcUsedByRobot[15,12]: w[15,12] - v[15,12] >= 0
 ArcUsedByRobot[15,13]: w[15,13] - v[15,13] >= 0
 ArcUsedByRobot[15,14]: w[15,14] - v[15,14] >= 0
 ArcUsedByRobot[15,15]: w[15,15] - v[15,15] >= 0
 ArcUsedByRobot[15,16]: w[15,16] - v[15,16] >= 0
 ArcUsedByRobot[15,17]: w[15,17] - v[15,17] >= 0
 ArcUsedByRobot[15,18]: w[15,18] - v[15,18] >= 0
 ArcUsedByRobot[15,19]: w[15,19] - v[15,19] >= 0
 ArcUsedByRobot[15,20]: w[15,20] - v[15,20] >= 0
 ArcUsedByRobot[15,21]: w[15,21] - v[15,21] >= 0
 ArcUsedByRobot[15,22]: w[15,22] - v[15,22] >= 0
 ArcUsedByRobot[15,23]: w[15,23] - v[15,23] >= 0
 ArcUsedByRobot[16,1]: w[16,1] - v[16,1] >= 0
 ArcUsedByRobot[16,2]: w[16,2] - v[16,2] >= 0
 ArcUsedByRobot[16,3]: w[16,3] - v[16,3] >= 0
 ArcUsedByRobot[16,4]: w[16,4] - v[16,4] >= 0
 ArcUsedByRobot[16,5]: w[16,5] - v[16,5] >= 0
 ArcUsedByRobot[16,6]: w[16,6] - v[16,6] >= 0
 ArcUsedByRobot[16,7]: w[16,7] - v[16,7] >= 0
 ArcUsedByRobot[16,8]: w[16,8] - v[16,8] >= 0
 ArcUsedByRobot[16,9]: w[16,9] - v[16,9] >= 0
 ArcUsedByRobot[16,10]: w[16,10] - v[16,10] >= 0
 ArcUsedByRobot[16,11]: w[16,11] - v[16,11] >= 0
 ArcUsedByRobot[16,12]: w[16,12] - v[16,12] >= 0
 ArcUsedByRobot[16,13]: w[16,13] - v[16,13] >= 0
 ArcUsedByRobot[16,14]: w[16,14] - v[16,14] >= 0
 ArcUsedByRobot[16,15]: w[16,15] - v[16,15] >= 0
 ArcUsedByRobot[16,16]: w[16,16] - v[16,16] >= 0
 ArcUsedByRobot[16,17]: w[16,17] - v[16,17] >= 0
 ArcUsedByRobot[16,18]: w[16,18] - v[16,18] >= 0
 ArcUsedByRobot[16,19]: w[16,19] - v[16,19] >= 0
 ArcUsedByRobot[16,20]: w[16,20] - v[16,20] >= 0
 ArcUsedByRobot[16,21]: w[16,21] - v[16,21] >= 0
 ArcUsedByRobot[16,22]: w[16,22] - v[16,22] >= 0
 ArcUsedByRobot[16,23]: w[16,23] - v[16,23] >= 0
 ArcUsedByRobot[17,1]: w[17,1] - v[17,1] >= 0
 ArcUsedByRobot[17,2]: w[17,2] - v[17,2] >= 0
 ArcUsedByRobot[17,3]: w[17,3] - v[17,3] >= 0
 ArcUsedByRobot[17,4]: w[17,4] - v[17,4] >= 0
 ArcUsedByRobot[17,5]: w[17,5] - v[17,5] >= 0
 ArcUsedByRobot[17,6]: w[17,6] - v[17,6] >= 0
 ArcUsedByRobot[17,7]: w[17,7] - v[17,7] >= 0
 ArcUsedByRobot[17,8]: w[17,8] - v[17,8] >= 0
 ArcUsedByRobot[17,9]: w[17,9] - v[17,9] >= 0
 ArcUsedByRobot[17,10]: w[17,10] - v[17,10] >= 0
 ArcUsedByRobot[17,11]: w[17,11] - v[17,11] >= 0
 ArcUsedByRobot[17,12]: w[17,12] - v[17,12] >= 0
 ArcUsedByRobot[17,13]: w[17,13] - v[17,13] >= 0
 ArcUsedByRobot[17,14]: w[17,14] - v[17,14] >= 0
 ArcUsedByRobot[17,15]: w[17,15] - v[17,15] >= 0
 ArcUsedByRobot[17,16]: w[17,16] - v[17,16] >= 0
 ArcUsedByRobot[17,17]: w[17,17] - v[17,17] >= 0
 ArcUsedByRobot[17,18]: w[17,18] - v[17,18] >= 0
 ArcUsedByRobot[17,19]: w[17,19] - v[17,19] >= 0
 ArcUsedByRobot[17,20]: w[17,20] - v[17,20] >= 0
 ArcUsedByRobot[17,21]: w[17,21] - v[17,21] >= 0
 ArcUsedByRobot[17,22]: w[17,22] - v[17,22] >= 0
 ArcUsedByRobot[17,23]: w[17,23] - v[17,23] >= 0
 ArcUsedByRobot[18,1]: w[18,1] - v[18,1] >= 0
 ArcUsedByRobot[18,2]: w[18,2] - v[18,2] >= 0
 ArcUsedByRobot[18,3]: w[18,3] - v[18,3] >= 0
 ArcUsedByRobot[18,4]: w[18,4] - v[18,4] >= 0
 ArcUsedByRobot[18,5]: w[18,5] - v[18,5] >= 0
 ArcUsedByRobot[18,6]: w[18,6] - v[18,6] >= 0
 ArcUsedByRobot[18,7]: w[18,7] - v[18,7] >= 0
 ArcUsedByRobot[18,8]: w[18,8] - v[18,8] >= 0
 ArcUsedByRobot[18,9]: w[18,9] - v[18,9] >= 0
 ArcUsedByRobot[18,10]: w[18,10] - v[18,10] >= 0
 ArcUsedByRobot[18,11]: w[18,11] - v[18,11] >= 0
 ArcUsedByRobot[18,12]: w[18,12] - v[18,12] >= 0
 ArcUsedByRobot[18,13]: w[18,13] - v[18,13] >= 0
 ArcUsedByRobot[18,14]: w[18,14] - v[18,14] >= 0
 ArcUsedByRobot[18,15]: w[18,15] - v[18,15] >= 0
 ArcUsedByRobot[18,16]: w[18,16] - v[18,16] >= 0
 ArcUsedByRobot[18,17]: w[18,17] - v[18,17] >= 0
 ArcUsedByRobot[18,18]: w[18,18] - v[18,18] >= 0
 ArcUsedByRobot[18,19]: w[18,19] - v[18,19] >= 0
 ArcUsedByRobot[18,20]: w[18,20] - v[18,20] >= 0
 ArcUsedByRobot[18,21]: w[18,21] - v[18,21] >= 0
 ArcUsedByRobot[18,22]: w[18,22] - v[18,22] >= 0
 ArcUsedByRobot[18,23]: w[18,23] - v[18,23] >= 0
 ArcUsedByRobot[19,1]: w[19,1] - v[19,1] >= 0
 ArcUsedByRobot[19,2]: w[19,2] - v[19,2] >= 0
 ArcUsedByRobot[19,3]: w[19,3] - v[19,3] >= 0
 ArcUsedByRobot[19,4]: w[19,4] - v[19,4] >= 0
 ArcUsedByRobot[19,5]: w[19,5] - v[19,5] >= 0
 ArcUsedByRobot[19,6]: w[19,6] - v[19,6] >= 0
 ArcUsedByRobot[19,7]: w[19,7] - v[19,7] >= 0
 ArcUsedByRobot[19,8]: w[19,8] - v[19,8] >= 0
 ArcUsedByRobot[19,9]: w[19,9] - v[19,9] >= 0
 ArcUsedByRobot[19,10]: w[19,10] - v[19,10] >= 0
 ArcUsedByRobot[19,11]: w[19,11] - v[19,11] >= 0
 ArcUsedByRobot[19,12]: w[19,12] - v[19,12] >= 0
 ArcUsedByRobot[19,13]: w[19,13] - v[19,13] >= 0
 ArcUsedByRobot[19,14]: w[19,14] - v[19,14] >= 0
 ArcUsedByRobot[19,15]: w[19,15] - v[19,15] >= 0
 ArcUsedByRobot[19,16]: w[19,16] - v[19,16] >= 0
 ArcUsedByRobot[19,17]: w[19,17] - v[19,17] >= 0
 ArcUsedByRobot[19,18]: w[19,18] - v[19,18] >= 0
 ArcUsedByRobot[19,19]: w[19,19] - v[19,19] >= 0
 ArcUsedByRobot[19,20]: w[19,20] - v[19,20] >= 0
 ArcUsedByRobot[19,21]: w[19,21] - v[19,21] >= 0
 ArcUsedByRobot[19,22]: w[19,22] - v[19,22] >= 0
 ArcUsedByRobot[19,23]: w[19,23] - v[19,23] >= 0
 ArcUsedByRobot[20,1]: w[20,1] - v[20,1] >= 0
 ArcUsedByRobot[20,2]: w[20,2] - v[20,2] >= 0
 ArcUsedByRobot[20,3]: w[20,3] - v[20,3] >= 0
 ArcUsedByRobot[20,4]: w[20,4] - v[20,4] >= 0
 ArcUsedByRobot[20,5]: w[20,5] - v[20,5] >= 0
 ArcUsedByRobot[20,6]: w[20,6] - v[20,6] >= 0
 ArcUsedByRobot[20,7]: w[20,7] - v[20,7] >= 0
 ArcUsedByRobot[20,8]: w[20,8] - v[20,8] >= 0
 ArcUsedByRobot[20,9]: w[20,9] - v[20,9] >= 0
 ArcUsedByRobot[20,10]: w[20,10] - v[20,10] >= 0
 ArcUsedByRobot[20,11]: w[20,11] - v[20,11] >= 0
 ArcUsedByRobot[20,12]: w[20,12] - v[20,12] >= 0
 ArcUsedByRobot[20,13]: w[20,13] - v[20,13] >= 0
 ArcUsedByRobot[20,14]: w[20,14] - v[20,14] >= 0
 ArcUsedByRobot[20,15]: w[20,15] - v[20,15] >= 0
 ArcUsedByRobot[20,16]: w[20,16] - v[20,16] >= 0
 ArcUsedByRobot[20,17]: w[20,17] - v[20,17] >= 0
 ArcUsedByRobot[20,18]: w[20,18] - v[20,18] >= 0
 ArcUsedByRobot[20,19]: w[20,19] - v[20,19] >= 0
 ArcUsedByRobot[20,20]: w[20,20] - v[20,20] >= 0
 ArcUsedByRobot[20,21]: w[20,21] - v[20,21] >= 0
 ArcUsedByRobot[20,22]: w[20,22] - v[20,22] >= 0
 ArcUsedByRobot[20,23]: w[20,23] - v[20,23] >= 0
 ArcUsedByRobot[21,1]: w[21,1] - v[21,1] >= 0
 ArcUsedByRobot[21,2]: w[21,2] - v[21,2] >= 0
 ArcUsedByRobot[21,3]: w[21,3] - v[21,3] >= 0
 ArcUsedByRobot[21,4]: w[21,4] - v[21,4] >= 0
 ArcUsedByRobot[21,5]: w[21,5] - v[21,5] >= 0
 ArcUsedByRobot[21,6]: w[21,6] - v[21,6] >= 0
 ArcUsedByRobot[21,7]: w[21,7] - v[21,7] >= 0
 ArcUsedByRobot[21,8]: w[21,8] - v[21,8] >= 0
 ArcUsedByRobot[21,9]: w[21,9] - v[21,9] >= 0
 ArcUsedByRobot[21,10]: w[21,10] - v[21,10] >= 0
 ArcUsedByRobot[21,11]: w[21,11] - v[21,11] >= 0
 ArcUsedByRobot[21,12]: w[21,12] - v[21,12] >= 0
 ArcUsedByRobot[21,13]: w[21,13] - v[21,13] >= 0
 ArcUsedByRobot[21,14]: w[21,14] - v[21,14] >= 0
 ArcUsedByRobot[21,15]: w[21,15] - v[21,15] >= 0
 ArcUsedByRobot[21,16]: w[21,16] - v[21,16] >= 0
 ArcUsedByRobot[21,17]: w[21,17] - v[21,17] >= 0
 ArcUsedByRobot[21,18]: w[21,18] - v[21,18] >= 0
 ArcUsedByRobot[21,19]: w[21,19] - v[21,19] >= 0
 ArcUsedByRobot[21,20]: w[21,20] - v[21,20] >= 0
 ArcUsedByRobot[21,21]: w[21,21] - v[21,21] >= 0
 ArcUsedByRobot[21,22]: w[21,22] - v[21,22] >= 0
 ArcUsedByRobot[21,23]: w[21,23] - v[21,23] >= 0
 ArcUsedByRobot[22,1]: w[22,1] - v[22,1] >= 0
 ArcUsedByRobot[22,2]: w[22,2] - v[22,2] >= 0
 ArcUsedByRobot[22,3]: w[22,3] - v[22,3] >= 0
 ArcUsedByRobot[22,4]: w[22,4] - v[22,4] >= 0
 ArcUsedByRobot[22,5]: w[22,5] - v[22,5] >= 0
 ArcUsedByRobot[22,6]: w[22,6] - v[22,6] >= 0
 ArcUsedByRobot[22,7]: w[22,7] - v[22,7] >= 0
 ArcUsedByRobot[22,8]: w[22,8] - v[22,8] >= 0
 ArcUsedByRobot[22,9]: w[22,9] - v[22,9] >= 0
 ArcUsedByRobot[22,10]: w[22,10] - v[22,10] >= 0
 ArcUsedByRobot[22,11]: w[22,11] - v[22,11] >= 0
 ArcUsedByRobot[22,12]: w[22,12] - v[22,12] >= 0
 ArcUsedByRobot[22,13]: w[22,13] - v[22,13] >= 0
 ArcUsedByRobot[22,14]: w[22,14] - v[22,14] >= 0
 ArcUsedByRobot[22,15]: w[22,15] - v[22,15] >= 0
 ArcUsedByRobot[22,16]: w[22,16] - v[22,16] >= 0
 ArcUsedByRobot[22,17]: w[22,17] - v[22,17] >= 0
 ArcUsedByRobot[22,18]: w[22,18] - v[22,18] >= 0
 ArcUsedByRobot[22,19]: w[22,19] - v[22,19] >= 0
 ArcUsedByRobot[22,20]: w[22,20] - v[22,20] >= 0
 ArcUsedByRobot[22,21]: w[22,21] - v[22,21] >= 0
 ArcUsedByRobot[22,22]: w[22,22] - v[22,22] >= 0
 ArcUsedByRobot[22,23]: w[22,23] - v[22,23] >= 0
 StartServiceRobots[0,1]: - 999999 v[0,1] - service_time[0]
   - start_service[0] + start_service[1] >= -999999
 StartServiceRobots[0,2]: - 999999 v[0,2] - service_time[0]
   - start_service[0] + start_service[2] >= -999945.4287857366
 StartServiceRobots[0,3]: - 999999 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999962.9193004568
 StartServiceRobots[0,4]: - 999999 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999937.583726213
 StartServiceRobots[0,5]: - 999999 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999947.3853854308
 StartServiceRobots[0,6]: - 999999 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999936.1484321967
 StartServiceRobots[0,7]: - 999999 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999941.126196999
 StartServiceRobots[0,8]: - 999999 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999951.7620508042
 StartServiceRobots[0,9]: - 999999 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999971.4290013848
 StartServiceRobots[0,10]: - 999999 v[0,10] - service_time[0]
   - start_service[0] + start_service[10] >= -999934.7445159944
 StartServiceRobots[0,11]: - 999999 v[0,11] - service_time[0]
   - start_service[0] + start_service[11] >= -999984.1635755197
 StartServiceRobots[0,12]: - 999999 v[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999945.4287857366
 StartServiceRobots[0,13]: - 999999 v[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999962.9193004568
 StartServiceRobots[0,14]: - 999999 v[0,14] - service_time[0]
   - start_service[0] + start_service[14] >= -999937.583726213
 StartServiceRobots[0,15]: - 999999 v[0,15] - service_time[0]
   - start_service[0] + start_service[15] >= -999947.3853854308
 StartServiceRobots[0,16]: - 999999 v[0,16] - service_time[0]
   - start_service[0] + start_service[16] >= -999936.1484321967
 StartServiceRobots[0,17]: - 999999 v[0,17] - service_time[0]
   - start_service[0] + start_service[17] >= -999941.126196999
 StartServiceRobots[0,18]: - 999999 v[0,18] - service_time[0]
   - start_service[0] + start_service[18] >= -999951.7620508042
 StartServiceRobots[0,19]: - 999999 v[0,19] - service_time[0]
   - start_service[0] + start_service[19] >= -999971.4290013848
 StartServiceRobots[0,20]: - 999999 v[0,20] - service_time[0]
   - start_service[0] + start_service[20] >= -999934.7445159944
 StartServiceRobots[0,21]: - 999999 v[0,21] - service_time[0]
   - start_service[0] + start_service[21] >= -999984.1635755197
 StartServiceRobots[0,22]: - 999999 v[0,22] - service_time[0]
   - start_service[0] + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999 v[0,23] - service_time[0]
   - start_service[0] + start_service[23] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 999999 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999945.4287857366
 StartServiceRobots[1,3]: - 999999 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999962.9193004568
 StartServiceRobots[1,4]: - 999999 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999937.583726213
 StartServiceRobots[1,5]: - 999999 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999947.3853854308
 StartServiceRobots[1,6]: - 999999 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999936.1484321967
 StartServiceRobots[1,7]: - 999999 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999941.126196999
 StartServiceRobots[1,8]: - 999999 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999951.7620508042
 StartServiceRobots[1,9]: - 999999 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999971.4290013848
 StartServiceRobots[1,10]: - 999999 v[1,10] - service_time[1]
   - start_service[1] + start_service[10] >= -999934.7445159944
 StartServiceRobots[1,11]: - 999999 v[1,11] - service_time[1]
   - start_service[1] + start_service[11] >= -999984.1635755197
 StartServiceRobots[1,12]: - 999999 v[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999945.4287857366
 StartServiceRobots[1,13]: - 999999 v[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999962.9193004568
 StartServiceRobots[1,14]: - 999999 v[1,14] - service_time[1]
   - start_service[1] + start_service[14] >= -999937.583726213
 StartServiceRobots[1,15]: - 999999 v[1,15] - service_time[1]
   - start_service[1] + start_service[15] >= -999947.3853854308
 StartServiceRobots[1,16]: - 999999 v[1,16] - service_time[1]
   - start_service[1] + start_service[16] >= -999936.1484321967
 StartServiceRobots[1,17]: - 999999 v[1,17] - service_time[1]
   - start_service[1] + start_service[17] >= -999941.126196999
 StartServiceRobots[1,18]: - 999999 v[1,18] - service_time[1]
   - start_service[1] + start_service[18] >= -999951.7620508042
 StartServiceRobots[1,19]: - 999999 v[1,19] - service_time[1]
   - start_service[1] + start_service[19] >= -999971.4290013848
 StartServiceRobots[1,20]: - 999999 v[1,20] - service_time[1]
   - start_service[1] + start_service[20] >= -999934.7445159944
 StartServiceRobots[1,21]: - 999999 v[1,21] - service_time[1]
   - start_service[1] + start_service[21] >= -999984.1635755197
 StartServiceRobots[1,22]: - 999999 v[1,22] - service_time[1]
   - start_service[1] + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999 v[1,23] - service_time[1]
   - start_service[1] + start_service[23] >= -999999
 StartServiceRobots[2,1]: - 999999 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999945.4287857366
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 999999 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999955.9804138733
 StartServiceRobots[2,4]: - 999999 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999938.8231751657
 StartServiceRobots[2,5]: - 999999 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999926.0251344698
 StartServiceRobots[2,6]: - 999999 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999925.9721696612
 StartServiceRobots[2,7]: - 999999 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999933.5494026708
 StartServiceRobots[2,8]: - 999999 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999979.7861738935
 StartServiceRobots[2,9]: - 999999 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999931.6046053576
 StartServiceRobots[2,10]: - 999999 v[2,10] - service_time[2]
   - start_service[2] + start_service[10] >= -999987.1892001542
 StartServiceRobots[2,11]: - 999999 v[2,11] - service_time[2]
   - start_service[2] + start_service[11] >= -999932.5412105162
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 999999 v[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999955.9804138733
 StartServiceRobots[2,14]: - 999999 v[2,14] - service_time[2]
   - start_service[2] + start_service[14] >= -999938.8231751657
 StartServiceRobots[2,15]: - 999999 v[2,15] - service_time[2]
   - start_service[2] + start_service[15] >= -999926.0251344698
 StartServiceRobots[2,16]: - 999999 v[2,16] - service_time[2]
   - start_service[2] + start_service[16] >= -999925.9721696612
 StartServiceRobots[2,17]: - 999999 v[2,17] - service_time[2]
   - start_service[2] + start_service[17] >= -999933.5494026708
 StartServiceRobots[2,18]: - 999999 v[2,18] - service_time[2]
   - start_service[2] + start_service[18] >= -999979.7861738935
 StartServiceRobots[2,19]: - 999999 v[2,19] - service_time[2]
   - start_service[2] + start_service[19] >= -999931.6046053576
 StartServiceRobots[2,20]: - 999999 v[2,20] - service_time[2]
   - start_service[2] + start_service[20] >= -999987.1892001542
 StartServiceRobots[2,21]: - 999999 v[2,21] - service_time[2]
   - start_service[2] + start_service[21] >= -999932.5412105162
 StartServiceRobots[2,22]: - 999999 v[2,22] - service_time[2]
   - start_service[2] + start_service[22] >= -999945.4287857366
 StartServiceRobots[2,23]: - 999999 v[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= -999945.4287857366
 StartServiceRobots[3,1]: - 999999 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999962.9193004568
 StartServiceRobots[3,2]: - 999999 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999955.9804138733
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 999999 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999972.474248387
 StartServiceRobots[3,5]: - 999999 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999916.1872075291
 StartServiceRobots[3,6]: - 999999 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999908.5111139182
 StartServiceRobots[3,7]: - 999999 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999972.6248534749
 StartServiceRobots[3,8]: - 999999 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999973.4499059909
 StartServiceRobots[3,9]: - 999999 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999936.1977742163
 StartServiceRobots[3,10]: - 999999 v[3,10] - service_time[3]
   - start_service[3] + start_service[10] >= -999944.4584358049
 StartServiceRobots[3,11]: - 999999 v[3,11] - service_time[3]
   - start_service[3] + start_service[11] >= -999959.1641715461
 StartServiceRobots[3,12]: - 999999 v[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999955.9804138733
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 999999 v[3,14] - service_time[3]
   - start_service[3] + start_service[14] >= -999972.474248387
 StartServiceRobots[3,15]: - 999999 v[3,15] - service_time[3]
   - start_service[3] + start_service[15] >= -999916.1872075291
 StartServiceRobots[3,16]: - 999999 v[3,16] - service_time[3]
   - start_service[3] + start_service[16] >= -999908.5111139182
 StartServiceRobots[3,17]: - 999999 v[3,17] - service_time[3]
   - start_service[3] + start_service[17] >= -999972.6248534749
 StartServiceRobots[3,18]: - 999999 v[3,18] - service_time[3]
   - start_service[3] + start_service[18] >= -999973.4499059909
 StartServiceRobots[3,19]: - 999999 v[3,19] - service_time[3]
   - start_service[3] + start_service[19] >= -999936.1977742163
 StartServiceRobots[3,20]: - 999999 v[3,20] - service_time[3]
   - start_service[3] + start_service[20] >= -999944.4584358049
 StartServiceRobots[3,21]: - 999999 v[3,21] - service_time[3]
   - start_service[3] + start_service[21] >= -999959.1641715461
 StartServiceRobots[3,22]: - 999999 v[3,22] - service_time[3]
   - start_service[3] + start_service[22] >= -999962.9193004568
 StartServiceRobots[3,23]: - 999999 v[3,23] - service_time[3]
   - start_service[3] + start_service[23] >= -999962.9193004568
 StartServiceRobots[4,1]: - 999999 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999937.583726213
 StartServiceRobots[4,2]: - 999999 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999938.8231751657
 StartServiceRobots[4,3]: - 999999 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999972.474248387
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 999999 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999889.662152088
 StartServiceRobots[4,6]: - 999999 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999882.2562239703
 StartServiceRobots[4,7]: - 999999 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999988.8635851692
 StartServiceRobots[4,8]: - 999999 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999957.9553616339
 StartServiceRobots[4,9]: - 999999 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999910.2653522416
 StartServiceRobots[4,10]: - 999999 v[4,10] - service_time[4]
   - start_service[4] + start_service[10] >= -999929.0561937442
 StartServiceRobots[4,11]: - 999999 v[4,11] - service_time[4]
   - start_service[4] + start_service[11] >= -999937.3674259853
 StartServiceRobots[4,12]: - 999999 v[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999938.8231751657
 StartServiceRobots[4,13]: - 999999 v[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999972.474248387
 StartServiceRobots[4,14]: - 999999 v[4,14] - service_time[4]
   - start_service[4] + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 999999 v[4,15] - service_time[4]
   - start_service[4] + start_service[15] >= -999889.662152088
 StartServiceRobots[4,16]: - 999999 v[4,16] - service_time[4]
   - start_service[4] + start_service[16] >= -999882.2562239703
 StartServiceRobots[4,17]: - 999999 v[4,17] - service_time[4]
   - start_service[4] + start_service[17] >= -999988.8635851692
 StartServiceRobots[4,18]: - 999999 v[4,18] - service_time[4]
   - start_service[4] + start_service[18] >= -999957.9553616339
 StartServiceRobots[4,19]: - 999999 v[4,19] - service_time[4]
   - start_service[4] + start_service[19] >= -999910.2653522416
 StartServiceRobots[4,20]: - 999999 v[4,20] - service_time[4]
   - start_service[4] + start_service[20] >= -999929.0561937442
 StartServiceRobots[4,21]: - 999999 v[4,21] - service_time[4]
   - start_service[4] + start_service[21] >= -999937.3674259853
 StartServiceRobots[4,22]: - 999999 v[4,22] - service_time[4]
   - start_service[4] + start_service[22] >= -999937.583726213
 StartServiceRobots[4,23]: - 999999 v[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= -999937.583726213
 StartServiceRobots[5,1]: - 999999 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999947.3853854308
 StartServiceRobots[5,2]: - 999999 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999926.0251344698
 StartServiceRobots[5,3]: - 999999 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999916.1872075291
 StartServiceRobots[5,4]: - 999999 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999889.662152088
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 999999 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999983.6081284663
 StartServiceRobots[5,7]: - 999999 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999891.227834211
 StartServiceRobots[5,8]: - 999999 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999918.6764967154
 StartServiceRobots[5,9]: - 999999 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999971.5729838425
 StartServiceRobots[5,10]: - 999999 v[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999921.6996578324
 StartServiceRobots[5,11]: - 999999 v[5,11] - service_time[5]
   - start_service[5] + start_service[11] >= -999937.5561410894
 StartServiceRobots[5,12]: - 999999 v[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999926.0251344698
 StartServiceRobots[5,13]: - 999999 v[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999916.1872075291
 StartServiceRobots[5,14]: - 999999 v[5,14] - service_time[5]
   - start_service[5] + start_service[14] >= -999889.662152088
 StartServiceRobots[5,15]: - 999999 v[5,15] - service_time[5]
   - start_service[5] + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 999999 v[5,16] - service_time[5]
   - start_service[5] + start_service[16] >= -999983.6081284663
 StartServiceRobots[5,17]: - 999999 v[5,17] - service_time[5]
   - start_service[5] + start_service[17] >= -999891.227834211
 StartServiceRobots[5,18]: - 999999 v[5,18] - service_time[5]
   - start_service[5] + start_service[18] >= -999918.6764967154
 StartServiceRobots[5,19]: - 999999 v[5,19] - service_time[5]
   - start_service[5] + start_service[19] >= -999971.5729838425
 StartServiceRobots[5,20]: - 999999 v[5,20] - service_time[5]
   - start_service[5] + start_service[20] >= -999921.6996578324
 StartServiceRobots[5,21]: - 999999 v[5,21] - service_time[5]
   - start_service[5] + start_service[21] >= -999937.5561410894
 StartServiceRobots[5,22]: - 999999 v[5,22] - service_time[5]
   - start_service[5] + start_service[22] >= -999947.3853854308
 StartServiceRobots[5,23]: - 999999 v[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= -999947.3853854308
 StartServiceRobots[6,1]: - 999999 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999936.1484321967
 StartServiceRobots[6,2]: - 999999 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999925.9721696612
 StartServiceRobots[6,3]: - 999999 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999908.5111139182
 StartServiceRobots[6,4]: - 999999 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999882.2562239703
 StartServiceRobots[6,5]: - 999999 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999983.6081284663
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 999999 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999882.642633686
 StartServiceRobots[6,8]: - 999999 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999915.2753088792
 StartServiceRobots[6,9]: - 999999 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999957.385467226
 StartServiceRobots[6,10]: - 999999 v[6,10] - service_time[6]
   - start_service[6] + start_service[10] >= -999923.9891144931
 StartServiceRobots[6,11]: - 999999 v[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999924.7300771383
 StartServiceRobots[6,12]: - 999999 v[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999925.9721696612
 StartServiceRobots[6,13]: - 999999 v[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999908.5111139182
 StartServiceRobots[6,14]: - 999999 v[6,14] - service_time[6]
   - start_service[6] + start_service[14] >= -999882.2562239703
 StartServiceRobots[6,15]: - 999999 v[6,15] - service_time[6]
   - start_service[6] + start_service[15] >= -999983.6081284663
 StartServiceRobots[6,16]: - 999999 v[6,16] - service_time[6]
   - start_service[6] + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 999999 v[6,17] - service_time[6]
   - start_service[6] + start_service[17] >= -999882.642633686
 StartServiceRobots[6,18]: - 999999 v[6,18] - service_time[6]
   - start_service[6] + start_service[18] >= -999915.2753088792
 StartServiceRobots[6,19]: - 999999 v[6,19] - service_time[6]
   - start_service[6] + start_service[19] >= -999957.385467226
 StartServiceRobots[6,20]: - 999999 v[6,20] - service_time[6]
   - start_service[6] + start_service[20] >= -999923.9891144931
 StartServiceRobots[6,21]: - 999999 v[6,21] - service_time[6]
   - start_service[6] + start_service[21] >= -999924.7300771383
 StartServiceRobots[6,22]: - 999999 v[6,22] - service_time[6]
   - start_service[6] + start_service[22] >= -999936.1484321967
 StartServiceRobots[6,23]: - 999999 v[6,23] - service_time[6]
   - start_service[6] + start_service[23] >= -999936.1484321967
 StartServiceRobots[7,1]: - 999999 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999941.126196999
 StartServiceRobots[7,2]: - 999999 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999933.5494026708
 StartServiceRobots[7,3]: - 999999 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999972.6248534749
 StartServiceRobots[7,4]: - 999999 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999988.8635851692
 StartServiceRobots[7,5]: - 999999 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999891.227834211
 StartServiceRobots[7,6]: - 999999 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999882.642633686
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 999999 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999952.7111189339
 StartServiceRobots[7,9]: - 999999 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999913.555315655
 StartServiceRobots[7,10]: - 999999 v[7,10] - service_time[7]
   - start_service[7] + start_service[10] >= -999923.0335188602
 StartServiceRobots[7,11]: - 999999 v[7,11] - service_time[7]
   - start_service[7] + start_service[11] >= -999943.1581189876
 StartServiceRobots[7,12]: - 999999 v[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999933.5494026708
 StartServiceRobots[7,13]: - 999999 v[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999972.6248534749
 StartServiceRobots[7,14]: - 999999 v[7,14] - service_time[7]
   - start_service[7] + start_service[14] >= -999988.8635851692
 StartServiceRobots[7,15]: - 999999 v[7,15] - service_time[7]
   - start_service[7] + start_service[15] >= -999891.227834211
 StartServiceRobots[7,16]: - 999999 v[7,16] - service_time[7]
   - start_service[7] + start_service[16] >= -999882.642633686
 StartServiceRobots[7,17]: - 999999 v[7,17] - service_time[7]
   - start_service[7] + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 999999 v[7,18] - service_time[7]
   - start_service[7] + start_service[18] >= -999952.7111189339
 StartServiceRobots[7,19]: - 999999 v[7,19] - service_time[7]
   - start_service[7] + start_service[19] >= -999913.555315655
 StartServiceRobots[7,20]: - 999999 v[7,20] - service_time[7]
   - start_service[7] + start_service[20] >= -999923.0335188602
 StartServiceRobots[7,21]: - 999999 v[7,21] - service_time[7]
   - start_service[7] + start_service[21] >= -999943.1581189876
 StartServiceRobots[7,22]: - 999999 v[7,22] - service_time[7]
   - start_service[7] + start_service[22] >= -999941.126196999
 StartServiceRobots[7,23]: - 999999 v[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= -999941.126196999
 StartServiceRobots[8,1]: - 999999 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999951.7620508042
 StartServiceRobots[8,2]: - 999999 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999979.7861738935
 StartServiceRobots[8,3]: - 999999 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999973.4499059909
 StartServiceRobots[8,4]: - 999999 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999957.9553616339
 StartServiceRobots[8,5]: - 999999 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999918.6764967154
 StartServiceRobots[8,6]: - 999999 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999915.2753088792
 StartServiceRobots[8,7]: - 999999 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999952.7111189339
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 999999 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999931.0123467562
 StartServiceRobots[8,10]: - 999999 v[8,10] - service_time[8]
   - start_service[8] + start_service[10] >= -999969.1712492406
 StartServiceRobots[8,11]: - 999999 v[8,11] - service_time[8]
   - start_service[8] + start_service[11] >= -999941.8023031916
 StartServiceRobots[8,12]: - 999999 v[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999979.7861738935
 StartServiceRobots[8,13]: - 999999 v[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999973.4499059909
 StartServiceRobots[8,14]: - 999999 v[8,14] - service_time[8]
   - start_service[8] + start_service[14] >= -999957.9553616339
 StartServiceRobots[8,15]: - 999999 v[8,15] - service_time[8]
   - start_service[8] + start_service[15] >= -999918.6764967154
 StartServiceRobots[8,16]: - 999999 v[8,16] - service_time[8]
   - start_service[8] + start_service[16] >= -999915.2753088792
 StartServiceRobots[8,17]: - 999999 v[8,17] - service_time[8]
   - start_service[8] + start_service[17] >= -999952.7111189339
 StartServiceRobots[8,18]: - 999999 v[8,18] - service_time[8]
   - start_service[8] + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 999999 v[8,19] - service_time[8]
   - start_service[8] + start_service[19] >= -999931.0123467562
 StartServiceRobots[8,20]: - 999999 v[8,20] - service_time[8]
   - start_service[8] + start_service[20] >= -999969.1712492406
 StartServiceRobots[8,21]: - 999999 v[8,21] - service_time[8]
   - start_service[8] + start_service[21] >= -999941.8023031916
 StartServiceRobots[8,22]: - 999999 v[8,22] - service_time[8]
   - start_service[8] + start_service[22] >= -999951.7620508042
 StartServiceRobots[8,23]: - 999999 v[8,23] - service_time[8]
   - start_service[8] + start_service[23] >= -999951.7620508042
 StartServiceRobots[9,1]: - 999999 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999971.4290013848
 StartServiceRobots[9,2]: - 999999 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999931.6046053576
 StartServiceRobots[9,3]: - 999999 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999936.1977742163
 StartServiceRobots[9,4]: - 999999 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999910.2653522416
 StartServiceRobots[9,5]: - 999999 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999971.5729838425
 StartServiceRobots[9,6]: - 999999 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999957.385467226
 StartServiceRobots[9,7]: - 999999 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999913.555315655
 StartServiceRobots[9,8]: - 999999 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999931.0123467562
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 999999 v[9,10] - service_time[9]
   - start_service[9] + start_service[10] >= -999923.5677362148
 StartServiceRobots[9,11]: - 999999 v[9,11] - service_time[9]
   - start_service[9] + start_service[11] >= -999964.3876103099
 StartServiceRobots[9,12]: - 999999 v[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999931.6046053576
 StartServiceRobots[9,13]: - 999999 v[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999936.1977742163
 StartServiceRobots[9,14]: - 999999 v[9,14] - service_time[9]
   - start_service[9] + start_service[14] >= -999910.2653522416
 StartServiceRobots[9,15]: - 999999 v[9,15] - service_time[9]
   - start_service[9] + start_service[15] >= -999971.5729838425
 StartServiceRobots[9,16]: - 999999 v[9,16] - service_time[9]
   - start_service[9] + start_service[16] >= -999957.385467226
 StartServiceRobots[9,17]: - 999999 v[9,17] - service_time[9]
   - start_service[9] + start_service[17] >= -999913.555315655
 StartServiceRobots[9,18]: - 999999 v[9,18] - service_time[9]
   - start_service[9] + start_service[18] >= -999931.0123467562
 StartServiceRobots[9,19]: - 999999 v[9,19] - service_time[9]
   - start_service[9] + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 999999 v[9,20] - service_time[9]
   - start_service[9] + start_service[20] >= -999923.5677362148
 StartServiceRobots[9,21]: - 999999 v[9,21] - service_time[9]
   - start_service[9] + start_service[21] >= -999964.3876103099
 StartServiceRobots[9,22]: - 999999 v[9,22] - service_time[9]
   - start_service[9] + start_service[22] >= -999971.4290013848
 StartServiceRobots[9,23]: - 999999 v[9,23] - service_time[9]
   - start_service[9] + start_service[23] >= -999971.4290013848
 StartServiceRobots[10,1]: - 999999 v[10,1] - service_time[10]
   + start_service[1] - start_service[10] >= -999934.7445159944
 StartServiceRobots[10,2]: - 999999 v[10,2] - service_time[10]
   + start_service[2] - start_service[10] >= -999987.1892001542
 StartServiceRobots[10,3]: - 999999 v[10,3] - service_time[10]
   + start_service[3] - start_service[10] >= -999944.4584358049
 StartServiceRobots[10,4]: - 999999 v[10,4] - service_time[10]
   + start_service[4] - start_service[10] >= -999929.0561937442
 StartServiceRobots[10,5]: - 999999 v[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999921.6996578324
 StartServiceRobots[10,6]: - 999999 v[10,6] - service_time[10]
   + start_service[6] - start_service[10] >= -999923.9891144931
 StartServiceRobots[10,7]: - 999999 v[10,7] - service_time[10]
   + start_service[7] - start_service[10] >= -999923.0335188602
 StartServiceRobots[10,8]: - 999999 v[10,8] - service_time[10]
   + start_service[8] - start_service[10] >= -999969.1712492406
 StartServiceRobots[10,9]: - 999999 v[10,9] - service_time[10]
   + start_service[9] - start_service[10] >= -999923.5677362148
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 999999 v[10,11] - service_time[10]
   - start_service[10] + start_service[11] >= -999921.3827498653
 StartServiceRobots[10,12]: - 999999 v[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999987.1892001542
 StartServiceRobots[10,13]: - 999999 v[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999944.4584358049
 StartServiceRobots[10,14]: - 999999 v[10,14] - service_time[10]
   - start_service[10] + start_service[14] >= -999929.0561937442
 StartServiceRobots[10,15]: - 999999 v[10,15] - service_time[10]
   - start_service[10] + start_service[15] >= -999921.6996578324
 StartServiceRobots[10,16]: - 999999 v[10,16] - service_time[10]
   - start_service[10] + start_service[16] >= -999923.9891144931
 StartServiceRobots[10,17]: - 999999 v[10,17] - service_time[10]
   - start_service[10] + start_service[17] >= -999923.0335188602
 StartServiceRobots[10,18]: - 999999 v[10,18] - service_time[10]
   - start_service[10] + start_service[18] >= -999969.1712492406
 StartServiceRobots[10,19]: - 999999 v[10,19] - service_time[10]
   - start_service[10] + start_service[19] >= -999923.5677362148
 StartServiceRobots[10,20]: - 999999 v[10,20] - service_time[10]
   - start_service[10] + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 999999 v[10,21] - service_time[10]
   - start_service[10] + start_service[21] >= -999921.3827498653
 StartServiceRobots[10,22]: - 999999 v[10,22] - service_time[10]
   - start_service[10] + start_service[22] >= -999934.7445159944
 StartServiceRobots[10,23]: - 999999 v[10,23] - service_time[10]
   - start_service[10] + start_service[23] >= -999934.7445159944
 StartServiceRobots[11,1]: - 999999 v[11,1] - service_time[11]
   + start_service[1] - start_service[11] >= -999984.1635755197
 StartServiceRobots[11,2]: - 999999 v[11,2] - service_time[11]
   + start_service[2] - start_service[11] >= -999932.5412105162
 StartServiceRobots[11,3]: - 999999 v[11,3] - service_time[11]
   + start_service[3] - start_service[11] >= -999959.1641715461
 StartServiceRobots[11,4]: - 999999 v[11,4] - service_time[11]
   + start_service[4] - start_service[11] >= -999937.3674259853
 StartServiceRobots[11,5]: - 999999 v[11,5] - service_time[11]
   + start_service[5] - start_service[11] >= -999937.5561410894
 StartServiceRobots[11,6]: - 999999 v[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999924.7300771383
 StartServiceRobots[11,7]: - 999999 v[11,7] - service_time[11]
   + start_service[7] - start_service[11] >= -999943.1581189876
 StartServiceRobots[11,8]: - 999999 v[11,8] - service_time[11]
   + start_service[8] - start_service[11] >= -999941.8023031916
 StartServiceRobots[11,9]: - 999999 v[11,9] - service_time[11]
   + start_service[9] - start_service[11] >= -999964.3876103099
 StartServiceRobots[11,10]: - 999999 v[11,10] - service_time[11]
   + start_service[10] - start_service[11] >= -999921.3827498653
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 999999 v[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999932.5412105162
 StartServiceRobots[11,13]: - 999999 v[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999959.1641715461
 StartServiceRobots[11,14]: - 999999 v[11,14] - service_time[11]
   - start_service[11] + start_service[14] >= -999937.3674259853
 StartServiceRobots[11,15]: - 999999 v[11,15] - service_time[11]
   - start_service[11] + start_service[15] >= -999937.5561410894
 StartServiceRobots[11,16]: - 999999 v[11,16] - service_time[11]
   - start_service[11] + start_service[16] >= -999924.7300771383
 StartServiceRobots[11,17]: - 999999 v[11,17] - service_time[11]
   - start_service[11] + start_service[17] >= -999943.1581189876
 StartServiceRobots[11,18]: - 999999 v[11,18] - service_time[11]
   - start_service[11] + start_service[18] >= -999941.8023031916
 StartServiceRobots[11,19]: - 999999 v[11,19] - service_time[11]
   - start_service[11] + start_service[19] >= -999964.3876103099
 StartServiceRobots[11,20]: - 999999 v[11,20] - service_time[11]
   - start_service[11] + start_service[20] >= -999921.3827498653
 StartServiceRobots[11,21]: - 999999 v[11,21] - service_time[11]
   - start_service[11] + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 999999 v[11,22] - service_time[11]
   - start_service[11] + start_service[22] >= -999984.1635755197
 StartServiceRobots[11,23]: - 999999 v[11,23] - service_time[11]
   - start_service[11] + start_service[23] >= -999984.1635755197
 StartServiceRobots[12,1]: - 999999 v[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999945.4287857366
 StartServiceRobots[12,2]: - 999999 v[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 999999 v[12,3] - service_time[12]
   + start_service[3] - start_service[12] >= -999955.9804138733
 StartServiceRobots[12,4]: - 999999 v[12,4] - service_time[12]
   + start_service[4] - start_service[12] >= -999938.8231751657
 StartServiceRobots[12,5]: - 999999 v[12,5] - service_time[12]
   + start_service[5] - start_service[12] >= -999926.0251344698
 StartServiceRobots[12,6]: - 999999 v[12,6] - service_time[12]
   + start_service[6] - start_service[12] >= -999925.9721696612
 StartServiceRobots[12,7]: - 999999 v[12,7] - service_time[12]
   + start_service[7] - start_service[12] >= -999933.5494026708
 StartServiceRobots[12,8]: - 999999 v[12,8] - service_time[12]
   + start_service[8] - start_service[12] >= -999979.7861738935
 StartServiceRobots[12,9]: - 999999 v[12,9] - service_time[12]
   + start_service[9] - start_service[12] >= -999931.6046053576
 StartServiceRobots[12,10]: - 999999 v[12,10] - service_time[12]
   + start_service[10] - start_service[12] >= -999987.1892001542
 StartServiceRobots[12,11]: - 999999 v[12,11] - service_time[12]
   + start_service[11] - start_service[12] >= -999932.5412105162
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 999999 v[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999955.9804138733
 StartServiceRobots[12,14]: - 999999 v[12,14] - service_time[12]
   - start_service[12] + start_service[14] >= -999938.8231751657
 StartServiceRobots[12,15]: - 999999 v[12,15] - service_time[12]
   - start_service[12] + start_service[15] >= -999926.0251344698
 StartServiceRobots[12,16]: - 999999 v[12,16] - service_time[12]
   - start_service[12] + start_service[16] >= -999925.9721696612
 StartServiceRobots[12,17]: - 999999 v[12,17] - service_time[12]
   - start_service[12] + start_service[17] >= -999933.5494026708
 StartServiceRobots[12,18]: - 999999 v[12,18] - service_time[12]
   - start_service[12] + start_service[18] >= -999979.7861738935
 StartServiceRobots[12,19]: - 999999 v[12,19] - service_time[12]
   - start_service[12] + start_service[19] >= -999931.6046053576
 StartServiceRobots[12,20]: - 999999 v[12,20] - service_time[12]
   - start_service[12] + start_service[20] >= -999987.1892001542
 StartServiceRobots[12,21]: - 999999 v[12,21] - service_time[12]
   - start_service[12] + start_service[21] >= -999932.5412105162
 StartServiceRobots[12,22]: - 999999 v[12,22] - service_time[12]
   - start_service[12] + start_service[22] >= -999945.4287857366
 StartServiceRobots[12,23]: - 999999 v[12,23] - service_time[12]
   - start_service[12] + start_service[23] >= -999945.4287857366
 StartServiceRobots[13,1]: - 999999 v[13,1] - service_time[13]
   + start_service[1] - start_service[13] >= -999962.9193004568
 StartServiceRobots[13,2]: - 999999 v[13,2] - service_time[13]
   + start_service[2] - start_service[13] >= -999955.9804138733
 StartServiceRobots[13,3]: - 999999 v[13,3] - service_time[13]
   + start_service[3] - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 999999 v[13,4] - service_time[13]
   + start_service[4] - start_service[13] >= -999972.474248387
 StartServiceRobots[13,5]: - 999999 v[13,5] - service_time[13]
   + start_service[5] - start_service[13] >= -999916.1872075291
 StartServiceRobots[13,6]: - 999999 v[13,6] - service_time[13]
   + start_service[6] - start_service[13] >= -999908.5111139182
 StartServiceRobots[13,7]: - 999999 v[13,7] - service_time[13]
   + start_service[7] - start_service[13] >= -999972.6248534749
 StartServiceRobots[13,8]: - 999999 v[13,8] - service_time[13]
   + start_service[8] - start_service[13] >= -999973.4499059909
 StartServiceRobots[13,9]: - 999999 v[13,9] - service_time[13]
   + start_service[9] - start_service[13] >= -999936.1977742163
 StartServiceRobots[13,10]: - 999999 v[13,10] - service_time[13]
   + start_service[10] - start_service[13] >= -999944.4584358049
 StartServiceRobots[13,11]: - 999999 v[13,11] - service_time[13]
   + start_service[11] - start_service[13] >= -999959.1641715461
 StartServiceRobots[13,12]: - 999999 v[13,12] - service_time[13]
   + start_service[12] - start_service[13] >= -999955.9804138733
 StartServiceRobots[13,13]: - 999999 v[13,13] - service_time[13]
   >= -999999
 StartServiceRobots[13,14]: - 999999 v[13,14] - service_time[13]
   - start_service[13] + start_service[14] >= -999972.474248387
 StartServiceRobots[13,15]: - 999999 v[13,15] - service_time[13]
   - start_service[13] + start_service[15] >= -999916.1872075291
 StartServiceRobots[13,16]: - 999999 v[13,16] - service_time[13]
   - start_service[13] + start_service[16] >= -999908.5111139182
 StartServiceRobots[13,17]: - 999999 v[13,17] - service_time[13]
   - start_service[13] + start_service[17] >= -999972.6248534749
 StartServiceRobots[13,18]: - 999999 v[13,18] - service_time[13]
   - start_service[13] + start_service[18] >= -999973.4499059909
 StartServiceRobots[13,19]: - 999999 v[13,19] - service_time[13]
   - start_service[13] + start_service[19] >= -999936.1977742163
 StartServiceRobots[13,20]: - 999999 v[13,20] - service_time[13]
   - start_service[13] + start_service[20] >= -999944.4584358049
 StartServiceRobots[13,21]: - 999999 v[13,21] - service_time[13]
   - start_service[13] + start_service[21] >= -999959.1641715461
 StartServiceRobots[13,22]: - 999999 v[13,22] - service_time[13]
   - start_service[13] + start_service[22] >= -999962.9193004568
 StartServiceRobots[13,23]: - 999999 v[13,23] - service_time[13]
   - start_service[13] + start_service[23] >= -999962.9193004568
 StartServiceRobots[14,1]: - 999999 v[14,1] - service_time[14]
   + start_service[1] - start_service[14] >= -999937.583726213
 StartServiceRobots[14,2]: - 999999 v[14,2] - service_time[14]
   + start_service[2] - start_service[14] >= -999938.8231751657
 StartServiceRobots[14,3]: - 999999 v[14,3] - service_time[14]
   + start_service[3] - start_service[14] >= -999972.474248387
 StartServiceRobots[14,4]: - 999999 v[14,4] - service_time[14]
   + start_service[4] - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 999999 v[14,5] - service_time[14]
   + start_service[5] - start_service[14] >= -999889.662152088
 StartServiceRobots[14,6]: - 999999 v[14,6] - service_time[14]
   + start_service[6] - start_service[14] >= -999882.2562239703
 StartServiceRobots[14,7]: - 999999 v[14,7] - service_time[14]
   + start_service[7] - start_service[14] >= -999988.8635851692
 StartServiceRobots[14,8]: - 999999 v[14,8] - service_time[14]
   + start_service[8] - start_service[14] >= -999957.9553616339
 StartServiceRobots[14,9]: - 999999 v[14,9] - service_time[14]
   + start_service[9] - start_service[14] >= -999910.2653522416
 StartServiceRobots[14,10]: - 999999 v[14,10] - service_time[14]
   + start_service[10] - start_service[14] >= -999929.0561937442
 StartServiceRobots[14,11]: - 999999 v[14,11] - service_time[14]
   + start_service[11] - start_service[14] >= -999937.3674259853
 StartServiceRobots[14,12]: - 999999 v[14,12] - service_time[14]
   + start_service[12] - start_service[14] >= -999938.8231751657
 StartServiceRobots[14,13]: - 999999 v[14,13] - service_time[14]
   + start_service[13] - start_service[14] >= -999972.474248387
 StartServiceRobots[14,14]: - 999999 v[14,14] - service_time[14]
   >= -999999
 StartServiceRobots[14,15]: - 999999 v[14,15] - service_time[14]
   - start_service[14] + start_service[15] >= -999889.662152088
 StartServiceRobots[14,16]: - 999999 v[14,16] - service_time[14]
   - start_service[14] + start_service[16] >= -999882.2562239703
 StartServiceRobots[14,17]: - 999999 v[14,17] - service_time[14]
   - start_service[14] + start_service[17] >= -999988.8635851692
 StartServiceRobots[14,18]: - 999999 v[14,18] - service_time[14]
   - start_service[14] + start_service[18] >= -999957.9553616339
 StartServiceRobots[14,19]: - 999999 v[14,19] - service_time[14]
   - start_service[14] + start_service[19] >= -999910.2653522416
 StartServiceRobots[14,20]: - 999999 v[14,20] - service_time[14]
   - start_service[14] + start_service[20] >= -999929.0561937442
 StartServiceRobots[14,21]: - 999999 v[14,21] - service_time[14]
   - start_service[14] + start_service[21] >= -999937.3674259853
 StartServiceRobots[14,22]: - 999999 v[14,22] - service_time[14]
   - start_service[14] + start_service[22] >= -999937.583726213
 StartServiceRobots[14,23]: - 999999 v[14,23] - service_time[14]
   - start_service[14] + start_service[23] >= -999937.583726213
 StartServiceRobots[15,1]: - 999999 v[15,1] - service_time[15]
   + start_service[1] - start_service[15] >= -999947.3853854308
 StartServiceRobots[15,2]: - 999999 v[15,2] - service_time[15]
   + start_service[2] - start_service[15] >= -999926.0251344698
 StartServiceRobots[15,3]: - 999999 v[15,3] - service_time[15]
   + start_service[3] - start_service[15] >= -999916.1872075291
 StartServiceRobots[15,4]: - 999999 v[15,4] - service_time[15]
   + start_service[4] - start_service[15] >= -999889.662152088
 StartServiceRobots[15,5]: - 999999 v[15,5] - service_time[15]
   + start_service[5] - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 999999 v[15,6] - service_time[15]
   + start_service[6] - start_service[15] >= -999983.6081284663
 StartServiceRobots[15,7]: - 999999 v[15,7] - service_time[15]
   + start_service[7] - start_service[15] >= -999891.227834211
 StartServiceRobots[15,8]: - 999999 v[15,8] - service_time[15]
   + start_service[8] - start_service[15] >= -999918.6764967154
 StartServiceRobots[15,9]: - 999999 v[15,9] - service_time[15]
   + start_service[9] - start_service[15] >= -999971.5729838425
 StartServiceRobots[15,10]: - 999999 v[15,10] - service_time[15]
   + start_service[10] - start_service[15] >= -999921.6996578324
 StartServiceRobots[15,11]: - 999999 v[15,11] - service_time[15]
   + start_service[11] - start_service[15] >= -999937.5561410894
 StartServiceRobots[15,12]: - 999999 v[15,12] - service_time[15]
   + start_service[12] - start_service[15] >= -999926.0251344698
 StartServiceRobots[15,13]: - 999999 v[15,13] - service_time[15]
   + start_service[13] - start_service[15] >= -999916.1872075291
 StartServiceRobots[15,14]: - 999999 v[15,14] - service_time[15]
   + start_service[14] - start_service[15] >= -999889.662152088
 StartServiceRobots[15,15]: - 999999 v[15,15] - service_time[15]
   >= -999999
 StartServiceRobots[15,16]: - 999999 v[15,16] - service_time[15]
   - start_service[15] + start_service[16] >= -999983.6081284663
 StartServiceRobots[15,17]: - 999999 v[15,17] - service_time[15]
   - start_service[15] + start_service[17] >= -999891.227834211
 StartServiceRobots[15,18]: - 999999 v[15,18] - service_time[15]
   - start_service[15] + start_service[18] >= -999918.6764967154
 StartServiceRobots[15,19]: - 999999 v[15,19] - service_time[15]
   - start_service[15] + start_service[19] >= -999971.5729838425
 StartServiceRobots[15,20]: - 999999 v[15,20] - service_time[15]
   - start_service[15] + start_service[20] >= -999921.6996578324
 StartServiceRobots[15,21]: - 999999 v[15,21] - service_time[15]
   - start_service[15] + start_service[21] >= -999937.5561410894
 StartServiceRobots[15,22]: - 999999 v[15,22] - service_time[15]
   - start_service[15] + start_service[22] >= -999947.3853854308
 StartServiceRobots[15,23]: - 999999 v[15,23] - service_time[15]
   - start_service[15] + start_service[23] >= -999947.3853854308
 StartServiceRobots[16,1]: - 999999 v[16,1] - service_time[16]
   + start_service[1] - start_service[16] >= -999936.1484321967
 StartServiceRobots[16,2]: - 999999 v[16,2] - service_time[16]
   + start_service[2] - start_service[16] >= -999925.9721696612
 StartServiceRobots[16,3]: - 999999 v[16,3] - service_time[16]
   + start_service[3] - start_service[16] >= -999908.5111139182
 StartServiceRobots[16,4]: - 999999 v[16,4] - service_time[16]
   + start_service[4] - start_service[16] >= -999882.2562239703
 StartServiceRobots[16,5]: - 999999 v[16,5] - service_time[16]
   + start_service[5] - start_service[16] >= -999983.6081284663
 StartServiceRobots[16,6]: - 999999 v[16,6] - service_time[16]
   + start_service[6] - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 999999 v[16,7] - service_time[16]
   + start_service[7] - start_service[16] >= -999882.642633686
 StartServiceRobots[16,8]: - 999999 v[16,8] - service_time[16]
   + start_service[8] - start_service[16] >= -999915.2753088792
 StartServiceRobots[16,9]: - 999999 v[16,9] - service_time[16]
   + start_service[9] - start_service[16] >= -999957.385467226
 StartServiceRobots[16,10]: - 999999 v[16,10] - service_time[16]
   + start_service[10] - start_service[16] >= -999923.9891144931
 StartServiceRobots[16,11]: - 999999 v[16,11] - service_time[16]
   + start_service[11] - start_service[16] >= -999924.7300771383
 StartServiceRobots[16,12]: - 999999 v[16,12] - service_time[16]
   + start_service[12] - start_service[16] >= -999925.9721696612
 StartServiceRobots[16,13]: - 999999 v[16,13] - service_time[16]
   + start_service[13] - start_service[16] >= -999908.5111139182
 StartServiceRobots[16,14]: - 999999 v[16,14] - service_time[16]
   + start_service[14] - start_service[16] >= -999882.2562239703
 StartServiceRobots[16,15]: - 999999 v[16,15] - service_time[16]
   + start_service[15] - start_service[16] >= -999983.6081284663
 StartServiceRobots[16,16]: - 999999 v[16,16] - service_time[16]
   >= -999999
 StartServiceRobots[16,17]: - 999999 v[16,17] - service_time[16]
   - start_service[16] + start_service[17] >= -999882.642633686
 StartServiceRobots[16,18]: - 999999 v[16,18] - service_time[16]
   - start_service[16] + start_service[18] >= -999915.2753088792
 StartServiceRobots[16,19]: - 999999 v[16,19] - service_time[16]
   - start_service[16] + start_service[19] >= -999957.385467226
 StartServiceRobots[16,20]: - 999999 v[16,20] - service_time[16]
   - start_service[16] + start_service[20] >= -999923.9891144931
 StartServiceRobots[16,21]: - 999999 v[16,21] - service_time[16]
   - start_service[16] + start_service[21] >= -999924.7300771383
 StartServiceRobots[16,22]: - 999999 v[16,22] - service_time[16]
   - start_service[16] + start_service[22] >= -999936.1484321967
 StartServiceRobots[16,23]: - 999999 v[16,23] - service_time[16]
   - start_service[16] + start_service[23] >= -999936.1484321967
 StartServiceRobots[17,1]: - 999999 v[17,1] - service_time[17]
   + start_service[1] - start_service[17] >= -999941.126196999
 StartServiceRobots[17,2]: - 999999 v[17,2] - service_time[17]
   + start_service[2] - start_service[17] >= -999933.5494026708
 StartServiceRobots[17,3]: - 999999 v[17,3] - service_time[17]
   + start_service[3] - start_service[17] >= -999972.6248534749
 StartServiceRobots[17,4]: - 999999 v[17,4] - service_time[17]
   + start_service[4] - start_service[17] >= -999988.8635851692
 StartServiceRobots[17,5]: - 999999 v[17,5] - service_time[17]
   + start_service[5] - start_service[17] >= -999891.227834211
 StartServiceRobots[17,6]: - 999999 v[17,6] - service_time[17]
   + start_service[6] - start_service[17] >= -999882.642633686
 StartServiceRobots[17,7]: - 999999 v[17,7] - service_time[17]
   + start_service[7] - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 999999 v[17,8] - service_time[17]
   + start_service[8] - start_service[17] >= -999952.7111189339
 StartServiceRobots[17,9]: - 999999 v[17,9] - service_time[17]
   + start_service[9] - start_service[17] >= -999913.555315655
 StartServiceRobots[17,10]: - 999999 v[17,10] - service_time[17]
   + start_service[10] - start_service[17] >= -999923.0335188602
 StartServiceRobots[17,11]: - 999999 v[17,11] - service_time[17]
   + start_service[11] - start_service[17] >= -999943.1581189876
 StartServiceRobots[17,12]: - 999999 v[17,12] - service_time[17]
   + start_service[12] - start_service[17] >= -999933.5494026708
 StartServiceRobots[17,13]: - 999999 v[17,13] - service_time[17]
   + start_service[13] - start_service[17] >= -999972.6248534749
 StartServiceRobots[17,14]: - 999999 v[17,14] - service_time[17]
   + start_service[14] - start_service[17] >= -999988.8635851692
 StartServiceRobots[17,15]: - 999999 v[17,15] - service_time[17]
   + start_service[15] - start_service[17] >= -999891.227834211
 StartServiceRobots[17,16]: - 999999 v[17,16] - service_time[17]
   + start_service[16] - start_service[17] >= -999882.642633686
 StartServiceRobots[17,17]: - 999999 v[17,17] - service_time[17]
   >= -999999
 StartServiceRobots[17,18]: - 999999 v[17,18] - service_time[17]
   - start_service[17] + start_service[18] >= -999952.7111189339
 StartServiceRobots[17,19]: - 999999 v[17,19] - service_time[17]
   - start_service[17] + start_service[19] >= -999913.555315655
 StartServiceRobots[17,20]: - 999999 v[17,20] - service_time[17]
   - start_service[17] + start_service[20] >= -999923.0335188602
 StartServiceRobots[17,21]: - 999999 v[17,21] - service_time[17]
   - start_service[17] + start_service[21] >= -999943.1581189876
 StartServiceRobots[17,22]: - 999999 v[17,22] - service_time[17]
   - start_service[17] + start_service[22] >= -999941.126196999
 StartServiceRobots[17,23]: - 999999 v[17,23] - service_time[17]
   - start_service[17] + start_service[23] >= -999941.126196999
 StartServiceRobots[18,1]: - 999999 v[18,1] - service_time[18]
   + start_service[1] - start_service[18] >= -999951.7620508042
 StartServiceRobots[18,2]: - 999999 v[18,2] - service_time[18]
   + start_service[2] - start_service[18] >= -999979.7861738935
 StartServiceRobots[18,3]: - 999999 v[18,3] - service_time[18]
   + start_service[3] - start_service[18] >= -999973.4499059909
 StartServiceRobots[18,4]: - 999999 v[18,4] - service_time[18]
   + start_service[4] - start_service[18] >= -999957.9553616339
 StartServiceRobots[18,5]: - 999999 v[18,5] - service_time[18]
   + start_service[5] - start_service[18] >= -999918.6764967154
 StartServiceRobots[18,6]: - 999999 v[18,6] - service_time[18]
   + start_service[6] - start_service[18] >= -999915.2753088792
 StartServiceRobots[18,7]: - 999999 v[18,7] - service_time[18]
   + start_service[7] - start_service[18] >= -999952.7111189339
 StartServiceRobots[18,8]: - 999999 v[18,8] - service_time[18]
   + start_service[8] - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 999999 v[18,9] - service_time[18]
   + start_service[9] - start_service[18] >= -999931.0123467562
 StartServiceRobots[18,10]: - 999999 v[18,10] - service_time[18]
   + start_service[10] - start_service[18] >= -999969.1712492406
 StartServiceRobots[18,11]: - 999999 v[18,11] - service_time[18]
   + start_service[11] - start_service[18] >= -999941.8023031916
 StartServiceRobots[18,12]: - 999999 v[18,12] - service_time[18]
   + start_service[12] - start_service[18] >= -999979.7861738935
 StartServiceRobots[18,13]: - 999999 v[18,13] - service_time[18]
   + start_service[13] - start_service[18] >= -999973.4499059909
 StartServiceRobots[18,14]: - 999999 v[18,14] - service_time[18]
   + start_service[14] - start_service[18] >= -999957.9553616339
 StartServiceRobots[18,15]: - 999999 v[18,15] - service_time[18]
   + start_service[15] - start_service[18] >= -999918.6764967154
 StartServiceRobots[18,16]: - 999999 v[18,16] - service_time[18]
   + start_service[16] - start_service[18] >= -999915.2753088792
 StartServiceRobots[18,17]: - 999999 v[18,17] - service_time[18]
   + start_service[17] - start_service[18] >= -999952.7111189339
 StartServiceRobots[18,18]: - 999999 v[18,18] - service_time[18]
   >= -999999
 StartServiceRobots[18,19]: - 999999 v[18,19] - service_time[18]
   - start_service[18] + start_service[19] >= -999931.0123467562
 StartServiceRobots[18,20]: - 999999 v[18,20] - service_time[18]
   - start_service[18] + start_service[20] >= -999969.1712492406
 StartServiceRobots[18,21]: - 999999 v[18,21] - service_time[18]
   - start_service[18] + start_service[21] >= -999941.8023031916
 StartServiceRobots[18,22]: - 999999 v[18,22] - service_time[18]
   - start_service[18] + start_service[22] >= -999951.7620508042
 StartServiceRobots[18,23]: - 999999 v[18,23] - service_time[18]
   - start_service[18] + start_service[23] >= -999951.7620508042
 StartServiceRobots[19,1]: - 999999 v[19,1] - service_time[19]
   + start_service[1] - start_service[19] >= -999971.4290013848
 StartServiceRobots[19,2]: - 999999 v[19,2] - service_time[19]
   + start_service[2] - start_service[19] >= -999931.6046053576
 StartServiceRobots[19,3]: - 999999 v[19,3] - service_time[19]
   + start_service[3] - start_service[19] >= -999936.1977742163
 StartServiceRobots[19,4]: - 999999 v[19,4] - service_time[19]
   + start_service[4] - start_service[19] >= -999910.2653522416
 StartServiceRobots[19,5]: - 999999 v[19,5] - service_time[19]
   + start_service[5] - start_service[19] >= -999971.5729838425
 StartServiceRobots[19,6]: - 999999 v[19,6] - service_time[19]
   + start_service[6] - start_service[19] >= -999957.385467226
 StartServiceRobots[19,7]: - 999999 v[19,7] - service_time[19]
   + start_service[7] - start_service[19] >= -999913.555315655
 StartServiceRobots[19,8]: - 999999 v[19,8] - service_time[19]
   + start_service[8] - start_service[19] >= -999931.0123467562
 StartServiceRobots[19,9]: - 999999 v[19,9] - service_time[19]
   + start_service[9] - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 999999 v[19,10] - service_time[19]
   + start_service[10] - start_service[19] >= -999923.5677362148
 StartServiceRobots[19,11]: - 999999 v[19,11] - service_time[19]
   + start_service[11] - start_service[19] >= -999964.3876103099
 StartServiceRobots[19,12]: - 999999 v[19,12] - service_time[19]
   + start_service[12] - start_service[19] >= -999931.6046053576
 StartServiceRobots[19,13]: - 999999 v[19,13] - service_time[19]
   + start_service[13] - start_service[19] >= -999936.1977742163
 StartServiceRobots[19,14]: - 999999 v[19,14] - service_time[19]
   + start_service[14] - start_service[19] >= -999910.2653522416
 StartServiceRobots[19,15]: - 999999 v[19,15] - service_time[19]
   + start_service[15] - start_service[19] >= -999971.5729838425
 StartServiceRobots[19,16]: - 999999 v[19,16] - service_time[19]
   + start_service[16] - start_service[19] >= -999957.385467226
 StartServiceRobots[19,17]: - 999999 v[19,17] - service_time[19]
   + start_service[17] - start_service[19] >= -999913.555315655
 StartServiceRobots[19,18]: - 999999 v[19,18] - service_time[19]
   + start_service[18] - start_service[19] >= -999931.0123467562
 StartServiceRobots[19,19]: - 999999 v[19,19] - service_time[19]
   >= -999999
 StartServiceRobots[19,20]: - 999999 v[19,20] - service_time[19]
   - start_service[19] + start_service[20] >= -999923.5677362148
 StartServiceRobots[19,21]: - 999999 v[19,21] - service_time[19]
   - start_service[19] + start_service[21] >= -999964.3876103099
 StartServiceRobots[19,22]: - 999999 v[19,22] - service_time[19]
   - start_service[19] + start_service[22] >= -999971.4290013848
 StartServiceRobots[19,23]: - 999999 v[19,23] - service_time[19]
   - start_service[19] + start_service[23] >= -999971.4290013848
 StartServiceRobots[20,1]: - 999999 v[20,1] - service_time[20]
   + start_service[1] - start_service[20] >= -999934.7445159944
 StartServiceRobots[20,2]: - 999999 v[20,2] - service_time[20]
   + start_service[2] - start_service[20] >= -999987.1892001542
 StartServiceRobots[20,3]: - 999999 v[20,3] - service_time[20]
   + start_service[3] - start_service[20] >= -999944.4584358049
 StartServiceRobots[20,4]: - 999999 v[20,4] - service_time[20]
   + start_service[4] - start_service[20] >= -999929.0561937442
 StartServiceRobots[20,5]: - 999999 v[20,5] - service_time[20]
   + start_service[5] - start_service[20] >= -999921.6996578324
 StartServiceRobots[20,6]: - 999999 v[20,6] - service_time[20]
   + start_service[6] - start_service[20] >= -999923.9891144931
 StartServiceRobots[20,7]: - 999999 v[20,7] - service_time[20]
   + start_service[7] - start_service[20] >= -999923.0335188602
 StartServiceRobots[20,8]: - 999999 v[20,8] - service_time[20]
   + start_service[8] - start_service[20] >= -999969.1712492406
 StartServiceRobots[20,9]: - 999999 v[20,9] - service_time[20]
   + start_service[9] - start_service[20] >= -999923.5677362148
 StartServiceRobots[20,10]: - 999999 v[20,10] - service_time[20]
   + start_service[10] - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 999999 v[20,11] - service_time[20]
   + start_service[11] - start_service[20] >= -999921.3827498653
 StartServiceRobots[20,12]: - 999999 v[20,12] - service_time[20]
   + start_service[12] - start_service[20] >= -999987.1892001542
 StartServiceRobots[20,13]: - 999999 v[20,13] - service_time[20]
   + start_service[13] - start_service[20] >= -999944.4584358049
 StartServiceRobots[20,14]: - 999999 v[20,14] - service_time[20]
   + start_service[14] - start_service[20] >= -999929.0561937442
 StartServiceRobots[20,15]: - 999999 v[20,15] - service_time[20]
   + start_service[15] - start_service[20] >= -999921.6996578324
 StartServiceRobots[20,16]: - 999999 v[20,16] - service_time[20]
   + start_service[16] - start_service[20] >= -999923.9891144931
 StartServiceRobots[20,17]: - 999999 v[20,17] - service_time[20]
   + start_service[17] - start_service[20] >= -999923.0335188602
 StartServiceRobots[20,18]: - 999999 v[20,18] - service_time[20]
   + start_service[18] - start_service[20] >= -999969.1712492406
 StartServiceRobots[20,19]: - 999999 v[20,19] - service_time[20]
   + start_service[19] - start_service[20] >= -999923.5677362148
 StartServiceRobots[20,20]: - 999999 v[20,20] - service_time[20]
   >= -999999
 StartServiceRobots[20,21]: - 999999 v[20,21] - service_time[20]
   - start_service[20] + start_service[21] >= -999921.3827498653
 StartServiceRobots[20,22]: - 999999 v[20,22] - service_time[20]
   - start_service[20] + start_service[22] >= -999934.7445159944
 StartServiceRobots[20,23]: - 999999 v[20,23] - service_time[20]
   - start_service[20] + start_service[23] >= -999934.7445159944
 StartServiceRobots[21,1]: - 999999 v[21,1] - service_time[21]
   + start_service[1] - start_service[21] >= -999984.1635755197
 StartServiceRobots[21,2]: - 999999 v[21,2] - service_time[21]
   + start_service[2] - start_service[21] >= -999932.5412105162
 StartServiceRobots[21,3]: - 999999 v[21,3] - service_time[21]
   + start_service[3] - start_service[21] >= -999959.1641715461
 StartServiceRobots[21,4]: - 999999 v[21,4] - service_time[21]
   + start_service[4] - start_service[21] >= -999937.3674259853
 StartServiceRobots[21,5]: - 999999 v[21,5] - service_time[21]
   + start_service[5] - start_service[21] >= -999937.5561410894
 StartServiceRobots[21,6]: - 999999 v[21,6] - service_time[21]
   + start_service[6] - start_service[21] >= -999924.7300771383
 StartServiceRobots[21,7]: - 999999 v[21,7] - service_time[21]
   + start_service[7] - start_service[21] >= -999943.1581189876
 StartServiceRobots[21,8]: - 999999 v[21,8] - service_time[21]
   + start_service[8] - start_service[21] >= -999941.8023031916
 StartServiceRobots[21,9]: - 999999 v[21,9] - service_time[21]
   + start_service[9] - start_service[21] >= -999964.3876103099
 StartServiceRobots[21,10]: - 999999 v[21,10] - service_time[21]
   + start_service[10] - start_service[21] >= -999921.3827498653
 StartServiceRobots[21,11]: - 999999 v[21,11] - service_time[21]
   + start_service[11] - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 999999 v[21,12] - service_time[21]
   + start_service[12] - start_service[21] >= -999932.5412105162
 StartServiceRobots[21,13]: - 999999 v[21,13] - service_time[21]
   + start_service[13] - start_service[21] >= -999959.1641715461
 StartServiceRobots[21,14]: - 999999 v[21,14] - service_time[21]
   + start_service[14] - start_service[21] >= -999937.3674259853
 StartServiceRobots[21,15]: - 999999 v[21,15] - service_time[21]
   + start_service[15] - start_service[21] >= -999937.5561410894
 StartServiceRobots[21,16]: - 999999 v[21,16] - service_time[21]
   + start_service[16] - start_service[21] >= -999924.7300771383
 StartServiceRobots[21,17]: - 999999 v[21,17] - service_time[21]
   + start_service[17] - start_service[21] >= -999943.1581189876
 StartServiceRobots[21,18]: - 999999 v[21,18] - service_time[21]
   + start_service[18] - start_service[21] >= -999941.8023031916
 StartServiceRobots[21,19]: - 999999 v[21,19] - service_time[21]
   + start_service[19] - start_service[21] >= -999964.3876103099
 StartServiceRobots[21,20]: - 999999 v[21,20] - service_time[21]
   + start_service[20] - start_service[21] >= -999921.3827498653
 StartServiceRobots[21,21]: - 999999 v[21,21] - service_time[21]
   >= -999999
 StartServiceRobots[21,22]: - 999999 v[21,22] - service_time[21]
   - start_service[21] + start_service[22] >= -999984.1635755197
 StartServiceRobots[21,23]: - 999999 v[21,23] - service_time[21]
   - start_service[21] + start_service[23] >= -999984.1635755197
 StartServiceRobots[22,1]: - 999999 v[22,1] - service_time[22]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 999999 v[22,2] - service_time[22]
   + start_service[2] - start_service[22] >= -999945.4287857366
 StartServiceRobots[22,3]: - 999999 v[22,3] - service_time[22]
   + start_service[3] - start_service[22] >= -999962.9193004568
 StartServiceRobots[22,4]: - 999999 v[22,4] - service_time[22]
   + start_service[4] - start_service[22] >= -999937.583726213
 StartServiceRobots[22,5]: - 999999 v[22,5] - service_time[22]
   + start_service[5] - start_service[22] >= -999947.3853854308
 StartServiceRobots[22,6]: - 999999 v[22,6] - service_time[22]
   + start_service[6] - start_service[22] >= -999936.1484321967
 StartServiceRobots[22,7]: - 999999 v[22,7] - service_time[22]
   + start_service[7] - start_service[22] >= -999941.126196999
 StartServiceRobots[22,8]: - 999999 v[22,8] - service_time[22]
   + start_service[8] - start_service[22] >= -999951.7620508042
 StartServiceRobots[22,9]: - 999999 v[22,9] - service_time[22]
   + start_service[9] - start_service[22] >= -999971.4290013848
 StartServiceRobots[22,10]: - 999999 v[22,10] - service_time[22]
   + start_service[10] - start_service[22] >= -999934.7445159944
 StartServiceRobots[22,11]: - 999999 v[22,11] - service_time[22]
   + start_service[11] - start_service[22] >= -999984.1635755197
 StartServiceRobots[22,12]: - 999999 v[22,12] - service_time[22]
   + start_service[12] - start_service[22] >= -999945.4287857366
 StartServiceRobots[22,13]: - 999999 v[22,13] - service_time[22]
   + start_service[13] - start_service[22] >= -999962.9193004568
 StartServiceRobots[22,14]: - 999999 v[22,14] - service_time[22]
   + start_service[14] - start_service[22] >= -999937.583726213
 StartServiceRobots[22,15]: - 999999 v[22,15] - service_time[22]
   + start_service[15] - start_service[22] >= -999947.3853854308
 StartServiceRobots[22,16]: - 999999 v[22,16] - service_time[22]
   + start_service[16] - start_service[22] >= -999936.1484321967
 StartServiceRobots[22,17]: - 999999 v[22,17] - service_time[22]
   + start_service[17] - start_service[22] >= -999941.126196999
 StartServiceRobots[22,18]: - 999999 v[22,18] - service_time[22]
   + start_service[18] - start_service[22] >= -999951.7620508042
 StartServiceRobots[22,19]: - 999999 v[22,19] - service_time[22]
   + start_service[19] - start_service[22] >= -999971.4290013848
 StartServiceRobots[22,20]: - 999999 v[22,20] - service_time[22]
   + start_service[20] - start_service[22] >= -999934.7445159944
 StartServiceRobots[22,21]: - 999999 v[22,21] - service_time[22]
   + start_service[21] - start_service[22] >= -999984.1635755197
 StartServiceRobots[22,22]: - 999999 v[22,22] - service_time[22]
   >= -999999
 StartServiceRobots[22,23]: - 999999 v[22,23] - service_time[22]
   - start_service[22] + start_service[23] >= -999999
 RobotsEndService[2]: - service_time[2] - start_service[2]
   + start_service[12] >= 0
 RobotsEndService[3]: - service_time[3] - start_service[3]
   + start_service[13] >= 0
 RobotsEndService[4]: - service_time[4] - start_service[4]
   + start_service[14] >= 0
 RobotsEndService[5]: - service_time[5] - start_service[5]
   + start_service[15] >= 0
 RobotsEndService[6]: - service_time[6] - start_service[6]
   + start_service[16] >= 0
 RobotsEndService[7]: - service_time[7] - start_service[7]
   + start_service[17] >= 0
 RobotsEndService[8]: - service_time[8] - start_service[8]
   + start_service[18] >= 0
 RobotsEndService[9]: - service_time[9] - start_service[9]
   + start_service[19] >= 0
 RobotsEndService[10]: - service_time[10] - start_service[10]
   + start_service[20] >= 0
 RobotsEndService[11]: - service_time[11] - start_service[11]
   + start_service[21] >= 0
 VehicleRobotSynchronization1[0,1]: - 2 x[0,1] + w[0,1] <= 0
 VehicleRobotSynchronization1[0,2]: - 2 x[0,2] + w[0,2] <= 0
 VehicleRobotSynchronization1[0,3]: - 2 x[0,3] + w[0,3] <= 0
 VehicleRobotSynchronization1[0,4]: - 2 x[0,4] + w[0,4] <= 0
 VehicleRobotSynchronization1[0,5]: - 2 x[0,5] + w[0,5] <= 0
 VehicleRobotSynchronization1[0,6]: - 2 x[0,6] + w[0,6] <= 0
 VehicleRobotSynchronization1[0,7]: - 2 x[0,7] + w[0,7] <= 0
 VehicleRobotSynchronization1[0,8]: - 2 x[0,8] + w[0,8] <= 0
 VehicleRobotSynchronization1[0,9]: - 2 x[0,9] + w[0,9] <= 0
 VehicleRobotSynchronization1[0,10]: - 2 x[0,10] + w[0,10] <= 0
 VehicleRobotSynchronization1[0,11]: - 2 x[0,11] + w[0,11] <= 0
 VehicleRobotSynchronization1[0,12]: - 2 x[0,12] + w[0,12] <= 0
 VehicleRobotSynchronization1[0,13]: - 2 x[0,13] + w[0,13] <= 0
 VehicleRobotSynchronization1[0,14]: - 2 x[0,14] + w[0,14] <= 0
 VehicleRobotSynchronization1[0,15]: - 2 x[0,15] + w[0,15] <= 0
 VehicleRobotSynchronization1[0,16]: - 2 x[0,16] + w[0,16] <= 0
 VehicleRobotSynchronization1[0,17]: - 2 x[0,17] + w[0,17] <= 0
 VehicleRobotSynchronization1[0,18]: - 2 x[0,18] + w[0,18] <= 0
 VehicleRobotSynchronization1[0,19]: - 2 x[0,19] + w[0,19] <= 0
 VehicleRobotSynchronization1[0,20]: - 2 x[0,20] + w[0,20] <= 0
 VehicleRobotSynchronization1[0,21]: - 2 x[0,21] + w[0,21] <= 0
 VehicleRobotSynchronization1[0,22]: - 2 x[0,22] + w[0,22] <= 0
 VehicleRobotSynchronization1[0,23]: - 2 x[0,23] + w[0,23] <= 0
 VehicleRobotSynchronization1[1,0]: - 2 x[1,0] + w[1,0] <= 0
 VehicleRobotSynchronization1[1,2]: - 2 x[1,2] + w[1,2] <= 0
 VehicleRobotSynchronization1[1,3]: - 2 x[1,3] + w[1,3] <= 0
 VehicleRobotSynchronization1[1,4]: - 2 x[1,4] + w[1,4] <= 0
 VehicleRobotSynchronization1[1,5]: - 2 x[1,5] + w[1,5] <= 0
 VehicleRobotSynchronization1[1,6]: - 2 x[1,6] + w[1,6] <= 0
 VehicleRobotSynchronization1[1,7]: - 2 x[1,7] + w[1,7] <= 0
 VehicleRobotSynchronization1[1,8]: - 2 x[1,8] + w[1,8] <= 0
 VehicleRobotSynchronization1[1,9]: - 2 x[1,9] + w[1,9] <= 0
 VehicleRobotSynchronization1[1,10]: - 2 x[1,10] + w[1,10] <= 0
 VehicleRobotSynchronization1[1,11]: - 2 x[1,11] + w[1,11] <= 0
 VehicleRobotSynchronization1[1,12]: - 2 x[1,12] + w[1,12] <= 0
 VehicleRobotSynchronization1[1,13]: - 2 x[1,13] + w[1,13] <= 0
 VehicleRobotSynchronization1[1,14]: - 2 x[1,14] + w[1,14] <= 0
 VehicleRobotSynchronization1[1,15]: - 2 x[1,15] + w[1,15] <= 0
 VehicleRobotSynchronization1[1,16]: - 2 x[1,16] + w[1,16] <= 0
 VehicleRobotSynchronization1[1,17]: - 2 x[1,17] + w[1,17] <= 0
 VehicleRobotSynchronization1[1,18]: - 2 x[1,18] + w[1,18] <= 0
 VehicleRobotSynchronization1[1,19]: - 2 x[1,19] + w[1,19] <= 0
 VehicleRobotSynchronization1[1,20]: - 2 x[1,20] + w[1,20] <= 0
 VehicleRobotSynchronization1[1,21]: - 2 x[1,21] + w[1,21] <= 0
 VehicleRobotSynchronization1[1,22]: - 2 x[1,22] + w[1,22] <= 0
 VehicleRobotSynchronization1[1,23]: - 2 x[1,23] + w[1,23] <= 0
 VehicleRobotSynchronization1[2,0]: - 2 x[2,0] + w[2,0] <= 0
 VehicleRobotSynchronization1[2,1]: - 2 x[2,1] + w[2,1] <= 0
 VehicleRobotSynchronization1[2,3]: - 2 x[2,3] + w[2,3] <= 0
 VehicleRobotSynchronization1[2,4]: - 2 x[2,4] + w[2,4] <= 0
 VehicleRobotSynchronization1[2,5]: - 2 x[2,5] + w[2,5] <= 0
 VehicleRobotSynchronization1[2,6]: - 2 x[2,6] + w[2,6] <= 0
 VehicleRobotSynchronization1[2,7]: - 2 x[2,7] + w[2,7] <= 0
 VehicleRobotSynchronization1[2,8]: - 2 x[2,8] + w[2,8] <= 0
 VehicleRobotSynchronization1[2,9]: - 2 x[2,9] + w[2,9] <= 0
 VehicleRobotSynchronization1[2,10]: - 2 x[2,10] + w[2,10] <= 0
 VehicleRobotSynchronization1[2,11]: - 2 x[2,11] + w[2,11] <= 0
 VehicleRobotSynchronization1[2,13]: - 2 x[2,13] + w[2,13] <= 0
 VehicleRobotSynchronization1[2,14]: - 2 x[2,14] + w[2,14] <= 0
 VehicleRobotSynchronization1[2,15]: - 2 x[2,15] + w[2,15] <= 0
 VehicleRobotSynchronization1[2,16]: - 2 x[2,16] + w[2,16] <= 0
 VehicleRobotSynchronization1[2,17]: - 2 x[2,17] + w[2,17] <= 0
 VehicleRobotSynchronization1[2,18]: - 2 x[2,18] + w[2,18] <= 0
 VehicleRobotSynchronization1[2,19]: - 2 x[2,19] + w[2,19] <= 0
 VehicleRobotSynchronization1[2,20]: - 2 x[2,20] + w[2,20] <= 0
 VehicleRobotSynchronization1[2,21]: - 2 x[2,21] + w[2,21] <= 0
 VehicleRobotSynchronization1[2,22]: - 2 x[2,22] + w[2,22] <= 0
 VehicleRobotSynchronization1[2,23]: - 2 x[2,23] + w[2,23] <= 0
 VehicleRobotSynchronization1[3,0]: - 2 x[3,0] + w[3,0] <= 0
 VehicleRobotSynchronization1[3,1]: - 2 x[3,1] + w[3,1] <= 0
 VehicleRobotSynchronization1[3,2]: - 2 x[3,2] + w[3,2] <= 0
 VehicleRobotSynchronization1[3,4]: - 2 x[3,4] + w[3,4] <= 0
 VehicleRobotSynchronization1[3,5]: - 2 x[3,5] + w[3,5] <= 0
 VehicleRobotSynchronization1[3,6]: - 2 x[3,6] + w[3,6] <= 0
 VehicleRobotSynchronization1[3,7]: - 2 x[3,7] + w[3,7] <= 0
 VehicleRobotSynchronization1[3,8]: - 2 x[3,8] + w[3,8] <= 0
 VehicleRobotSynchronization1[3,9]: - 2 x[3,9] + w[3,9] <= 0
 VehicleRobotSynchronization1[3,10]: - 2 x[3,10] + w[3,10] <= 0
 VehicleRobotSynchronization1[3,11]: - 2 x[3,11] + w[3,11] <= 0
 VehicleRobotSynchronization1[3,12]: - 2 x[3,12] + w[3,12] <= 0
 VehicleRobotSynchronization1[3,14]: - 2 x[3,14] + w[3,14] <= 0
 VehicleRobotSynchronization1[3,15]: - 2 x[3,15] + w[3,15] <= 0
 VehicleRobotSynchronization1[3,16]: - 2 x[3,16] + w[3,16] <= 0
 VehicleRobotSynchronization1[3,17]: - 2 x[3,17] + w[3,17] <= 0
 VehicleRobotSynchronization1[3,18]: - 2 x[3,18] + w[3,18] <= 0
 VehicleRobotSynchronization1[3,19]: - 2 x[3,19] + w[3,19] <= 0
 VehicleRobotSynchronization1[3,20]: - 2 x[3,20] + w[3,20] <= 0
 VehicleRobotSynchronization1[3,21]: - 2 x[3,21] + w[3,21] <= 0
 VehicleRobotSynchronization1[3,22]: - 2 x[3,22] + w[3,22] <= 0
 VehicleRobotSynchronization1[3,23]: - 2 x[3,23] + w[3,23] <= 0
 VehicleRobotSynchronization1[4,0]: - 2 x[4,0] + w[4,0] <= 0
 VehicleRobotSynchronization1[4,1]: - 2 x[4,1] + w[4,1] <= 0
 VehicleRobotSynchronization1[4,2]: - 2 x[4,2] + w[4,2] <= 0
 VehicleRobotSynchronization1[4,3]: - 2 x[4,3] + w[4,3] <= 0
 VehicleRobotSynchronization1[4,5]: - 2 x[4,5] + w[4,5] <= 0
 VehicleRobotSynchronization1[4,6]: - 2 x[4,6] + w[4,6] <= 0
 VehicleRobotSynchronization1[4,7]: - 2 x[4,7] + w[4,7] <= 0
 VehicleRobotSynchronization1[4,8]: - 2 x[4,8] + w[4,8] <= 0
 VehicleRobotSynchronization1[4,9]: - 2 x[4,9] + w[4,9] <= 0
 VehicleRobotSynchronization1[4,10]: - 2 x[4,10] + w[4,10] <= 0
 VehicleRobotSynchronization1[4,11]: - 2 x[4,11] + w[4,11] <= 0
 VehicleRobotSynchronization1[4,12]: - 2 x[4,12] + w[4,12] <= 0
 VehicleRobotSynchronization1[4,13]: - 2 x[4,13] + w[4,13] <= 0
 VehicleRobotSynchronization1[4,15]: - 2 x[4,15] + w[4,15] <= 0
 VehicleRobotSynchronization1[4,16]: - 2 x[4,16] + w[4,16] <= 0
 VehicleRobotSynchronization1[4,17]: - 2 x[4,17] + w[4,17] <= 0
 VehicleRobotSynchronization1[4,18]: - 2 x[4,18] + w[4,18] <= 0
 VehicleRobotSynchronization1[4,19]: - 2 x[4,19] + w[4,19] <= 0
 VehicleRobotSynchronization1[4,20]: - 2 x[4,20] + w[4,20] <= 0
 VehicleRobotSynchronization1[4,21]: - 2 x[4,21] + w[4,21] <= 0
 VehicleRobotSynchronization1[4,22]: - 2 x[4,22] + w[4,22] <= 0
 VehicleRobotSynchronization1[4,23]: - 2 x[4,23] + w[4,23] <= 0
 VehicleRobotSynchronization1[5,0]: - 2 x[5,0] + w[5,0] <= 0
 VehicleRobotSynchronization1[5,1]: - 2 x[5,1] + w[5,1] <= 0
 VehicleRobotSynchronization1[5,2]: - 2 x[5,2] + w[5,2] <= 0
 VehicleRobotSynchronization1[5,3]: - 2 x[5,3] + w[5,3] <= 0
 VehicleRobotSynchronization1[5,4]: - 2 x[5,4] + w[5,4] <= 0
 VehicleRobotSynchronization1[5,6]: - 2 x[5,6] + w[5,6] <= 0
 VehicleRobotSynchronization1[5,7]: - 2 x[5,7] + w[5,7] <= 0
 VehicleRobotSynchronization1[5,8]: - 2 x[5,8] + w[5,8] <= 0
 VehicleRobotSynchronization1[5,9]: - 2 x[5,9] + w[5,9] <= 0
 VehicleRobotSynchronization1[5,10]: - 2 x[5,10] + w[5,10] <= 0
 VehicleRobotSynchronization1[5,11]: - 2 x[5,11] + w[5,11] <= 0
 VehicleRobotSynchronization1[5,12]: - 2 x[5,12] + w[5,12] <= 0
 VehicleRobotSynchronization1[5,13]: - 2 x[5,13] + w[5,13] <= 0
 VehicleRobotSynchronization1[5,14]: - 2 x[5,14] + w[5,14] <= 0
 VehicleRobotSynchronization1[5,16]: - 2 x[5,16] + w[5,16] <= 0
 VehicleRobotSynchronization1[5,17]: - 2 x[5,17] + w[5,17] <= 0
 VehicleRobotSynchronization1[5,18]: - 2 x[5,18] + w[5,18] <= 0
 VehicleRobotSynchronization1[5,19]: - 2 x[5,19] + w[5,19] <= 0
 VehicleRobotSynchronization1[5,20]: - 2 x[5,20] + w[5,20] <= 0
 VehicleRobotSynchronization1[5,21]: - 2 x[5,21] + w[5,21] <= 0
 VehicleRobotSynchronization1[5,22]: - 2 x[5,22] + w[5,22] <= 0
 VehicleRobotSynchronization1[5,23]: - 2 x[5,23] + w[5,23] <= 0
 VehicleRobotSynchronization1[6,0]: - 2 x[6,0] + w[6,0] <= 0
 VehicleRobotSynchronization1[6,1]: - 2 x[6,1] + w[6,1] <= 0
 VehicleRobotSynchronization1[6,2]: - 2 x[6,2] + w[6,2] <= 0
 VehicleRobotSynchronization1[6,3]: - 2 x[6,3] + w[6,3] <= 0
 VehicleRobotSynchronization1[6,4]: - 2 x[6,4] + w[6,4] <= 0
 VehicleRobotSynchronization1[6,5]: - 2 x[6,5] + w[6,5] <= 0
 VehicleRobotSynchronization1[6,7]: - 2 x[6,7] + w[6,7] <= 0
 VehicleRobotSynchronization1[6,8]: - 2 x[6,8] + w[6,8] <= 0
 VehicleRobotSynchronization1[6,9]: - 2 x[6,9] + w[6,9] <= 0
 VehicleRobotSynchronization1[6,10]: - 2 x[6,10] + w[6,10] <= 0
 VehicleRobotSynchronization1[6,11]: - 2 x[6,11] + w[6,11] <= 0
 VehicleRobotSynchronization1[6,12]: - 2 x[6,12] + w[6,12] <= 0
 VehicleRobotSynchronization1[6,13]: - 2 x[6,13] + w[6,13] <= 0
 VehicleRobotSynchronization1[6,14]: - 2 x[6,14] + w[6,14] <= 0
 VehicleRobotSynchronization1[6,15]: - 2 x[6,15] + w[6,15] <= 0
 VehicleRobotSynchronization1[6,17]: - 2 x[6,17] + w[6,17] <= 0
 VehicleRobotSynchronization1[6,18]: - 2 x[6,18] + w[6,18] <= 0
 VehicleRobotSynchronization1[6,19]: - 2 x[6,19] + w[6,19] <= 0
 VehicleRobotSynchronization1[6,20]: - 2 x[6,20] + w[6,20] <= 0
 VehicleRobotSynchronization1[6,21]: - 2 x[6,21] + w[6,21] <= 0
 VehicleRobotSynchronization1[6,22]: - 2 x[6,22] + w[6,22] <= 0
 VehicleRobotSynchronization1[6,23]: - 2 x[6,23] + w[6,23] <= 0
 VehicleRobotSynchronization1[7,0]: - 2 x[7,0] + w[7,0] <= 0
 VehicleRobotSynchronization1[7,1]: - 2 x[7,1] + w[7,1] <= 0
 VehicleRobotSynchronization1[7,2]: - 2 x[7,2] + w[7,2] <= 0
 VehicleRobotSynchronization1[7,3]: - 2 x[7,3] + w[7,3] <= 0
 VehicleRobotSynchronization1[7,4]: - 2 x[7,4] + w[7,4] <= 0
 VehicleRobotSynchronization1[7,5]: - 2 x[7,5] + w[7,5] <= 0
 VehicleRobotSynchronization1[7,6]: - 2 x[7,6] + w[7,6] <= 0
 VehicleRobotSynchronization1[7,8]: - 2 x[7,8] + w[7,8] <= 0
 VehicleRobotSynchronization1[7,9]: - 2 x[7,9] + w[7,9] <= 0
 VehicleRobotSynchronization1[7,10]: - 2 x[7,10] + w[7,10] <= 0
 VehicleRobotSynchronization1[7,11]: - 2 x[7,11] + w[7,11] <= 0
 VehicleRobotSynchronization1[7,12]: - 2 x[7,12] + w[7,12] <= 0
 VehicleRobotSynchronization1[7,13]: - 2 x[7,13] + w[7,13] <= 0
 VehicleRobotSynchronization1[7,14]: - 2 x[7,14] + w[7,14] <= 0
 VehicleRobotSynchronization1[7,15]: - 2 x[7,15] + w[7,15] <= 0
 VehicleRobotSynchronization1[7,16]: - 2 x[7,16] + w[7,16] <= 0
 VehicleRobotSynchronization1[7,18]: - 2 x[7,18] + w[7,18] <= 0
 VehicleRobotSynchronization1[7,19]: - 2 x[7,19] + w[7,19] <= 0
 VehicleRobotSynchronization1[7,20]: - 2 x[7,20] + w[7,20] <= 0
 VehicleRobotSynchronization1[7,21]: - 2 x[7,21] + w[7,21] <= 0
 VehicleRobotSynchronization1[7,22]: - 2 x[7,22] + w[7,22] <= 0
 VehicleRobotSynchronization1[7,23]: - 2 x[7,23] + w[7,23] <= 0
 VehicleRobotSynchronization1[8,0]: - 2 x[8,0] + w[8,0] <= 0
 VehicleRobotSynchronization1[8,1]: - 2 x[8,1] + w[8,1] <= 0
 VehicleRobotSynchronization1[8,2]: - 2 x[8,2] + w[8,2] <= 0
 VehicleRobotSynchronization1[8,3]: - 2 x[8,3] + w[8,3] <= 0
 VehicleRobotSynchronization1[8,4]: - 2 x[8,4] + w[8,4] <= 0
 VehicleRobotSynchronization1[8,5]: - 2 x[8,5] + w[8,5] <= 0
 VehicleRobotSynchronization1[8,6]: - 2 x[8,6] + w[8,6] <= 0
 VehicleRobotSynchronization1[8,7]: - 2 x[8,7] + w[8,7] <= 0
 VehicleRobotSynchronization1[8,9]: - 2 x[8,9] + w[8,9] <= 0
 VehicleRobotSynchronization1[8,10]: - 2 x[8,10] + w[8,10] <= 0
 VehicleRobotSynchronization1[8,11]: - 2 x[8,11] + w[8,11] <= 0
 VehicleRobotSynchronization1[8,12]: - 2 x[8,12] + w[8,12] <= 0
 VehicleRobotSynchronization1[8,13]: - 2 x[8,13] + w[8,13] <= 0
 VehicleRobotSynchronization1[8,14]: - 2 x[8,14] + w[8,14] <= 0
 VehicleRobotSynchronization1[8,15]: - 2 x[8,15] + w[8,15] <= 0
 VehicleRobotSynchronization1[8,16]: - 2 x[8,16] + w[8,16] <= 0
 VehicleRobotSynchronization1[8,17]: - 2 x[8,17] + w[8,17] <= 0
 VehicleRobotSynchronization1[8,19]: - 2 x[8,19] + w[8,19] <= 0
 VehicleRobotSynchronization1[8,20]: - 2 x[8,20] + w[8,20] <= 0
 VehicleRobotSynchronization1[8,21]: - 2 x[8,21] + w[8,21] <= 0
 VehicleRobotSynchronization1[8,22]: - 2 x[8,22] + w[8,22] <= 0
 VehicleRobotSynchronization1[8,23]: - 2 x[8,23] + w[8,23] <= 0
 VehicleRobotSynchronization1[9,0]: - 2 x[9,0] + w[9,0] <= 0
 VehicleRobotSynchronization1[9,1]: - 2 x[9,1] + w[9,1] <= 0
 VehicleRobotSynchronization1[9,2]: - 2 x[9,2] + w[9,2] <= 0
 VehicleRobotSynchronization1[9,3]: - 2 x[9,3] + w[9,3] <= 0
 VehicleRobotSynchronization1[9,4]: - 2 x[9,4] + w[9,4] <= 0
 VehicleRobotSynchronization1[9,5]: - 2 x[9,5] + w[9,5] <= 0
 VehicleRobotSynchronization1[9,6]: - 2 x[9,6] + w[9,6] <= 0
 VehicleRobotSynchronization1[9,7]: - 2 x[9,7] + w[9,7] <= 0
 VehicleRobotSynchronization1[9,8]: - 2 x[9,8] + w[9,8] <= 0
 VehicleRobotSynchronization1[9,10]: - 2 x[9,10] + w[9,10] <= 0
 VehicleRobotSynchronization1[9,11]: - 2 x[9,11] + w[9,11] <= 0
 VehicleRobotSynchronization1[9,12]: - 2 x[9,12] + w[9,12] <= 0
 VehicleRobotSynchronization1[9,13]: - 2 x[9,13] + w[9,13] <= 0
 VehicleRobotSynchronization1[9,14]: - 2 x[9,14] + w[9,14] <= 0
 VehicleRobotSynchronization1[9,15]: - 2 x[9,15] + w[9,15] <= 0
 VehicleRobotSynchronization1[9,16]: - 2 x[9,16] + w[9,16] <= 0
 VehicleRobotSynchronization1[9,17]: - 2 x[9,17] + w[9,17] <= 0
 VehicleRobotSynchronization1[9,18]: - 2 x[9,18] + w[9,18] <= 0
 VehicleRobotSynchronization1[9,20]: - 2 x[9,20] + w[9,20] <= 0
 VehicleRobotSynchronization1[9,21]: - 2 x[9,21] + w[9,21] <= 0
 VehicleRobotSynchronization1[9,22]: - 2 x[9,22] + w[9,22] <= 0
 VehicleRobotSynchronization1[9,23]: - 2 x[9,23] + w[9,23] <= 0
 VehicleRobotSynchronization1[10,0]: - 2 x[10,0] + w[10,0] <= 0
 VehicleRobotSynchronization1[10,1]: - 2 x[10,1] + w[10,1] <= 0
 VehicleRobotSynchronization1[10,2]: - 2 x[10,2] + w[10,2] <= 0
 VehicleRobotSynchronization1[10,3]: - 2 x[10,3] + w[10,3] <= 0
 VehicleRobotSynchronization1[10,4]: - 2 x[10,4] + w[10,4] <= 0
 VehicleRobotSynchronization1[10,5]: - 2 x[10,5] + w[10,5] <= 0
 VehicleRobotSynchronization1[10,6]: - 2 x[10,6] + w[10,6] <= 0
 VehicleRobotSynchronization1[10,7]: - 2 x[10,7] + w[10,7] <= 0
 VehicleRobotSynchronization1[10,8]: - 2 x[10,8] + w[10,8] <= 0
 VehicleRobotSynchronization1[10,9]: - 2 x[10,9] + w[10,9] <= 0
 VehicleRobotSynchronization1[10,11]: - 2 x[10,11] + w[10,11] <= 0
 VehicleRobotSynchronization1[10,12]: - 2 x[10,12] + w[10,12] <= 0
 VehicleRobotSynchronization1[10,13]: - 2 x[10,13] + w[10,13] <= 0
 VehicleRobotSynchronization1[10,14]: - 2 x[10,14] + w[10,14] <= 0
 VehicleRobotSynchronization1[10,15]: - 2 x[10,15] + w[10,15] <= 0
 VehicleRobotSynchronization1[10,16]: - 2 x[10,16] + w[10,16] <= 0
 VehicleRobotSynchronization1[10,17]: - 2 x[10,17] + w[10,17] <= 0
 VehicleRobotSynchronization1[10,18]: - 2 x[10,18] + w[10,18] <= 0
 VehicleRobotSynchronization1[10,19]: - 2 x[10,19] + w[10,19] <= 0
 VehicleRobotSynchronization1[10,21]: - 2 x[10,21] + w[10,21] <= 0
 VehicleRobotSynchronization1[10,22]: - 2 x[10,22] + w[10,22] <= 0
 VehicleRobotSynchronization1[10,23]: - 2 x[10,23] + w[10,23] <= 0
 VehicleRobotSynchronization1[11,0]: - 2 x[11,0] + w[11,0] <= 0
 VehicleRobotSynchronization1[11,1]: - 2 x[11,1] + w[11,1] <= 0
 VehicleRobotSynchronization1[11,2]: - 2 x[11,2] + w[11,2] <= 0
 VehicleRobotSynchronization1[11,3]: - 2 x[11,3] + w[11,3] <= 0
 VehicleRobotSynchronization1[11,4]: - 2 x[11,4] + w[11,4] <= 0
 VehicleRobotSynchronization1[11,5]: - 2 x[11,5] + w[11,5] <= 0
 VehicleRobotSynchronization1[11,6]: - 2 x[11,6] + w[11,6] <= 0
 VehicleRobotSynchronization1[11,7]: - 2 x[11,7] + w[11,7] <= 0
 VehicleRobotSynchronization1[11,8]: - 2 x[11,8] + w[11,8] <= 0
 VehicleRobotSynchronization1[11,9]: - 2 x[11,9] + w[11,9] <= 0
 VehicleRobotSynchronization1[11,10]: - 2 x[11,10] + w[11,10] <= 0
 VehicleRobotSynchronization1[11,12]: - 2 x[11,12] + w[11,12] <= 0
 VehicleRobotSynchronization1[11,13]: - 2 x[11,13] + w[11,13] <= 0
 VehicleRobotSynchronization1[11,14]: - 2 x[11,14] + w[11,14] <= 0
 VehicleRobotSynchronization1[11,15]: - 2 x[11,15] + w[11,15] <= 0
 VehicleRobotSynchronization1[11,16]: - 2 x[11,16] + w[11,16] <= 0
 VehicleRobotSynchronization1[11,17]: - 2 x[11,17] + w[11,17] <= 0
 VehicleRobotSynchronization1[11,18]: - 2 x[11,18] + w[11,18] <= 0
 VehicleRobotSynchronization1[11,19]: - 2 x[11,19] + w[11,19] <= 0
 VehicleRobotSynchronization1[11,20]: - 2 x[11,20] + w[11,20] <= 0
 VehicleRobotSynchronization1[11,22]: - 2 x[11,22] + w[11,22] <= 0
 VehicleRobotSynchronization1[11,23]: - 2 x[11,23] + w[11,23] <= 0
 VehicleRobotSynchronization1[12,0]: - 2 x[12,0] + w[12,0] <= 0
 VehicleRobotSynchronization1[12,1]: - 2 x[12,1] + w[12,1] <= 0
 VehicleRobotSynchronization1[12,3]: - 2 x[12,3] + w[12,3] <= 0
 VehicleRobotSynchronization1[12,4]: - 2 x[12,4] + w[12,4] <= 0
 VehicleRobotSynchronization1[12,5]: - 2 x[12,5] + w[12,5] <= 0
 VehicleRobotSynchronization1[12,6]: - 2 x[12,6] + w[12,6] <= 0
 VehicleRobotSynchronization1[12,7]: - 2 x[12,7] + w[12,7] <= 0
 VehicleRobotSynchronization1[12,8]: - 2 x[12,8] + w[12,8] <= 0
 VehicleRobotSynchronization1[12,9]: - 2 x[12,9] + w[12,9] <= 0
 VehicleRobotSynchronization1[12,10]: - 2 x[12,10] + w[12,10] <= 0
 VehicleRobotSynchronization1[12,11]: - 2 x[12,11] + w[12,11] <= 0
 VehicleRobotSynchronization1[12,13]: - 2 x[12,13] + w[12,13] <= 0
 VehicleRobotSynchronization1[12,14]: - 2 x[12,14] + w[12,14] <= 0
 VehicleRobotSynchronization1[12,15]: - 2 x[12,15] + w[12,15] <= 0
 VehicleRobotSynchronization1[12,16]: - 2 x[12,16] + w[12,16] <= 0
 VehicleRobotSynchronization1[12,17]: - 2 x[12,17] + w[12,17] <= 0
 VehicleRobotSynchronization1[12,18]: - 2 x[12,18] + w[12,18] <= 0
 VehicleRobotSynchronization1[12,19]: - 2 x[12,19] + w[12,19] <= 0
 VehicleRobotSynchronization1[12,20]: - 2 x[12,20] + w[12,20] <= 0
 VehicleRobotSynchronization1[12,21]: - 2 x[12,21] + w[12,21] <= 0
 VehicleRobotSynchronization1[12,22]: - 2 x[12,22] + w[12,22] <= 0
 VehicleRobotSynchronization1[12,23]: - 2 x[12,23] + w[12,23] <= 0
 VehicleRobotSynchronization1[13,0]: - 2 x[13,0] + w[13,0] <= 0
 VehicleRobotSynchronization1[13,1]: - 2 x[13,1] + w[13,1] <= 0
 VehicleRobotSynchronization1[13,2]: - 2 x[13,2] + w[13,2] <= 0
 VehicleRobotSynchronization1[13,4]: - 2 x[13,4] + w[13,4] <= 0
 VehicleRobotSynchronization1[13,5]: - 2 x[13,5] + w[13,5] <= 0
 VehicleRobotSynchronization1[13,6]: - 2 x[13,6] + w[13,6] <= 0
 VehicleRobotSynchronization1[13,7]: - 2 x[13,7] + w[13,7] <= 0
 VehicleRobotSynchronization1[13,8]: - 2 x[13,8] + w[13,8] <= 0
 VehicleRobotSynchronization1[13,9]: - 2 x[13,9] + w[13,9] <= 0
 VehicleRobotSynchronization1[13,10]: - 2 x[13,10] + w[13,10] <= 0
 VehicleRobotSynchronization1[13,11]: - 2 x[13,11] + w[13,11] <= 0
 VehicleRobotSynchronization1[13,12]: - 2 x[13,12] + w[13,12] <= 0
 VehicleRobotSynchronization1[13,14]: - 2 x[13,14] + w[13,14] <= 0
 VehicleRobotSynchronization1[13,15]: - 2 x[13,15] + w[13,15] <= 0
 VehicleRobotSynchronization1[13,16]: - 2 x[13,16] + w[13,16] <= 0
 VehicleRobotSynchronization1[13,17]: - 2 x[13,17] + w[13,17] <= 0
 VehicleRobotSynchronization1[13,18]: - 2 x[13,18] + w[13,18] <= 0
 VehicleRobotSynchronization1[13,19]: - 2 x[13,19] + w[13,19] <= 0
 VehicleRobotSynchronization1[13,20]: - 2 x[13,20] + w[13,20] <= 0
 VehicleRobotSynchronization1[13,21]: - 2 x[13,21] + w[13,21] <= 0
 VehicleRobotSynchronization1[13,22]: - 2 x[13,22] + w[13,22] <= 0
 VehicleRobotSynchronization1[13,23]: - 2 x[13,23] + w[13,23] <= 0
 VehicleRobotSynchronization1[14,0]: - 2 x[14,0] + w[14,0] <= 0
 VehicleRobotSynchronization1[14,1]: - 2 x[14,1] + w[14,1] <= 0
 VehicleRobotSynchronization1[14,2]: - 2 x[14,2] + w[14,2] <= 0
 VehicleRobotSynchronization1[14,3]: - 2 x[14,3] + w[14,3] <= 0
 VehicleRobotSynchronization1[14,5]: - 2 x[14,5] + w[14,5] <= 0
 VehicleRobotSynchronization1[14,6]: - 2 x[14,6] + w[14,6] <= 0
 VehicleRobotSynchronization1[14,7]: - 2 x[14,7] + w[14,7] <= 0
 VehicleRobotSynchronization1[14,8]: - 2 x[14,8] + w[14,8] <= 0
 VehicleRobotSynchronization1[14,9]: - 2 x[14,9] + w[14,9] <= 0
 VehicleRobotSynchronization1[14,10]: - 2 x[14,10] + w[14,10] <= 0
 VehicleRobotSynchronization1[14,11]: - 2 x[14,11] + w[14,11] <= 0
 VehicleRobotSynchronization1[14,12]: - 2 x[14,12] + w[14,12] <= 0
 VehicleRobotSynchronization1[14,13]: - 2 x[14,13] + w[14,13] <= 0
 VehicleRobotSynchronization1[14,15]: - 2 x[14,15] + w[14,15] <= 0
 VehicleRobotSynchronization1[14,16]: - 2 x[14,16] + w[14,16] <= 0
 VehicleRobotSynchronization1[14,17]: - 2 x[14,17] + w[14,17] <= 0
 VehicleRobotSynchronization1[14,18]: - 2 x[14,18] + w[14,18] <= 0
 VehicleRobotSynchronization1[14,19]: - 2 x[14,19] + w[14,19] <= 0
 VehicleRobotSynchronization1[14,20]: - 2 x[14,20] + w[14,20] <= 0
 VehicleRobotSynchronization1[14,21]: - 2 x[14,21] + w[14,21] <= 0
 VehicleRobotSynchronization1[14,22]: - 2 x[14,22] + w[14,22] <= 0
 VehicleRobotSynchronization1[14,23]: - 2 x[14,23] + w[14,23] <= 0
 VehicleRobotSynchronization1[15,0]: - 2 x[15,0] + w[15,0] <= 0
 VehicleRobotSynchronization1[15,1]: - 2 x[15,1] + w[15,1] <= 0
 VehicleRobotSynchronization1[15,2]: - 2 x[15,2] + w[15,2] <= 0
 VehicleRobotSynchronization1[15,3]: - 2 x[15,3] + w[15,3] <= 0
 VehicleRobotSynchronization1[15,4]: - 2 x[15,4] + w[15,4] <= 0
 VehicleRobotSynchronization1[15,6]: - 2 x[15,6] + w[15,6] <= 0
 VehicleRobotSynchronization1[15,7]: - 2 x[15,7] + w[15,7] <= 0
 VehicleRobotSynchronization1[15,8]: - 2 x[15,8] + w[15,8] <= 0
 VehicleRobotSynchronization1[15,9]: - 2 x[15,9] + w[15,9] <= 0
 VehicleRobotSynchronization1[15,10]: - 2 x[15,10] + w[15,10] <= 0
 VehicleRobotSynchronization1[15,11]: - 2 x[15,11] + w[15,11] <= 0
 VehicleRobotSynchronization1[15,12]: - 2 x[15,12] + w[15,12] <= 0
 VehicleRobotSynchronization1[15,13]: - 2 x[15,13] + w[15,13] <= 0
 VehicleRobotSynchronization1[15,14]: - 2 x[15,14] + w[15,14] <= 0
 VehicleRobotSynchronization1[15,16]: - 2 x[15,16] + w[15,16] <= 0
 VehicleRobotSynchronization1[15,17]: - 2 x[15,17] + w[15,17] <= 0
 VehicleRobotSynchronization1[15,18]: - 2 x[15,18] + w[15,18] <= 0
 VehicleRobotSynchronization1[15,19]: - 2 x[15,19] + w[15,19] <= 0
 VehicleRobotSynchronization1[15,20]: - 2 x[15,20] + w[15,20] <= 0
 VehicleRobotSynchronization1[15,21]: - 2 x[15,21] + w[15,21] <= 0
 VehicleRobotSynchronization1[15,22]: - 2 x[15,22] + w[15,22] <= 0
 VehicleRobotSynchronization1[15,23]: - 2 x[15,23] + w[15,23] <= 0
 VehicleRobotSynchronization1[16,0]: - 2 x[16,0] + w[16,0] <= 0
 VehicleRobotSynchronization1[16,1]: - 2 x[16,1] + w[16,1] <= 0
 VehicleRobotSynchronization1[16,2]: - 2 x[16,2] + w[16,2] <= 0
 VehicleRobotSynchronization1[16,3]: - 2 x[16,3] + w[16,3] <= 0
 VehicleRobotSynchronization1[16,4]: - 2 x[16,4] + w[16,4] <= 0
 VehicleRobotSynchronization1[16,5]: - 2 x[16,5] + w[16,5] <= 0
 VehicleRobotSynchronization1[16,7]: - 2 x[16,7] + w[16,7] <= 0
 VehicleRobotSynchronization1[16,8]: - 2 x[16,8] + w[16,8] <= 0
 VehicleRobotSynchronization1[16,9]: - 2 x[16,9] + w[16,9] <= 0
 VehicleRobotSynchronization1[16,10]: - 2 x[16,10] + w[16,10] <= 0
 VehicleRobotSynchronization1[16,11]: - 2 x[16,11] + w[16,11] <= 0
 VehicleRobotSynchronization1[16,12]: - 2 x[16,12] + w[16,12] <= 0
 VehicleRobotSynchronization1[16,13]: - 2 x[16,13] + w[16,13] <= 0
 VehicleRobotSynchronization1[16,14]: - 2 x[16,14] + w[16,14] <= 0
 VehicleRobotSynchronization1[16,15]: - 2 x[16,15] + w[16,15] <= 0
 VehicleRobotSynchronization1[16,17]: - 2 x[16,17] + w[16,17] <= 0
 VehicleRobotSynchronization1[16,18]: - 2 x[16,18] + w[16,18] <= 0
 VehicleRobotSynchronization1[16,19]: - 2 x[16,19] + w[16,19] <= 0
 VehicleRobotSynchronization1[16,20]: - 2 x[16,20] + w[16,20] <= 0
 VehicleRobotSynchronization1[16,21]: - 2 x[16,21] + w[16,21] <= 0
 VehicleRobotSynchronization1[16,22]: - 2 x[16,22] + w[16,22] <= 0
 VehicleRobotSynchronization1[16,23]: - 2 x[16,23] + w[16,23] <= 0
 VehicleRobotSynchronization1[17,0]: - 2 x[17,0] + w[17,0] <= 0
 VehicleRobotSynchronization1[17,1]: - 2 x[17,1] + w[17,1] <= 0
 VehicleRobotSynchronization1[17,2]: - 2 x[17,2] + w[17,2] <= 0
 VehicleRobotSynchronization1[17,3]: - 2 x[17,3] + w[17,3] <= 0
 VehicleRobotSynchronization1[17,4]: - 2 x[17,4] + w[17,4] <= 0
 VehicleRobotSynchronization1[17,5]: - 2 x[17,5] + w[17,5] <= 0
 VehicleRobotSynchronization1[17,6]: - 2 x[17,6] + w[17,6] <= 0
 VehicleRobotSynchronization1[17,8]: - 2 x[17,8] + w[17,8] <= 0
 VehicleRobotSynchronization1[17,9]: - 2 x[17,9] + w[17,9] <= 0
 VehicleRobotSynchronization1[17,10]: - 2 x[17,10] + w[17,10] <= 0
 VehicleRobotSynchronization1[17,11]: - 2 x[17,11] + w[17,11] <= 0
 VehicleRobotSynchronization1[17,12]: - 2 x[17,12] + w[17,12] <= 0
 VehicleRobotSynchronization1[17,13]: - 2 x[17,13] + w[17,13] <= 0
 VehicleRobotSynchronization1[17,14]: - 2 x[17,14] + w[17,14] <= 0
 VehicleRobotSynchronization1[17,15]: - 2 x[17,15] + w[17,15] <= 0
 VehicleRobotSynchronization1[17,16]: - 2 x[17,16] + w[17,16] <= 0
 VehicleRobotSynchronization1[17,18]: - 2 x[17,18] + w[17,18] <= 0
 VehicleRobotSynchronization1[17,19]: - 2 x[17,19] + w[17,19] <= 0
 VehicleRobotSynchronization1[17,20]: - 2 x[17,20] + w[17,20] <= 0
 VehicleRobotSynchronization1[17,21]: - 2 x[17,21] + w[17,21] <= 0
 VehicleRobotSynchronization1[17,22]: - 2 x[17,22] + w[17,22] <= 0
 VehicleRobotSynchronization1[17,23]: - 2 x[17,23] + w[17,23] <= 0
 VehicleRobotSynchronization1[18,0]: - 2 x[18,0] + w[18,0] <= 0
 VehicleRobotSynchronization1[18,1]: - 2 x[18,1] + w[18,1] <= 0
 VehicleRobotSynchronization1[18,2]: - 2 x[18,2] + w[18,2] <= 0
 VehicleRobotSynchronization1[18,3]: - 2 x[18,3] + w[18,3] <= 0
 VehicleRobotSynchronization1[18,4]: - 2 x[18,4] + w[18,4] <= 0
 VehicleRobotSynchronization1[18,5]: - 2 x[18,5] + w[18,5] <= 0
 VehicleRobotSynchronization1[18,6]: - 2 x[18,6] + w[18,6] <= 0
 VehicleRobotSynchronization1[18,7]: - 2 x[18,7] + w[18,7] <= 0
 VehicleRobotSynchronization1[18,9]: - 2 x[18,9] + w[18,9] <= 0
 VehicleRobotSynchronization1[18,10]: - 2 x[18,10] + w[18,10] <= 0
 VehicleRobotSynchronization1[18,11]: - 2 x[18,11] + w[18,11] <= 0
 VehicleRobotSynchronization1[18,12]: - 2 x[18,12] + w[18,12] <= 0
 VehicleRobotSynchronization1[18,13]: - 2 x[18,13] + w[18,13] <= 0
 VehicleRobotSynchronization1[18,14]: - 2 x[18,14] + w[18,14] <= 0
 VehicleRobotSynchronization1[18,15]: - 2 x[18,15] + w[18,15] <= 0
 VehicleRobotSynchronization1[18,16]: - 2 x[18,16] + w[18,16] <= 0
 VehicleRobotSynchronization1[18,17]: - 2 x[18,17] + w[18,17] <= 0
 VehicleRobotSynchronization1[18,19]: - 2 x[18,19] + w[18,19] <= 0
 VehicleRobotSynchronization1[18,20]: - 2 x[18,20] + w[18,20] <= 0
 VehicleRobotSynchronization1[18,21]: - 2 x[18,21] + w[18,21] <= 0
 VehicleRobotSynchronization1[18,22]: - 2 x[18,22] + w[18,22] <= 0
 VehicleRobotSynchronization1[18,23]: - 2 x[18,23] + w[18,23] <= 0
 VehicleRobotSynchronization1[19,0]: - 2 x[19,0] + w[19,0] <= 0
 VehicleRobotSynchronization1[19,1]: - 2 x[19,1] + w[19,1] <= 0
 VehicleRobotSynchronization1[19,2]: - 2 x[19,2] + w[19,2] <= 0
 VehicleRobotSynchronization1[19,3]: - 2 x[19,3] + w[19,3] <= 0
 VehicleRobotSynchronization1[19,4]: - 2 x[19,4] + w[19,4] <= 0
 VehicleRobotSynchronization1[19,5]: - 2 x[19,5] + w[19,5] <= 0
 VehicleRobotSynchronization1[19,6]: - 2 x[19,6] + w[19,6] <= 0
 VehicleRobotSynchronization1[19,7]: - 2 x[19,7] + w[19,7] <= 0
 VehicleRobotSynchronization1[19,8]: - 2 x[19,8] + w[19,8] <= 0
 VehicleRobotSynchronization1[19,10]: - 2 x[19,10] + w[19,10] <= 0
 VehicleRobotSynchronization1[19,11]: - 2 x[19,11] + w[19,11] <= 0
 VehicleRobotSynchronization1[19,12]: - 2 x[19,12] + w[19,12] <= 0
 VehicleRobotSynchronization1[19,13]: - 2 x[19,13] + w[19,13] <= 0
 VehicleRobotSynchronization1[19,14]: - 2 x[19,14] + w[19,14] <= 0
 VehicleRobotSynchronization1[19,15]: - 2 x[19,15] + w[19,15] <= 0
 VehicleRobotSynchronization1[19,16]: - 2 x[19,16] + w[19,16] <= 0
 VehicleRobotSynchronization1[19,17]: - 2 x[19,17] + w[19,17] <= 0
 VehicleRobotSynchronization1[19,18]: - 2 x[19,18] + w[19,18] <= 0
 VehicleRobotSynchronization1[19,20]: - 2 x[19,20] + w[19,20] <= 0
 VehicleRobotSynchronization1[19,21]: - 2 x[19,21] + w[19,21] <= 0
 VehicleRobotSynchronization1[19,22]: - 2 x[19,22] + w[19,22] <= 0
 VehicleRobotSynchronization1[19,23]: - 2 x[19,23] + w[19,23] <= 0
 VehicleRobotSynchronization1[20,0]: - 2 x[20,0] + w[20,0] <= 0
 VehicleRobotSynchronization1[20,1]: - 2 x[20,1] + w[20,1] <= 0
 VehicleRobotSynchronization1[20,2]: - 2 x[20,2] + w[20,2] <= 0
 VehicleRobotSynchronization1[20,3]: - 2 x[20,3] + w[20,3] <= 0
 VehicleRobotSynchronization1[20,4]: - 2 x[20,4] + w[20,4] <= 0
 VehicleRobotSynchronization1[20,5]: - 2 x[20,5] + w[20,5] <= 0
 VehicleRobotSynchronization1[20,6]: - 2 x[20,6] + w[20,6] <= 0
 VehicleRobotSynchronization1[20,7]: - 2 x[20,7] + w[20,7] <= 0
 VehicleRobotSynchronization1[20,8]: - 2 x[20,8] + w[20,8] <= 0
 VehicleRobotSynchronization1[20,9]: - 2 x[20,9] + w[20,9] <= 0
 VehicleRobotSynchronization1[20,11]: - 2 x[20,11] + w[20,11] <= 0
 VehicleRobotSynchronization1[20,12]: - 2 x[20,12] + w[20,12] <= 0
 VehicleRobotSynchronization1[20,13]: - 2 x[20,13] + w[20,13] <= 0
 VehicleRobotSynchronization1[20,14]: - 2 x[20,14] + w[20,14] <= 0
 VehicleRobotSynchronization1[20,15]: - 2 x[20,15] + w[20,15] <= 0
 VehicleRobotSynchronization1[20,16]: - 2 x[20,16] + w[20,16] <= 0
 VehicleRobotSynchronization1[20,17]: - 2 x[20,17] + w[20,17] <= 0
 VehicleRobotSynchronization1[20,18]: - 2 x[20,18] + w[20,18] <= 0
 VehicleRobotSynchronization1[20,19]: - 2 x[20,19] + w[20,19] <= 0
 VehicleRobotSynchronization1[20,21]: - 2 x[20,21] + w[20,21] <= 0
 VehicleRobotSynchronization1[20,22]: - 2 x[20,22] + w[20,22] <= 0
 VehicleRobotSynchronization1[20,23]: - 2 x[20,23] + w[20,23] <= 0
 VehicleRobotSynchronization1[21,0]: - 2 x[21,0] + w[21,0] <= 0
 VehicleRobotSynchronization1[21,1]: - 2 x[21,1] + w[21,1] <= 0
 VehicleRobotSynchronization1[21,2]: - 2 x[21,2] + w[21,2] <= 0
 VehicleRobotSynchronization1[21,3]: - 2 x[21,3] + w[21,3] <= 0
 VehicleRobotSynchronization1[21,4]: - 2 x[21,4] + w[21,4] <= 0
 VehicleRobotSynchronization1[21,5]: - 2 x[21,5] + w[21,5] <= 0
 VehicleRobotSynchronization1[21,6]: - 2 x[21,6] + w[21,6] <= 0
 VehicleRobotSynchronization1[21,7]: - 2 x[21,7] + w[21,7] <= 0
 VehicleRobotSynchronization1[21,8]: - 2 x[21,8] + w[21,8] <= 0
 VehicleRobotSynchronization1[21,9]: - 2 x[21,9] + w[21,9] <= 0
 VehicleRobotSynchronization1[21,10]: - 2 x[21,10] + w[21,10] <= 0
 VehicleRobotSynchronization1[21,12]: - 2 x[21,12] + w[21,12] <= 0
 VehicleRobotSynchronization1[21,13]: - 2 x[21,13] + w[21,13] <= 0
 VehicleRobotSynchronization1[21,14]: - 2 x[21,14] + w[21,14] <= 0
 VehicleRobotSynchronization1[21,15]: - 2 x[21,15] + w[21,15] <= 0
 VehicleRobotSynchronization1[21,16]: - 2 x[21,16] + w[21,16] <= 0
 VehicleRobotSynchronization1[21,17]: - 2 x[21,17] + w[21,17] <= 0
 VehicleRobotSynchronization1[21,18]: - 2 x[21,18] + w[21,18] <= 0
 VehicleRobotSynchronization1[21,19]: - 2 x[21,19] + w[21,19] <= 0
 VehicleRobotSynchronization1[21,20]: - 2 x[21,20] + w[21,20] <= 0
 VehicleRobotSynchronization1[21,22]: - 2 x[21,22] + w[21,22] <= 0
 VehicleRobotSynchronization1[21,23]: - 2 x[21,23] + w[21,23] <= 0
 VehicleRobotSynchronization1[22,0]: - 2 x[22,0] + w[22,0] <= 0
 VehicleRobotSynchronization1[22,1]: - 2 x[22,1] + w[22,1] <= 0
 VehicleRobotSynchronization1[22,2]: - 2 x[22,2] + w[22,2] <= 0
 VehicleRobotSynchronization1[22,3]: - 2 x[22,3] + w[22,3] <= 0
 VehicleRobotSynchronization1[22,4]: - 2 x[22,4] + w[22,4] <= 0
 VehicleRobotSynchronization1[22,5]: - 2 x[22,5] + w[22,5] <= 0
 VehicleRobotSynchronization1[22,6]: - 2 x[22,6] + w[22,6] <= 0
 VehicleRobotSynchronization1[22,7]: - 2 x[22,7] + w[22,7] <= 0
 VehicleRobotSynchronization1[22,8]: - 2 x[22,8] + w[22,8] <= 0
 VehicleRobotSynchronization1[22,9]: - 2 x[22,9] + w[22,9] <= 0
 VehicleRobotSynchronization1[22,10]: - 2 x[22,10] + w[22,10] <= 0
 VehicleRobotSynchronization1[22,11]: - 2 x[22,11] + w[22,11] <= 0
 VehicleRobotSynchronization1[22,12]: - 2 x[22,12] + w[22,12] <= 0
 VehicleRobotSynchronization1[22,13]: - 2 x[22,13] + w[22,13] <= 0
 VehicleRobotSynchronization1[22,14]: - 2 x[22,14] + w[22,14] <= 0
 VehicleRobotSynchronization1[22,15]: - 2 x[22,15] + w[22,15] <= 0
 VehicleRobotSynchronization1[22,16]: - 2 x[22,16] + w[22,16] <= 0
 VehicleRobotSynchronization1[22,17]: - 2 x[22,17] + w[22,17] <= 0
 VehicleRobotSynchronization1[22,18]: - 2 x[22,18] + w[22,18] <= 0
 VehicleRobotSynchronization1[22,19]: - 2 x[22,19] + w[22,19] <= 0
 VehicleRobotSynchronization1[22,20]: - 2 x[22,20] + w[22,20] <= 0
 VehicleRobotSynchronization1[22,21]: - 2 x[22,21] + w[22,21] <= 0
 VehicleRobotSynchronization1[22,23]: - 2 x[22,23] + w[22,23] <= 0
 SettingMode[2]: - w[2,12] + y[2,1] + 2 y[2,2] = 0
 SettingMode[3]: - w[3,13] + y[3,1] + 2 y[3,2] = 0
 SettingMode[4]: - w[4,14] + y[4,1] + 2 y[4,2] = 0
 SettingMode[5]: - w[5,15] + y[5,1] + 2 y[5,2] = 0
 SettingMode[6]: - w[6,16] + y[6,1] + 2 y[6,2] = 0
 SettingMode[7]: - w[7,17] + y[7,1] + 2 y[7,2] = 0
 SettingMode[8]: - w[8,18] + y[8,1] + 2 y[8,2] = 0
 SettingMode[9]: - w[9,19] + y[9,1] + 2 y[9,2] = 0
 SettingMode[10]: - w[10,20] + y[10,1] + 2 y[10,2] = 0
 SettingMode[11]: - w[11,21] + y[11,1] + 2 y[11,2] = 0
 SettingServiceTime[2]: - 220 y[2,1] - 110 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 340 y[3,1] - 170 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 490 y[4,1] - 245 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 490 y[5,1] - 245 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 300 y[6,1] - 150 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 440 y[7,1] - 220 y[7,2] + service_time[7] = 0
 SettingServiceTime[8]: - 270 y[8,1] - 135 y[8,2] + service_time[8] = 0
 SettingServiceTime[9]: - 240 y[9,1] - 120 y[9,2] + service_time[9] = 0
 SettingServiceTime[10]: - 340 y[10,1] - 170 y[10,2] + service_time[10]
   = 0
 SettingServiceTime[11]: - 230 y[11,1] - 115 y[11,2] + service_time[11]
   = 0
 AverageTimeRobots: - 7.653030609056256 w[0,2] - 5.154385649029794 w[0,3]
   - 8.773753398145478 w[0,4] - 7.373516367027557 w[0,5]
   - 8.978795400474022 w[0,6] - 8.267686142994348 w[0,7]
   - 6.748278456541565 w[0,8] - 3.938714087887019 w[0,9]
   - 9.179354857933594 w[0,10] - 2.119489211471819 w[0,11]
   - 7.653030609056256 w[0,12] - 5.154385649029794 w[0,13]
   - 8.773753398145478 w[0,14] - 7.373516367027557 w[0,15]
   - 8.978795400474022 w[0,16] - 8.267686142994348 w[0,17]
   - 6.748278456541565 w[0,18] - 3.938714087887019 w[0,19]
   - 9.179354857933594 w[0,20] - 2.119489211471819 w[0,21]
   - 7.653030609056256 w[1,2] - 5.154385649029794 w[1,3]
   - 8.773753398145478 w[1,4] - 7.373516367027557 w[1,5]
   - 8.978795400474022 w[1,6] - 8.267686142994348 w[1,7]
   - 6.748278456541565 w[1,8] - 3.938714087887019 w[1,9]
   - 9.179354857933594 w[1,10] - 2.119489211471819 w[1,11]
   - 7.653030609056256 w[1,12] - 5.154385649029794 w[1,13]
   - 8.773753398145478 w[1,14] - 7.373516367027557 w[1,15]
   - 8.978795400474022 w[1,16] - 8.267686142994348 w[1,17]
   - 6.748278456541565 w[1,18] - 3.938714087887019 w[1,19]
   - 9.179354857933594 w[1,20] - 2.119489211471819 w[1,21]
   - 7.653030609056256 w[2,1] - 6.14565516095383 w[2,3]
   - 8.596689262039828 w[2,4] - 10.42498079001441 w[2,5]
   - 10.43254719125622 w[2,6] - 9.35008533273758 w[2,7]
   - 2.744832300932999 w[2,8] - 9.627913520343157 w[2,9]
   - 1.687257120839084 w[2,10] - 9.494112783392445 w[2,11]
   - 6.14565516095383 w[2,13] - 8.596689262039828 w[2,14]
   - 10.42498079001441 w[2,15] - 10.43254719125622 w[2,16]
   - 9.35008533273758 w[2,17] - 2.744832300932999 w[2,18]
   - 9.627913520343157 w[2,19] - 1.687257120839084 w[2,20]
   - 9.494112783392445 w[2,21] - 7.653030609056256 w[2,22]
   - 7.653030609056256 w[2,23] - 5.154385649029794 w[3,1]
   - 6.14565516095383 w[3,2] - 3.789393087568826 w[3,4]
   - 11.83039892442283 w[3,5] - 12.92698372597812 w[3,6]
   - 3.767878075007341 w[3,7] - 3.650013429871618 w[3,8]
   - 8.971746540533758 w[3,9] - 7.791652027865135 w[3,10]
   - 5.690832636266907 w[3,11] - 6.14565516095383 w[3,12]
   - 3.789393087568826 w[3,14] - 11.83039892442283 w[3,15]
   - 12.92698372597812 w[3,16] - 3.767878075007341 w[3,17]
   - 3.650013429871618 w[3,18] - 8.971746540533758 w[3,19]
   - 7.791652027865135 w[3,20] - 5.690832636266907 w[3,21]
   - 5.154385649029794 w[3,22] - 5.154385649029794 w[3,23]
   - 8.773753398145478 w[4,1] - 8.596689262039828 w[4,2]
   - 3.789393087568826 w[4,3] - 15.61969255885572 w[4,5]
   - 16.67768228994561 w[4,6] - 1.448059261536187 w[4,7]
   - 5.863519766576599 w[4,8] - 12.67637825119609 w[4,9]
   - 9.99197232225618 w[4,10] - 8.804653430672097 w[4,11]
   - 8.596689262039828 w[4,12] - 3.789393087568826 w[4,13]
   - 15.61969255885572 w[4,15] - 16.67768228994561 w[4,16]
   - 1.448059261536187 w[4,17] - 5.863519766576599 w[4,18]
   - 12.67637825119609 w[4,19] - 9.99197232225618 w[4,20]
   - 8.804653430672097 w[4,21] - 8.773753398145478 w[4,22]
   - 8.773753398145478 w[4,23] - 7.373516367027557 w[5,1]
   - 10.42498079001441 w[5,2] - 11.83039892442283 w[5,3]
   - 15.61969255885572 w[5,4] - 2.19883879053222 w[5,6]
   - 15.39602368414665 w[5,7] - 11.47478618351826 w[5,8]
   - 3.918145165358823 w[5,9] - 11.04290602393819 w[5,10]
   - 8.777694130085639 w[5,11] - 10.42498079001441 w[5,12]
   - 11.83039892442283 w[5,13] - 15.61969255885572 w[5,14]
   - 2.19883879053222 w[5,16] - 15.39602368414665 w[5,17]
   - 11.47478618351826 w[5,18] - 3.918145165358823 w[5,19]
   - 11.04290602393819 w[5,20] - 8.777694130085639 w[5,21]
   - 7.373516367027557 w[5,22] - 7.373516367027557 w[5,23]
   - 8.978795400474022 w[6,1] - 10.43254719125622 w[6,2]
   - 12.92698372597812 w[6,3] - 16.67768228994561 w[6,4]
   - 2.19883879053222 w[6,5] - 16.62248090200579 w[6,7]
   - 11.96067016010668 w[6,8] - 5.944933253422422 w[6,9]
   - 10.71584078669426 w[6,10] - 10.60998898024581 w[6,11]
   - 10.43254719125622 w[6,12] - 12.92698372597812 w[6,13]
   - 16.67768228994561 w[6,14] - 2.19883879053222 w[6,15]
   - 16.62248090200579 w[6,17] - 11.96067016010668 w[6,18]
   - 5.944933253422422 w[6,19] - 10.71584078669426 w[6,20]
   - 10.60998898024581 w[6,21] - 8.978795400474022 w[6,22]
   - 8.978795400474022 w[6,23] - 8.267686142994348 w[7,1]
   - 9.35008533273758 w[7,2] - 3.767878075007341 w[7,3]
   - 1.448059261536187 w[7,4] - 15.39602368414665 w[7,5]
   - 16.62248090200579 w[7,6] - 6.612697295157725 w[7,8]
   - 12.20638347786994 w[7,9] - 10.85235444854572 w[7,10]
   - 7.977411573211986 w[7,11] - 9.35008533273758 w[7,12]
   - 3.767878075007341 w[7,13] - 1.448059261536187 w[7,14]
   - 15.39602368414665 w[7,15] - 16.62248090200579 w[7,16]
   - 6.612697295157725 w[7,18] - 12.20638347786994 w[7,19]
   - 10.85235444854572 w[7,20] - 7.977411573211986 w[7,21]
   - 8.267686142994348 w[7,22] - 8.267686142994348 w[7,23]
   - 6.748278456541565 w[8,1] - 2.744832300932999 w[8,2]
   - 3.650013429871618 w[8,3] - 5.863519766576599 w[8,4]
   - 11.47478618351826 w[8,5] - 11.96067016010668 w[8,6]
   - 6.612697295157725 w[8,7] - 9.712521891969692 w[8,9]
   - 4.261250108474765 w[8,10] - 8.171099544055711 w[8,11]
   - 2.744832300932999 w[8,12] - 3.650013429871618 w[8,13]
   - 5.863519766576599 w[8,14] - 11.47478618351826 w[8,15]
   - 11.96067016010668 w[8,16] - 6.612697295157725 w[8,17]
   - 9.712521891969692 w[8,19] - 4.261250108474765 w[8,20]
   - 8.171099544055711 w[8,21] - 6.748278456541565 w[8,22]
   - 6.748278456541565 w[8,23] - 3.938714087887019 w[9,1]
   - 9.627913520343157 w[9,2] - 8.971746540533758 w[9,3]
   - 12.67637825119609 w[9,4] - 3.918145165358823 w[9,5]
   - 5.944933253422422 w[9,6] - 12.20638347786994 w[9,7]
   - 9.712521891969692 w[9,8] - 10.77603768359544 w[9,10]
   - 4.944627098584724 w[9,11] - 9.627913520343157 w[9,12]
   - 8.971746540533758 w[9,13] - 12.67637825119609 w[9,14]
   - 3.918145165358823 w[9,15] - 5.944933253422422 w[9,16]
   - 12.20638347786994 w[9,17] - 9.712521891969692 w[9,18]
   - 10.77603768359544 w[9,20] - 4.944627098584724 w[9,21]
   - 3.938714087887019 w[9,22] - 3.938714087887019 w[9,23]
   - 9.179354857933594 w[10,1] - 1.687257120839084 w[10,2]
   - 7.791652027865135 w[10,3] - 9.99197232225618 w[10,4]
   - 11.04290602393819 w[10,5] - 10.71584078669426 w[10,6]
   - 10.85235444854572 w[10,7] - 4.261250108474765 w[10,8]
   - 10.77603768359544 w[10,9] - 11.08817859067009 w[10,11]
   - 1.687257120839084 w[10,12] - 7.791652027865135 w[10,13]
   - 9.99197232225618 w[10,14] - 11.04290602393819 w[10,15]
   - 10.71584078669426 w[10,16] - 10.85235444854572 w[10,17]
   - 4.261250108474765 w[10,18] - 10.77603768359544 w[10,19]
   - 11.08817859067009 w[10,21] - 9.179354857933594 w[10,22]
   - 9.179354857933594 w[10,23] - 2.119489211471819 w[11,1]
   - 9.494112783392445 w[11,2] - 5.690832636266907 w[11,3]
   - 8.804653430672097 w[11,4] - 8.777694130085639 w[11,5]
   - 10.60998898024581 w[11,6] - 7.977411573211986 w[11,7]
   - 8.171099544055711 w[11,8] - 4.944627098584724 w[11,9]
   - 11.08817859067009 w[11,10] - 9.494112783392445 w[11,12]
   - 5.690832636266907 w[11,13] - 8.804653430672097 w[11,14]
   - 8.777694130085639 w[11,15] - 10.60998898024581 w[11,16]
   - 7.977411573211986 w[11,17] - 8.171099544055711 w[11,18]
   - 4.944627098584724 w[11,19] - 11.08817859067009 w[11,20]
   - 2.119489211471819 w[11,22] - 2.119489211471819 w[11,23]
   - 7.653030609056256 w[12,1] - 6.14565516095383 w[12,3]
   - 8.596689262039828 w[12,4] - 10.42498079001441 w[12,5]
   - 10.43254719125622 w[12,6] - 9.35008533273758 w[12,7]
   - 2.744832300932999 w[12,8] - 9.627913520343157 w[12,9]
   - 1.687257120839084 w[12,10] - 9.494112783392445 w[12,11]
   - 6.14565516095383 w[12,13] - 8.596689262039828 w[12,14]
   - 10.42498079001441 w[12,15] - 10.43254719125622 w[12,16]
   - 9.35008533273758 w[12,17] - 2.744832300932999 w[12,18]
   - 9.627913520343157 w[12,19] - 1.687257120839084 w[12,20]
   - 9.494112783392445 w[12,21] - 7.653030609056256 w[12,22]
   - 7.653030609056256 w[12,23] - 5.154385649029794 w[13,1]
   - 6.14565516095383 w[13,2] - 3.789393087568826 w[13,4]
   - 11.83039892442283 w[13,5] - 12.92698372597812 w[13,6]
   - 3.767878075007341 w[13,7] - 3.650013429871618 w[13,8]
   - 8.971746540533758 w[13,9] - 7.791652027865135 w[13,10]
   - 5.690832636266907 w[13,11] - 6.14565516095383 w[13,12]
   - 3.789393087568826 w[13,14] - 11.83039892442283 w[13,15]
   - 12.92698372597812 w[13,16] - 3.767878075007341 w[13,17]
   - 3.650013429871618 w[13,18] - 8.971746540533758 w[13,19]
   - 7.791652027865135 w[13,20] - 5.690832636266907 w[13,21]
   - 5.154385649029794 w[13,22] - 5.154385649029794 w[13,23]
   - 8.773753398145478 w[14,1] - 8.596689262039828 w[14,2]
   - 3.789393087568826 w[14,3] - 15.61969255885572 w[14,5]
   - 16.67768228994561 w[14,6] - 1.448059261536187 w[14,7]
   - 5.863519766576599 w[14,8] - 12.67637825119609 w[14,9]
   - 9.99197232225618 w[14,10] - 8.804653430672097 w[14,11]
   - 8.596689262039828 w[14,12] - 3.789393087568826 w[14,13]
   - 15.61969255885572 w[14,15] - 16.67768228994561 w[14,16]
   - 1.448059261536187 w[14,17] - 5.863519766576599 w[14,18]
   - 12.67637825119609 w[14,19] - 9.99197232225618 w[14,20]
   - 8.804653430672097 w[14,21] - 8.773753398145478 w[14,22]
   - 8.773753398145478 w[14,23] - 7.373516367027557 w[15,1]
   - 10.42498079001441 w[15,2] - 11.83039892442283 w[15,3]
   - 15.61969255885572 w[15,4] - 2.19883879053222 w[15,6]
   - 15.39602368414665 w[15,7] - 11.47478618351826 w[15,8]
   - 3.918145165358823 w[15,9] - 11.04290602393819 w[15,10]
   - 8.777694130085639 w[15,11] - 10.42498079001441 w[15,12]
   - 11.83039892442283 w[15,13] - 15.61969255885572 w[15,14]
   - 2.19883879053222 w[15,16] - 15.39602368414665 w[15,17]
   - 11.47478618351826 w[15,18] - 3.918145165358823 w[15,19]
   - 11.04290602393819 w[15,20] - 8.777694130085639 w[15,21]
   - 7.373516367027557 w[15,22] - 7.373516367027557 w[15,23]
   - 8.978795400474022 w[16,1] - 10.43254719125622 w[16,2]
   - 12.92698372597812 w[16,3] - 16.67768228994561 w[16,4]
   - 2.19883879053222 w[16,5] - 16.62248090200579 w[16,7]
   - 11.96067016010668 w[16,8] - 5.944933253422422 w[16,9]
   - 10.71584078669426 w[16,10] - 10.60998898024581 w[16,11]
   - 10.43254719125622 w[16,12] - 12.92698372597812 w[16,13]
   - 16.67768228994561 w[16,14] - 2.19883879053222 w[16,15]
   - 16.62248090200579 w[16,17] - 11.96067016010668 w[16,18]
   - 5.944933253422422 w[16,19] - 10.71584078669426 w[16,20]
   - 10.60998898024581 w[16,21] - 8.978795400474022 w[16,22]
   - 8.978795400474022 w[16,23] - 8.267686142994348 w[17,1]
   - 9.35008533273758 w[17,2] - 3.767878075007341 w[17,3]
   - 1.448059261536187 w[17,4] - 15.39602368414665 w[17,5]
   - 16.62248090200579 w[17,6] - 6.612697295157725 w[17,8]
   - 12.20638347786994 w[17,9] - 10.85235444854572 w[17,10]
   - 7.977411573211986 w[17,11] - 9.35008533273758 w[17,12]
   - 3.767878075007341 w[17,13] - 1.448059261536187 w[17,14]
   - 15.39602368414665 w[17,15] - 16.62248090200579 w[17,16]
   - 6.612697295157725 w[17,18] - 12.20638347786994 w[17,19]
   - 10.85235444854572 w[17,20] - 7.977411573211986 w[17,21]
   - 8.267686142994348 w[17,22] - 8.267686142994348 w[17,23]
   - 6.748278456541565 w[18,1] - 2.744832300932999 w[18,2]
   - 3.650013429871618 w[18,3] - 5.863519766576599 w[18,4]
   - 11.47478618351826 w[18,5] - 11.96067016010668 w[18,6]
   - 6.612697295157725 w[18,7] - 9.712521891969692 w[18,9]
   - 4.261250108474765 w[18,10] - 8.171099544055711 w[18,11]
   - 2.744832300932999 w[18,12] - 3.650013429871618 w[18,13]
   - 5.863519766576599 w[18,14] - 11.47478618351826 w[18,15]
   - 11.96067016010668 w[18,16] - 6.612697295157725 w[18,17]
   - 9.712521891969692 w[18,19] - 4.261250108474765 w[18,20]
   - 8.171099544055711 w[18,21] - 6.748278456541565 w[18,22]
   - 6.748278456541565 w[18,23] - 3.938714087887019 w[19,1]
   - 9.627913520343157 w[19,2] - 8.971746540533758 w[19,3]
   - 12.67637825119609 w[19,4] - 3.918145165358823 w[19,5]
   - 5.944933253422422 w[19,6] - 12.20638347786994 w[19,7]
   - 9.712521891969692 w[19,8] - 10.77603768359544 w[19,10]
   - 4.944627098584724 w[19,11] - 9.627913520343157 w[19,12]
   - 8.971746540533758 w[19,13] - 12.67637825119609 w[19,14]
   - 3.918145165358823 w[19,15] - 5.944933253422422 w[19,16]
   - 12.20638347786994 w[19,17] - 9.712521891969692 w[19,18]
   - 10.77603768359544 w[19,20] - 4.944627098584724 w[19,21]
   - 3.938714087887019 w[19,22] - 3.938714087887019 w[19,23]
   - 9.179354857933594 w[20,1] - 1.687257120839084 w[20,2]
   - 7.791652027865135 w[20,3] - 9.99197232225618 w[20,4]
   - 11.04290602393819 w[20,5] - 10.71584078669426 w[20,6]
   - 10.85235444854572 w[20,7] - 4.261250108474765 w[20,8]
   - 10.77603768359544 w[20,9] - 11.08817859067009 w[20,11]
   - 1.687257120839084 w[20,12] - 7.791652027865135 w[20,13]
   - 9.99197232225618 w[20,14] - 11.04290602393819 w[20,15]
   - 10.71584078669426 w[20,16] - 10.85235444854572 w[20,17]
   - 4.261250108474765 w[20,18] - 10.77603768359544 w[20,19]
   - 11.08817859067009 w[20,21] - 9.179354857933594 w[20,22]
   - 9.179354857933594 w[20,23] - 2.119489211471819 w[21,1]
   - 9.494112783392445 w[21,2] - 5.690832636266907 w[21,3]
   - 8.804653430672097 w[21,4] - 8.777694130085639 w[21,5]
   - 10.60998898024581 w[21,6] - 7.977411573211986 w[21,7]
   - 8.171099544055711 w[21,8] - 4.944627098584724 w[21,9]
   - 11.08817859067009 w[21,10] - 9.494112783392445 w[21,12]
   - 5.690832636266907 w[21,13] - 8.804653430672097 w[21,14]
   - 8.777694130085639 w[21,15] - 10.60998898024581 w[21,16]
   - 7.977411573211986 w[21,17] - 8.171099544055711 w[21,18]
   - 4.944627098584724 w[21,19] - 11.08817859067009 w[21,20]
   - 2.119489211471819 w[21,22] - 2.119489211471819 w[21,23]
   - 7.653030609056256 w[22,2] - 5.154385649029794 w[22,3]
   - 8.773753398145478 w[22,4] - 7.373516367027557 w[22,5]
   - 8.978795400474022 w[22,6] - 8.267686142994348 w[22,7]
   - 6.748278456541565 w[22,8] - 3.938714087887019 w[22,9]
   - 9.179354857933594 w[22,10] - 2.119489211471819 w[22,11]
   - 7.653030609056256 w[22,12] - 5.154385649029794 w[22,13]
   - 8.773753398145478 w[22,14] - 7.373516367027557 w[22,15]
   - 8.978795400474022 w[22,16] - 8.267686142994348 w[22,17]
   - 6.748278456541565 w[22,18] - 3.938714087887019 w[22,19]
   - 9.179354857933594 w[22,20] - 2.119489211471819 w[22,21]
   - 31.42857142857143 y[2,1] - 62.85714285714285 y[2,2]
   - 48.57142857142857 y[3,1] - 97.14285714285714 y[3,2] - 70 y[4,1]
   - 140 y[4,2] - 70 y[5,1] - 140 y[5,2] - 42.85714285714285 y[6,1]
   - 85.71428571428571 y[6,2] - 62.85714285714285 y[7,1]
   - 125.7142857142857 y[7,2] - 38.57142857142857 y[8,1]
   - 77.14285714285714 y[8,2] - 34.28571428571428 y[9,1]
   - 68.57142857142857 y[9,2] - 48.57142857142857 y[10,1]
   - 97.14285714285714 y[10,2] - 32.85714285714285 y[11,1]
   - 65.71428571428571 y[11,2] + start_service[23] >= 0
 AverageTimeVehices: - 17.85707142113127 x[0,2] - 12.02689984773619 x[0,3]
   - 20.47209126233945 x[0,4] - 17.2048715230643 x[0,5]
   - 20.95052260110605 x[0,6] - 19.29126766698681 x[0,7]
   - 15.74598306526365 x[0,8] - 9.190332871736379 x[0,9]
   - 21.41849466851172 x[0,10] - 4.945474826767578 x[0,11]
   - 17.85707142113127 x[0,12] - 12.02689984773619 x[0,13]
   - 20.47209126233945 x[0,14] - 17.2048715230643 x[0,15]
   - 20.95052260110605 x[0,16] - 19.29126766698681 x[0,17]
   - 15.74598306526365 x[0,18] - 9.190332871736379 x[0,19]
   - 21.41849466851172 x[0,20] - 4.945474826767578 x[0,21]
   - 17.85707142113127 x[1,2] - 12.02689984773619 x[1,3]
   - 20.47209126233945 x[1,4] - 17.2048715230643 x[1,5]
   - 20.95052260110605 x[1,6] - 19.29126766698681 x[1,7]
   - 15.74598306526365 x[1,8] - 9.190332871736379 x[1,9]
   - 21.41849466851172 x[1,10] - 4.945474826767578 x[1,11]
   - 17.85707142113127 x[1,12] - 12.02689984773619 x[1,13]
   - 20.47209126233945 x[1,14] - 17.2048715230643 x[1,15]
   - 20.95052260110605 x[1,16] - 19.29126766698681 x[1,17]
   - 15.74598306526365 x[1,18] - 9.190332871736379 x[1,19]
   - 21.41849466851172 x[1,20] - 4.945474826767578 x[1,21]
   - 17.85707142113127 x[2,1] - 14.3398620422256 x[2,3]
   - 20.05894161142626 x[2,4] - 24.3249551767003 x[2,5]
   - 24.34261011293118 x[2,6] - 21.81686577638769 x[2,7]
   - 6.404608702176997 x[2,8] - 22.46513154746737 x[2,9]
   - 3.936933281957864 x[2,10] - 22.15292982791571 x[2,11]
   - 14.3398620422256 x[2,13] - 20.05894161142626 x[2,14]
   - 24.3249551767003 x[2,15] - 24.34261011293118 x[2,16]
   - 21.81686577638769 x[2,17] - 6.404608702176997 x[2,18]
   - 22.46513154746737 x[2,19] - 3.936933281957864 x[2,20]
   - 22.15292982791571 x[2,21] - 17.85707142113127 x[2,22]
   - 17.85707142113127 x[2,23] - 12.02689984773619 x[3,1]
   - 14.3398620422256 x[3,2] - 8.84191720432726 x[3,4]
   - 27.60426415698662 x[3,5] - 30.16296202728228 x[3,6]
   - 8.791715508350462 x[3,7] - 8.516698003033776 x[3,8]
   - 20.93407526124544 x[3,9] - 18.18052139835198 x[3,10]
   - 13.27860948462278 x[3,11] - 14.3398620422256 x[3,12]
   - 8.84191720432726 x[3,14] - 27.60426415698662 x[3,15]
   - 30.16296202728228 x[3,16] - 8.791715508350462 x[3,17]
   - 8.516698003033776 x[3,18] - 20.93407526124544 x[3,19]
   - 18.18052139835198 x[3,20] - 13.27860948462278 x[3,21]
   - 12.02689984773619 x[3,22] - 12.02689984773619 x[3,23]
   - 20.47209126233945 x[4,1] - 20.05894161142626 x[4,2]
   - 8.84191720432726 x[4,3] - 36.44594930399668 x[4,5]
   - 38.91459200987309 x[4,6] - 3.378804943584437 x[4,7]
   - 13.68154612201207 x[4,8] - 29.57821591945756 x[4,9]
   - 23.31460208526442 x[4,10] - 20.54419133823489 x[4,11]
   - 20.05894161142626 x[4,12] - 8.84191720432726 x[4,13]
   - 36.44594930399668 x[4,15] - 38.91459200987309 x[4,16]
   - 3.378804943584437 x[4,17] - 13.68154612201207 x[4,18]
   - 29.57821591945756 x[4,19] - 23.31460208526442 x[4,20]
   - 20.54419133823489 x[4,21] - 20.47209126233945 x[4,22]
   - 20.47209126233945 x[4,23] - 17.2048715230643 x[5,1]
   - 24.3249551767003 x[5,2] - 27.60426415698662 x[5,3]
   - 36.44594930399668 x[5,4] - 5.13062384457518 x[5,6]
   - 35.92405526300884 x[5,7] - 26.77450109487594 x[5,8]
   - 9.142338719170587 x[5,9] - 25.76678072252245 x[5,10]
   - 20.48128630353316 x[5,11] - 24.3249551767003 x[5,12]
   - 27.60426415698662 x[5,13] - 36.44594930399668 x[5,14]
   - 5.13062384457518 x[5,16] - 35.92405526300884 x[5,17]
   - 26.77450109487594 x[5,18] - 9.142338719170587 x[5,19]
   - 25.76678072252245 x[5,20] - 20.48128630353316 x[5,21]
   - 17.2048715230643 x[5,22] - 17.2048715230643 x[5,23]
   - 20.95052260110605 x[6,1] - 24.34261011293118 x[6,2]
   - 30.16296202728228 x[6,3] - 38.91459200987309 x[6,4]
   - 5.13062384457518 x[6,5] - 38.78578877134684 x[6,7]
   - 27.90823037358226 x[6,8] - 13.87151092465232 x[6,9]
   - 25.0036285022866 x[6,10] - 24.7566409539069 x[6,11]
   - 24.34261011293118 x[6,12] - 30.16296202728228 x[6,13]
   - 38.91459200987309 x[6,14] - 5.13062384457518 x[6,15]
   - 38.78578877134684 x[6,17] - 27.90823037358226 x[6,18]
   - 13.87151092465232 x[6,19] - 25.0036285022866 x[6,20]
   - 24.7566409539069 x[6,21] - 20.95052260110605 x[6,22]
   - 20.95052260110605 x[6,23] - 19.29126766698681 x[7,1]
   - 21.81686577638769 x[7,2] - 8.791715508350462 x[7,3]
   - 3.378804943584437 x[7,4] - 35.92405526300884 x[7,5]
   - 38.78578877134684 x[7,6] - 15.42962702203469 x[7,8]
   - 28.4815614483632 x[7,9] - 25.32216037994002 x[7,10]
   - 18.61396033749464 x[7,11] - 21.81686577638769 x[7,12]
   - 8.791715508350462 x[7,13] - 3.378804943584437 x[7,14]
   - 35.92405526300884 x[7,15] - 38.78578877134684 x[7,16]
   - 15.42962702203469 x[7,18] - 28.4815614483632 x[7,19]
   - 25.32216037994002 x[7,20] - 18.61396033749464 x[7,21]
   - 19.29126766698681 x[7,22] - 19.29126766698681 x[7,23]
   - 15.74598306526365 x[8,1] - 6.404608702176997 x[8,2]
   - 8.516698003033776 x[8,3] - 13.68154612201207 x[8,4]
   - 26.77450109487594 x[8,5] - 27.90823037358226 x[8,6]
   - 15.42962702203469 x[8,7] - 22.66255108126261 x[8,9]
   - 9.942916919774451 x[8,10] - 19.06589893612999 x[8,11]
   - 6.404608702176997 x[8,12] - 8.516698003033776 x[8,13]
   - 13.68154612201207 x[8,14] - 26.77450109487594 x[8,15]
   - 27.90823037358226 x[8,16] - 15.42962702203469 x[8,17]
   - 22.66255108126261 x[8,19] - 9.942916919774451 x[8,20]
   - 19.06589893612999 x[8,21] - 15.74598306526365 x[8,22]
   - 15.74598306526365 x[8,23] - 9.190332871736379 x[9,1]
   - 22.46513154746737 x[9,2] - 20.93407526124544 x[9,3]
   - 29.57821591945756 x[9,4] - 9.142338719170587 x[9,5]
   - 13.87151092465232 x[9,6] - 28.4815614483632 x[9,7]
   - 22.66255108126261 x[9,8] - 25.14408792838936 x[9,10]
   - 11.53746323003102 x[9,11] - 22.46513154746737 x[9,12]
   - 20.93407526124544 x[9,13] - 29.57821591945756 x[9,14]
   - 9.142338719170587 x[9,15] - 13.87151092465232 x[9,16]
   - 28.4815614483632 x[9,17] - 22.66255108126261 x[9,18]
   - 25.14408792838936 x[9,20] - 11.53746323003102 x[9,21]
   - 9.190332871736379 x[9,22] - 9.190332871736379 x[9,23]
   - 21.41849466851172 x[10,1] - 3.936933281957864 x[10,2]
   - 18.18052139835198 x[10,3] - 23.31460208526442 x[10,4]
   - 25.76678072252245 x[10,5] - 25.0036285022866 x[10,6]
   - 25.32216037994002 x[10,7] - 9.942916919774451 x[10,8]
   - 25.14408792838936 x[10,9] - 25.87241671156353 x[10,11]
   - 3.936933281957864 x[10,12] - 18.18052139835198 x[10,13]
   - 23.31460208526442 x[10,14] - 25.76678072252245 x[10,15]
   - 25.0036285022866 x[10,16] - 25.32216037994002 x[10,17]
   - 9.942916919774451 x[10,18] - 25.14408792838936 x[10,19]
   - 25.87241671156353 x[10,21] - 21.41849466851172 x[10,22]
   - 21.41849466851172 x[10,23] - 4.945474826767578 x[11,1]
   - 22.15292982791571 x[11,2] - 13.27860948462278 x[11,3]
   - 20.54419133823489 x[11,4] - 20.48128630353316 x[11,5]
   - 24.7566409539069 x[11,6] - 18.61396033749464 x[11,7]
   - 19.06589893612999 x[11,8] - 11.53746323003102 x[11,9]
   - 25.87241671156353 x[11,10] - 22.15292982791571 x[11,12]
   - 13.27860948462278 x[11,13] - 20.54419133823489 x[11,14]
   - 20.48128630353316 x[11,15] - 24.7566409539069 x[11,16]
   - 18.61396033749464 x[11,17] - 19.06589893612999 x[11,18]
   - 11.53746323003102 x[11,19] - 25.87241671156353 x[11,20]
   - 4.945474826767578 x[11,22] - 4.945474826767578 x[11,23]
   - 17.85707142113127 x[12,1] - 14.3398620422256 x[12,3]
   - 20.05894161142626 x[12,4] - 24.3249551767003 x[12,5]
   - 24.34261011293118 x[12,6] - 21.81686577638769 x[12,7]
   - 6.404608702176997 x[12,8] - 22.46513154746737 x[12,9]
   - 3.936933281957864 x[12,10] - 22.15292982791571 x[12,11]
   - 14.3398620422256 x[12,13] - 20.05894161142626 x[12,14]
   - 24.3249551767003 x[12,15] - 24.34261011293118 x[12,16]
   - 21.81686577638769 x[12,17] - 6.404608702176997 x[12,18]
   - 22.46513154746737 x[12,19] - 3.936933281957864 x[12,20]
   - 22.15292982791571 x[12,21] - 17.85707142113127 x[12,22]
   - 17.85707142113127 x[12,23] - 12.02689984773619 x[13,1]
   - 14.3398620422256 x[13,2] - 8.84191720432726 x[13,4]
   - 27.60426415698662 x[13,5] - 30.16296202728228 x[13,6]
   - 8.791715508350462 x[13,7] - 8.516698003033776 x[13,8]
   - 20.93407526124544 x[13,9] - 18.18052139835198 x[13,10]
   - 13.27860948462278 x[13,11] - 14.3398620422256 x[13,12]
   - 8.84191720432726 x[13,14] - 27.60426415698662 x[13,15]
   - 30.16296202728228 x[13,16] - 8.791715508350462 x[13,17]
   - 8.516698003033776 x[13,18] - 20.93407526124544 x[13,19]
   - 18.18052139835198 x[13,20] - 13.27860948462278 x[13,21]
   - 12.02689984773619 x[13,22] - 12.02689984773619 x[13,23]
   - 20.47209126233945 x[14,1] - 20.05894161142626 x[14,2]
   - 8.84191720432726 x[14,3] - 36.44594930399668 x[14,5]
   - 38.91459200987309 x[14,6] - 3.378804943584437 x[14,7]
   - 13.68154612201207 x[14,8] - 29.57821591945756 x[14,9]
   - 23.31460208526442 x[14,10] - 20.54419133823489 x[14,11]
   - 20.05894161142626 x[14,12] - 8.84191720432726 x[14,13]
   - 36.44594930399668 x[14,15] - 38.91459200987309 x[14,16]
   - 3.378804943584437 x[14,17] - 13.68154612201207 x[14,18]
   - 29.57821591945756 x[14,19] - 23.31460208526442 x[14,20]
   - 20.54419133823489 x[14,21] - 20.47209126233945 x[14,22]
   - 20.47209126233945 x[14,23] - 17.2048715230643 x[15,1]
   - 24.3249551767003 x[15,2] - 27.60426415698662 x[15,3]
   - 36.44594930399668 x[15,4] - 5.13062384457518 x[15,6]
   - 35.92405526300884 x[15,7] - 26.77450109487594 x[15,8]
   - 9.142338719170587 x[15,9] - 25.76678072252245 x[15,10]
   - 20.48128630353316 x[15,11] - 24.3249551767003 x[15,12]
   - 27.60426415698662 x[15,13] - 36.44594930399668 x[15,14]
   - 5.13062384457518 x[15,16] - 35.92405526300884 x[15,17]
   - 26.77450109487594 x[15,18] - 9.142338719170587 x[15,19]
   - 25.76678072252245 x[15,20] - 20.48128630353316 x[15,21]
   - 17.2048715230643 x[15,22] - 17.2048715230643 x[15,23]
   - 20.95052260110605 x[16,1] - 24.34261011293118 x[16,2]
   - 30.16296202728228 x[16,3] - 38.91459200987309 x[16,4]
   - 5.13062384457518 x[16,5] - 38.78578877134684 x[16,7]
   - 27.90823037358226 x[16,8] - 13.87151092465232 x[16,9]
   - 25.0036285022866 x[16,10] - 24.7566409539069 x[16,11]
   - 24.34261011293118 x[16,12] - 30.16296202728228 x[16,13]
   - 38.91459200987309 x[16,14] - 5.13062384457518 x[16,15]
   - 38.78578877134684 x[16,17] - 27.90823037358226 x[16,18]
   - 13.87151092465232 x[16,19] - 25.0036285022866 x[16,20]
   - 24.7566409539069 x[16,21] - 20.95052260110605 x[16,22]
   - 20.95052260110605 x[16,23] - 19.29126766698681 x[17,1]
   - 21.81686577638769 x[17,2] - 8.791715508350462 x[17,3]
   - 3.378804943584437 x[17,4] - 35.92405526300884 x[17,5]
   - 38.78578877134684 x[17,6] - 15.42962702203469 x[17,8]
   - 28.4815614483632 x[17,9] - 25.32216037994002 x[17,10]
   - 18.61396033749464 x[17,11] - 21.81686577638769 x[17,12]
   - 8.791715508350462 x[17,13] - 3.378804943584437 x[17,14]
   - 35.92405526300884 x[17,15] - 38.78578877134684 x[17,16]
   - 15.42962702203469 x[17,18] - 28.4815614483632 x[17,19]
   - 25.32216037994002 x[17,20] - 18.61396033749464 x[17,21]
   - 19.29126766698681 x[17,22] - 19.29126766698681 x[17,23]
   - 15.74598306526365 x[18,1] - 6.404608702176997 x[18,2]
   - 8.516698003033776 x[18,3] - 13.68154612201207 x[18,4]
   - 26.77450109487594 x[18,5] - 27.90823037358226 x[18,6]
   - 15.42962702203469 x[18,7] - 22.66255108126261 x[18,9]
   - 9.942916919774451 x[18,10] - 19.06589893612999 x[18,11]
   - 6.404608702176997 x[18,12] - 8.516698003033776 x[18,13]
   - 13.68154612201207 x[18,14] - 26.77450109487594 x[18,15]
   - 27.90823037358226 x[18,16] - 15.42962702203469 x[18,17]
   - 22.66255108126261 x[18,19] - 9.942916919774451 x[18,20]
   - 19.06589893612999 x[18,21] - 15.74598306526365 x[18,22]
   - 15.74598306526365 x[18,23] - 9.190332871736379 x[19,1]
   - 22.46513154746737 x[19,2] - 20.93407526124544 x[19,3]
   - 29.57821591945756 x[19,4] - 9.142338719170587 x[19,5]
   - 13.87151092465232 x[19,6] - 28.4815614483632 x[19,7]
   - 22.66255108126261 x[19,8] - 25.14408792838936 x[19,10]
   - 11.53746323003102 x[19,11] - 22.46513154746737 x[19,12]
   - 20.93407526124544 x[19,13] - 29.57821591945756 x[19,14]
   - 9.142338719170587 x[19,15] - 13.87151092465232 x[19,16]
   - 28.4815614483632 x[19,17] - 22.66255108126261 x[19,18]
   - 25.14408792838936 x[19,20] - 11.53746323003102 x[19,21]
   - 9.190332871736379 x[19,22] - 9.190332871736379 x[19,23]
   - 21.41849466851172 x[20,1] - 3.936933281957864 x[20,2]
   - 18.18052139835198 x[20,3] - 23.31460208526442 x[20,4]
   - 25.76678072252245 x[20,5] - 25.0036285022866 x[20,6]
   - 25.32216037994002 x[20,7] - 9.942916919774451 x[20,8]
   - 25.14408792838936 x[20,9] - 25.87241671156353 x[20,11]
   - 3.936933281957864 x[20,12] - 18.18052139835198 x[20,13]
   - 23.31460208526442 x[20,14] - 25.76678072252245 x[20,15]
   - 25.0036285022866 x[20,16] - 25.32216037994002 x[20,17]
   - 9.942916919774451 x[20,18] - 25.14408792838936 x[20,19]
   - 25.87241671156353 x[20,21] - 21.41849466851172 x[20,22]
   - 21.41849466851172 x[20,23] - 4.945474826767578 x[21,1]
   - 22.15292982791571 x[21,2] - 13.27860948462278 x[21,3]
   - 20.54419133823489 x[21,4] - 20.48128630353316 x[21,5]
   - 24.7566409539069 x[21,6] - 18.61396033749464 x[21,7]
   - 19.06589893612999 x[21,8] - 11.53746323003102 x[21,9]
   - 25.87241671156353 x[21,10] - 22.15292982791571 x[21,12]
   - 13.27860948462278 x[21,13] - 20.54419133823489 x[21,14]
   - 20.48128630353316 x[21,15] - 24.7566409539069 x[21,16]
   - 18.61396033749464 x[21,17] - 19.06589893612999 x[21,18]
   - 11.53746323003102 x[21,19] - 25.87241671156353 x[21,20]
   - 4.945474826767578 x[21,22] - 4.945474826767578 x[21,23]
   - 17.85707142113127 x[22,2] - 12.02689984773619 x[22,3]
   - 20.47209126233945 x[22,4] - 17.2048715230643 x[22,5]
   - 20.95052260110605 x[22,6] - 19.29126766698681 x[22,7]
   - 15.74598306526365 x[22,8] - 9.190332871736379 x[22,9]
   - 21.41849466851172 x[22,10] - 4.945474826767578 x[22,11]
   - 17.85707142113127 x[22,12] - 12.02689984773619 x[22,13]
   - 20.47209126233945 x[22,14] - 17.2048715230643 x[22,15]
   - 20.95052260110605 x[22,16] - 19.29126766698681 x[22,17]
   - 15.74598306526365 x[22,18] - 9.190332871736379 x[22,19]
   - 21.41849466851172 x[22,20] - 4.945474826767578 x[22,21]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[0]: - 107.1424285267876 x[0,2]
   - 72.16139908641712 x[0,3] - 122.8325475740367 x[0,4]
   - 103.2292291383858 x[0,5] - 125.7031356066363 x[0,6]
   - 115.7476060019209 x[0,7] - 94.47589839158192 x[0,8]
   - 55.14199723041828 x[0,9] - 128.5109680110703 x[0,10]
   - 29.67284896060547 x[0,11] - 107.1424285267876 x[0,12]
   - 72.16139908641712 x[0,13] - 122.8325475740367 x[0,14]
   - 103.2292291383858 x[0,15] - 125.7031356066363 x[0,16]
   - 115.7476060019209 x[0,17] - 94.47589839158192 x[0,18]
   - 55.14199723041828 x[0,19] - 128.5109680110703 x[0,20]
   - 29.67284896060547 x[0,21] - service_time[0] - start_service[0]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[1]: - 107.1424285267876 x[1,2]
   - 72.16139908641712 x[1,3] - 122.8325475740367 x[1,4]
   - 103.2292291383858 x[1,5] - 125.7031356066363 x[1,6]
   - 115.7476060019209 x[1,7] - 94.47589839158192 x[1,8]
   - 55.14199723041828 x[1,9] - 128.5109680110703 x[1,10]
   - 29.67284896060547 x[1,11] - 107.1424285267876 x[1,12]
   - 72.16139908641712 x[1,13] - 122.8325475740367 x[1,14]
   - 103.2292291383858 x[1,15] - 125.7031356066363 x[1,16]
   - 115.7476060019209 x[1,17] - 94.47589839158192 x[1,18]
   - 55.14199723041828 x[1,19] - 128.5109680110703 x[1,20]
   - 29.67284896060547 x[1,21] - service_time[1] - start_service[1]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[2]: - 53.5712142633938 x[2,1]
   - 53.5712142633938 x[2,2] - 79.10028566988538 x[2,3]
   - 121.5930986212971 x[2,4] - 124.5894800992938 x[2,5]
   - 135.8793981421117 x[2,6] - 123.3244003301235 x[2,7]
   - 66.45177530232195 x[2,8] - 94.96639325761123 x[2,9]
   - 76.06628385140876 x[2,10] - 81.29521396404986 x[2,11]
   - 53.5712142633938 x[2,12] - 79.10028566988538 x[2,13]
   - 121.5930986212971 x[2,14] - 124.5894800992938 x[2,15]
   - 135.8793981421117 x[2,16] - 123.3244003301235 x[2,17]
   - 66.45177530232195 x[2,18] - 94.96639325761123 x[2,19]
   - 76.06628385140876 x[2,20] - 81.29521396404986 x[2,21]
   - 53.5712142633938 x[2,22] - 53.5712142633938 x[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= 0
 LowerBoundRoutingVehicle[3]: - 36.08069954320856 x[3,1]
   - 96.59080039007061 x[3,2] - 36.08069954320856 x[3,3]
   - 87.94202540000012 x[3,4] - 134.4274070401528 x[3,5]
   - 153.340453885165 x[3,6] - 84.24894952601183 x[3,7]
   - 72.78804320489229 x[3,8] - 90.37322439894545 x[3,9]
   - 118.7970482005911 x[3,10] - 54.67225293417108 x[3,11]
   - 96.59080039007061 x[3,12] - 36.08069954320856 x[3,13]
   - 87.94202540000012 x[3,14] - 134.4274070401528 x[3,15]
   - 153.340453885165 x[3,16] - 84.24894952601183 x[3,17]
   - 72.78804320489229 x[3,18] - 90.37322439894545 x[3,19]
   - 118.7970482005911 x[3,20] - 54.67225293417108 x[3,21]
   - 36.08069954320856 x[3,22] - 36.08069954320856 x[3,23]
   - service_time[3] - start_service[3] + start_service[23] >= 0
 LowerBoundRoutingVehicle[4]: - 61.41627378701835 x[4,1]
   - 113.7480390976726 x[4,2] - 62.60645115619034 x[4,3]
   - 61.41627378701835 x[4,4] - 160.9524624811829 x[4,5]
   - 179.5953438329374 x[4,6] - 68.01021783171375 x[4,7]
   - 88.28258756182716 x[4,8] - 116.3056463735818 x[4,9]
   - 134.1992902613284 x[4,10] - 76.46899849500741 x[4,11]
   - 113.7480390976726 x[4,12] - 62.60645115619034 x[4,13]
   - 61.41627378701835 x[4,14] - 160.9524624811829 x[4,15]
   - 179.5953438329374 x[4,16] - 68.01021783171375 x[4,17]
   - 88.28258756182716 x[4,18] - 116.3056463735818 x[4,19]
   - 134.1992902613284 x[4,20] - 76.46899849500741 x[4,21]
   - 61.41627378701835 x[4,22] - 61.41627378701835 x[4,23]
   - service_time[4] - start_service[4] + start_service[23] >= 0
 LowerBoundRoutingVehicle[5]: - 51.6146145691929 x[5,1]
   - 126.5460797934947 x[5,2] - 118.8934920141684 x[5,3]
   - 170.7541216990084 x[5,4] - 51.6146145691929 x[5,5]
   - 78.2434393370437 x[5,6] - 165.645968789987 x[5,7]
   - 127.5614524804188 x[5,8] - 54.9980147727209 x[5,9]
   - 141.5558261731025 x[5,10] - 76.28028339090221 x[5,11]
   - 126.5460797934947 x[5,12] - 118.8934920141684 x[5,13]
   - 170.7541216990084 x[5,14] - 51.6146145691929 x[5,15]
   - 78.2434393370437 x[5,16] - 165.645968789987 x[5,17]
   - 127.5614524804188 x[5,18] - 54.9980147727209 x[5,19]
   - 141.5558261731025 x[5,20] - 76.28028339090221 x[5,21]
   - 51.6146145691929 x[5,22] - 51.6146145691929 x[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= 0
 LowerBoundRoutingVehicle[6]: - 62.85156780331815 x[6,1]
   - 126.5990446021873 x[6,2] - 126.5695856250554 x[6,3]
   - 178.1600498166376 x[6,4] - 67.00648610291844 x[6,5]
   - 62.85156780331815 x[6,6] - 174.231169315001 x[6,7]
   - 130.9626403165377 x[6,8] - 69.1855313891661 x[6,9]
   - 139.266369512395 x[6,10] - 89.10634734202344 x[6,11]
   - 126.5990446021873 x[6,12] - 126.5695856250554 x[6,13]
   - 178.1600498166376 x[6,14] - 67.00648610291844 x[6,15]
   - 62.85156780331815 x[6,16] - 174.231169315001 x[6,17]
   - 130.9626403165377 x[6,18] - 69.1855313891661 x[6,19]
   - 139.266369512395 x[6,20] - 89.10634734202344 x[6,21]
   - 62.85156780331815 x[6,22] - 62.85156780331815 x[6,23]
   - service_time[6] - start_service[6] + start_service[23] >= 0
 LowerBoundRoutingVehicle[7]: - 57.87380300096044 x[7,1]
   - 119.0218115925569 x[7,2] - 62.45584606825994 x[7,3]
   - 71.55268861777165 x[7,4] - 159.3867803582194 x[7,5]
   - 179.2089341173587 x[7,6] - 57.87380300096044 x[7,7]
   - 93.52683026189504 x[7,8] - 113.0156829602987 x[7,9]
   - 140.2219651453552 x[7,10] - 70.67830549278665 x[7,11]
   - 119.0218115925569 x[7,12] - 62.45584606825994 x[7,13]
   - 71.55268861777165 x[7,14] - 159.3867803582194 x[7,15]
   - 179.2089341173587 x[7,16] - 57.87380300096044 x[7,17]
   - 93.52683026189504 x[7,18] - 113.0156829602987 x[7,19]
   - 140.2219651453552 x[7,20] - 70.67830549278665 x[7,21]
   - 57.87380300096044 x[7,22] - 57.87380300096044 x[7,23]
   - service_time[7] - start_service[7] + start_service[23] >= 0
 LowerBoundRoutingVehicle[8]: - 47.23794919579096 x[8,1]
   - 72.78504036992479 x[8,2] - 61.63079355230989 x[8,3]
   - 102.4609121530546 x[8,4] - 131.9381178538207 x[8,5]
   - 146.5762589240649 x[8,6] - 104.1626840670645 x[8,7]
   - 47.23794919579096 x[8,8] - 95.55865185899698 x[8,9]
   - 94.08423476485852 x[8,10] - 72.03412128869272 x[8,11]
   - 72.78504036992479 x[8,12] - 61.63079355230989 x[8,13]
   - 102.4609121530546 x[8,14] - 131.9381178538207 x[8,15]
   - 146.5762589240649 x[8,16] - 104.1626840670645 x[8,17]
   - 47.23794919579096 x[8,18] - 95.55865185899698 x[8,19]
   - 94.08423476485852 x[8,20] - 72.03412128869272 x[8,21]
   - 47.23794919579096 x[8,22] - 47.23794919579096 x[8,23]
   - service_time[8] - start_service[8] + start_service[23] >= 0
 LowerBoundRoutingVehicle[9]: - 27.57099861520914 x[9,1]
   - 120.9666089057959 x[9,2] - 98.88292532694487 x[9,3]
   - 150.150921545391 x[9,4] - 79.04163072670467 x[9,5]
   - 104.4661005772751 x[9,6] - 143.31848734605 x[9,7]
   - 115.2256024395788 x[9,8] - 27.57099861520914 x[9,9]
   - 139.6877477907033 x[9,10] - 49.4488141703958 x[9,11]
   - 120.9666089057959 x[9,12] - 98.88292532694487 x[9,13]
   - 150.150921545391 x[9,14] - 79.04163072670467 x[9,15]
   - 104.4661005772751 x[9,16] - 143.31848734605 x[9,17]
   - 115.2256024395788 x[9,18] - 27.57099861520914 x[9,19]
   - 139.6877477907033 x[9,20] - 49.4488141703958 x[9,21]
   - 27.57099861520914 x[9,22] - 27.57099861520914 x[9,23]
   - service_time[9] - start_service[9] + start_service[23] >= 0
 LowerBoundRoutingVehicle[10]: - 64.25548400553517 x[10,1]
   - 65.38201410926739 x[10,2] - 90.62226373826451 x[10,3]
   - 131.3600800428116 x[10,4] - 128.9149567367603 x[10,5]
   - 137.862453310178 x[10,6] - 133.8402841407805 x[10,7]
   - 77.06669995511432 x[10,8] - 103.0032624003772 x[10,9]
   - 64.25548400553517 x[10,10] - 92.45367461499335 x[10,11]
   - 65.38201410926739 x[10,12] - 90.62226373826451 x[10,13]
   - 131.3600800428116 x[10,14] - 128.9149567367603 x[10,15]
   - 137.862453310178 x[10,16] - 133.8402841407805 x[10,17]
   - 77.06669995511432 x[10,18] - 103.0032624003772 x[10,19]
   - 64.25548400553517 x[10,20] - 92.45367461499335 x[10,21]
   - 64.25548400553517 x[10,22] - 64.25548400553517 x[10,23]
   - service_time[10] - start_service[10] + start_service[23] >= 0
 LowerBoundRoutingVehicle[11]: - 14.83642448030273 x[11,1]
   - 120.0300037471409 x[11,2] - 75.91652799707691 x[11,3]
   - 123.048847801723 x[11,4] - 113.0584734797924 x[11,5]
   - 137.1214906650389 x[11,6] - 113.7156840134443 x[11,7]
   - 104.4356460041809 x[11,8] - 62.1833883053022 x[11,9]
   - 141.8727341402258 x[11,10] - 14.83642448030273 x[11,11]
   - 120.0300037471409 x[11,12] - 75.91652799707691 x[11,13]
   - 123.048847801723 x[11,14] - 113.0584734797924 x[11,15]
   - 137.1214906650389 x[11,16] - 113.7156840134443 x[11,17]
   - 104.4356460041809 x[11,18] - 62.1833883053022 x[11,19]
   - 141.8727341402258 x[11,20] - 14.83642448030273 x[11,21]
   - 14.83642448030273 x[11,22] - 14.83642448030273 x[11,23]
   - service_time[11] - start_service[11] + start_service[23] >= 0
 LowerBoundRoutingVehicle[12]: - 53.5712142633938 x[12,1]
   - 53.5712142633938 x[12,2] - 79.10028566988538 x[12,3]
   - 121.5930986212971 x[12,4] - 124.5894800992938 x[12,5]
   - 135.8793981421117 x[12,6] - 123.3244003301235 x[12,7]
   - 66.45177530232195 x[12,8] - 94.96639325761123 x[12,9]
   - 76.06628385140876 x[12,10] - 81.29521396404986 x[12,11]
   - 53.5712142633938 x[12,12] - 79.10028566988538 x[12,13]
   - 121.5930986212971 x[12,14] - 124.5894800992938 x[12,15]
   - 135.8793981421117 x[12,16] - 123.3244003301235 x[12,17]
   - 66.45177530232195 x[12,18] - 94.96639325761123 x[12,19]
   - 76.06628385140876 x[12,20] - 81.29521396404986 x[12,21]
   - 53.5712142633938 x[12,22] - 53.5712142633938 x[12,23]
   - service_time[12] - start_service[12] + start_service[23] >= 0
 LowerBoundRoutingVehicle[13]: - 36.08069954320856 x[13,1]
   - 96.59080039007061 x[13,2] - 36.08069954320856 x[13,3]
   - 87.94202540000012 x[13,4] - 134.4274070401528 x[13,5]
   - 153.340453885165 x[13,6] - 84.24894952601183 x[13,7]
   - 72.78804320489229 x[13,8] - 90.37322439894545 x[13,9]
   - 118.7970482005911 x[13,10] - 54.67225293417108 x[13,11]
   - 96.59080039007061 x[13,12] - 36.08069954320856 x[13,13]
   - 87.94202540000012 x[13,14] - 134.4274070401528 x[13,15]
   - 153.340453885165 x[13,16] - 84.24894952601183 x[13,17]
   - 72.78804320489229 x[13,18] - 90.37322439894545 x[13,19]
   - 118.7970482005911 x[13,20] - 54.67225293417108 x[13,21]
   - 36.08069954320856 x[13,22] - 36.08069954320856 x[13,23]
   - service_time[13] - start_service[13] + start_service[23] >= 0
 LowerBoundRoutingVehicle[14]: - 61.41627378701835 x[14,1]
   - 113.7480390976726 x[14,2] - 62.60645115619034 x[14,3]
   - 61.41627378701835 x[14,4] - 160.9524624811829 x[14,5]
   - 179.5953438329374 x[14,6] - 68.01021783171375 x[14,7]
   - 88.28258756182716 x[14,8] - 116.3056463735818 x[14,9]
   - 134.1992902613284 x[14,10] - 76.46899849500741 x[14,11]
   - 113.7480390976726 x[14,12] - 62.60645115619034 x[14,13]
   - 61.41627378701835 x[14,14] - 160.9524624811829 x[14,15]
   - 179.5953438329374 x[14,16] - 68.01021783171375 x[14,17]
   - 88.28258756182716 x[14,18] - 116.3056463735818 x[14,19]
   - 134.1992902613284 x[14,20] - 76.46899849500741 x[14,21]
   - 61.41627378701835 x[14,22] - 61.41627378701835 x[14,23]
   - service_time[14] - start_service[14] + start_service[23] >= 0
 LowerBoundRoutingVehicle[15]: - 51.6146145691929 x[15,1]
   - 126.5460797934947 x[15,2] - 118.8934920141684 x[15,3]
   - 170.7541216990084 x[15,4] - 51.6146145691929 x[15,5]
   - 78.2434393370437 x[15,6] - 165.645968789987 x[15,7]
   - 127.5614524804188 x[15,8] - 54.9980147727209 x[15,9]
   - 141.5558261731025 x[15,10] - 76.28028339090221 x[15,11]
   - 126.5460797934947 x[15,12] - 118.8934920141684 x[15,13]
   - 170.7541216990084 x[15,14] - 51.6146145691929 x[15,15]
   - 78.2434393370437 x[15,16] - 165.645968789987 x[15,17]
   - 127.5614524804188 x[15,18] - 54.9980147727209 x[15,19]
   - 141.5558261731025 x[15,20] - 76.28028339090221 x[15,21]
   - 51.6146145691929 x[15,22] - 51.6146145691929 x[15,23]
   - service_time[15] - start_service[15] + start_service[23] >= 0
 LowerBoundRoutingVehicle[16]: - 62.85156780331815 x[16,1]
   - 126.5990446021873 x[16,2] - 126.5695856250554 x[16,3]
   - 178.1600498166376 x[16,4] - 67.00648610291844 x[16,5]
   - 62.85156780331815 x[16,6] - 174.231169315001 x[16,7]
   - 130.9626403165377 x[16,8] - 69.1855313891661 x[16,9]
   - 139.266369512395 x[16,10] - 89.10634734202344 x[16,11]
   - 126.5990446021873 x[16,12] - 126.5695856250554 x[16,13]
   - 178.1600498166376 x[16,14] - 67.00648610291844 x[16,15]
   - 62.85156780331815 x[16,16] - 174.231169315001 x[16,17]
   - 130.9626403165377 x[16,18] - 69.1855313891661 x[16,19]
   - 139.266369512395 x[16,20] - 89.10634734202344 x[16,21]
   - 62.85156780331815 x[16,22] - 62.85156780331815 x[16,23]
   - service_time[16] - start_service[16] + start_service[23] >= 0
 LowerBoundRoutingVehicle[17]: - 57.87380300096044 x[17,1]
   - 119.0218115925569 x[17,2] - 62.45584606825994 x[17,3]
   - 71.55268861777165 x[17,4] - 159.3867803582194 x[17,5]
   - 179.2089341173587 x[17,6] - 57.87380300096044 x[17,7]
   - 93.52683026189504 x[17,8] - 113.0156829602987 x[17,9]
   - 140.2219651453552 x[17,10] - 70.67830549278665 x[17,11]
   - 119.0218115925569 x[17,12] - 62.45584606825994 x[17,13]
   - 71.55268861777165 x[17,14] - 159.3867803582194 x[17,15]
   - 179.2089341173587 x[17,16] - 57.87380300096044 x[17,17]
   - 93.52683026189504 x[17,18] - 113.0156829602987 x[17,19]
   - 140.2219651453552 x[17,20] - 70.67830549278665 x[17,21]
   - 57.87380300096044 x[17,22] - 57.87380300096044 x[17,23]
   - service_time[17] - start_service[17] + start_service[23] >= 0
 LowerBoundRoutingVehicle[18]: - 47.23794919579096 x[18,1]
   - 72.78504036992479 x[18,2] - 61.63079355230989 x[18,3]
   - 102.4609121530546 x[18,4] - 131.9381178538207 x[18,5]
   - 146.5762589240649 x[18,6] - 104.1626840670645 x[18,7]
   - 47.23794919579096 x[18,8] - 95.55865185899698 x[18,9]
   - 94.08423476485852 x[18,10] - 72.03412128869272 x[18,11]
   - 72.78504036992479 x[18,12] - 61.63079355230989 x[18,13]
   - 102.4609121530546 x[18,14] - 131.9381178538207 x[18,15]
   - 146.5762589240649 x[18,16] - 104.1626840670645 x[18,17]
   - 47.23794919579096 x[18,18] - 95.55865185899698 x[18,19]
   - 94.08423476485852 x[18,20] - 72.03412128869272 x[18,21]
   - 47.23794919579096 x[18,22] - 47.23794919579096 x[18,23]
   - service_time[18] - start_service[18] + start_service[23] >= 0
 LowerBoundRoutingVehicle[19]: - 27.57099861520914 x[19,1]
   - 120.9666089057959 x[19,2] - 98.88292532694487 x[19,3]
   - 150.150921545391 x[19,4] - 79.04163072670467 x[19,5]
   - 104.4661005772751 x[19,6] - 143.31848734605 x[19,7]
   - 115.2256024395788 x[19,8] - 27.57099861520914 x[19,9]
   - 139.6877477907033 x[19,10] - 49.4488141703958 x[19,11]
   - 120.9666089057959 x[19,12] - 98.88292532694487 x[19,13]
   - 150.150921545391 x[19,14] - 79.04163072670467 x[19,15]
   - 104.4661005772751 x[19,16] - 143.31848734605 x[19,17]
   - 115.2256024395788 x[19,18] - 27.57099861520914 x[19,19]
   - 139.6877477907033 x[19,20] - 49.4488141703958 x[19,21]
   - 27.57099861520914 x[19,22] - 27.57099861520914 x[19,23]
   - service_time[19] - start_service[19] + start_service[23] >= 0
 LowerBoundRoutingVehicle[20]: - 64.25548400553517 x[20,1]
   - 65.38201410926739 x[20,2] - 90.62226373826451 x[20,3]
   - 131.3600800428116 x[20,4] - 128.9149567367603 x[20,5]
   - 137.862453310178 x[20,6] - 133.8402841407805 x[20,7]
   - 77.06669995511432 x[20,8] - 103.0032624003772 x[20,9]
   - 64.25548400553517 x[20,10] - 92.45367461499335 x[20,11]
   - 65.38201410926739 x[20,12] - 90.62226373826451 x[20,13]
   - 131.3600800428116 x[20,14] - 128.9149567367603 x[20,15]
   - 137.862453310178 x[20,16] - 133.8402841407805 x[20,17]
   - 77.06669995511432 x[20,18] - 103.0032624003772 x[20,19]
   - 64.25548400553517 x[20,20] - 92.45367461499335 x[20,21]
   - 64.25548400553517 x[20,22] - 64.25548400553517 x[20,23]
   - service_time[20] - start_service[20] + start_service[23] >= 0
 LowerBoundRoutingVehicle[21]: - 14.83642448030273 x[21,1]
   - 120.0300037471409 x[21,2] - 75.91652799707691 x[21,3]
   - 123.048847801723 x[21,4] - 113.0584734797924 x[21,5]
   - 137.1214906650389 x[21,6] - 113.7156840134443 x[21,7]
   - 104.4356460041809 x[21,8] - 62.1833883053022 x[21,9]
   - 141.8727341402258 x[21,10] - 14.83642448030273 x[21,11]
   - 120.0300037471409 x[21,12] - 75.91652799707691 x[21,13]
   - 123.048847801723 x[21,14] - 113.0584734797924 x[21,15]
   - 137.1214906650389 x[21,16] - 113.7156840134443 x[21,17]
   - 104.4356460041809 x[21,18] - 62.1833883053022 x[21,19]
   - 141.8727341402258 x[21,20] - 14.83642448030273 x[21,21]
   - 14.83642448030273 x[21,22] - 14.83642448030273 x[21,23]
   - service_time[21] - start_service[21] + start_service[23] >= 0
 LowerBoundRoutingVehicle[22]: - 107.1424285267876 x[22,2]
   - 72.16139908641712 x[22,3] - 122.8325475740367 x[22,4]
   - 103.2292291383858 x[22,5] - 125.7031356066363 x[22,6]
   - 115.7476060019209 x[22,7] - 94.47589839158192 x[22,8]
   - 55.14199723041828 x[22,9] - 128.5109680110703 x[22,10]
   - 29.67284896060547 x[22,11] - 107.1424285267876 x[22,12]
   - 72.16139908641712 x[22,13] - 122.8325475740367 x[22,14]
   - 103.2292291383858 x[22,15] - 125.7031356066363 x[22,16]
   - 115.7476060019209 x[22,17] - 94.47589839158192 x[22,18]
   - 55.14199723041828 x[22,19] - 128.5109680110703 x[22,20]
   - 29.67284896060547 x[22,21] - service_time[22] - start_service[22]
   + start_service[23] >= 0
 LowerBoundArrivalVehicle[1]: - 107.1424285267876 x[2,1]
   - 72.16139908641712 x[3,1] - 122.8325475740367 x[4,1]
   - 103.2292291383858 x[5,1] - 125.7031356066363 x[6,1]
   - 115.7476060019209 x[7,1] - 94.47589839158192 x[8,1]
   - 55.14199723041828 x[9,1] - 128.5109680110703 x[10,1]
   - 29.67284896060547 x[11,1] - 107.1424285267876 x[12,1]
   - 72.16139908641712 x[13,1] - 122.8325475740367 x[14,1]
   - 103.2292291383858 x[15,1] - 125.7031356066363 x[16,1]
   - 115.7476060019209 x[17,1] - 94.47589839158192 x[18,1]
   - 55.14199723041828 x[19,1] - 128.5109680110703 x[20,1]
   - 29.67284896060547 x[21,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 53.5712142633938 x[0,2]
   - 53.5712142633938 x[1,2] - 53.5712142633938 x[2,2]
   - 79.10028566988538 x[3,2] - 121.5930986212971 x[4,2]
   - 124.5894800992938 x[5,2] - 135.8793981421117 x[6,2]
   - 123.3244003301235 x[7,2] - 66.45177530232195 x[8,2]
   - 94.96639325761123 x[9,2] - 76.06628385140876 x[10,2]
   - 81.29521396404986 x[11,2] - 53.5712142633938 x[12,2]
   - 79.10028566988538 x[13,2] - 121.5930986212971 x[14,2]
   - 124.5894800992938 x[15,2] - 135.8793981421117 x[16,2]
   - 123.3244003301235 x[17,2] - 66.45177530232195 x[18,2]
   - 94.96639325761123 x[19,2] - 76.06628385140876 x[20,2]
   - 81.29521396404986 x[21,2] - 53.5712142633938 x[22,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 36.08069954320856 x[0,3]
   - 36.08069954320856 x[1,3] - 96.59080039007061 x[2,3]
   - 36.08069954320856 x[3,3] - 87.94202540000012 x[4,3]
   - 134.4274070401528 x[5,3] - 153.340453885165 x[6,3]
   - 84.24894952601183 x[7,3] - 72.78804320489229 x[8,3]
   - 90.37322439894545 x[9,3] - 118.7970482005911 x[10,3]
   - 54.67225293417108 x[11,3] - 96.59080039007061 x[12,3]
   - 36.08069954320856 x[13,3] - 87.94202540000012 x[14,3]
   - 134.4274070401528 x[15,3] - 153.340453885165 x[16,3]
   - 84.24894952601183 x[17,3] - 72.78804320489229 x[18,3]
   - 90.37322439894545 x[19,3] - 118.7970482005911 x[20,3]
   - 54.67225293417108 x[21,3] - 36.08069954320856 x[22,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 61.41627378701835 x[0,4]
   - 61.41627378701835 x[1,4] - 113.7480390976726 x[2,4]
   - 62.60645115619034 x[3,4] - 61.41627378701835 x[4,4]
   - 160.9524624811829 x[5,4] - 179.5953438329374 x[6,4]
   - 68.01021783171375 x[7,4] - 88.28258756182716 x[8,4]
   - 116.3056463735818 x[9,4] - 134.1992902613284 x[10,4]
   - 76.46899849500741 x[11,4] - 113.7480390976726 x[12,4]
   - 62.60645115619034 x[13,4] - 61.41627378701835 x[14,4]
   - 160.9524624811829 x[15,4] - 179.5953438329374 x[16,4]
   - 68.01021783171375 x[17,4] - 88.28258756182716 x[18,4]
   - 116.3056463735818 x[19,4] - 134.1992902613284 x[20,4]
   - 76.46899849500741 x[21,4] - 61.41627378701835 x[22,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 51.6146145691929 x[0,5]
   - 51.6146145691929 x[1,5] - 126.5460797934947 x[2,5]
   - 118.8934920141684 x[3,5] - 170.7541216990084 x[4,5]
   - 51.6146145691929 x[5,5] - 78.2434393370437 x[6,5]
   - 165.645968789987 x[7,5] - 127.5614524804188 x[8,5]
   - 54.9980147727209 x[9,5] - 141.5558261731025 x[10,5]
   - 76.28028339090221 x[11,5] - 126.5460797934947 x[12,5]
   - 118.8934920141684 x[13,5] - 170.7541216990084 x[14,5]
   - 51.6146145691929 x[15,5] - 78.2434393370437 x[16,5]
   - 165.645968789987 x[17,5] - 127.5614524804188 x[18,5]
   - 54.9980147727209 x[19,5] - 141.5558261731025 x[20,5]
   - 76.28028339090221 x[21,5] - 51.6146145691929 x[22,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 62.85156780331815 x[0,6]
   - 62.85156780331815 x[1,6] - 126.5990446021873 x[2,6]
   - 126.5695856250554 x[3,6] - 178.1600498166376 x[4,6]
   - 67.00648610291844 x[5,6] - 62.85156780331815 x[6,6]
   - 174.231169315001 x[7,6] - 130.9626403165377 x[8,6]
   - 69.1855313891661 x[9,6] - 139.266369512395 x[10,6]
   - 89.10634734202344 x[11,6] - 126.5990446021873 x[12,6]
   - 126.5695856250554 x[13,6] - 178.1600498166376 x[14,6]
   - 67.00648610291844 x[15,6] - 62.85156780331815 x[16,6]
   - 174.231169315001 x[17,6] - 130.9626403165377 x[18,6]
   - 69.1855313891661 x[19,6] - 139.266369512395 x[20,6]
   - 89.10634734202344 x[21,6] - 62.85156780331815 x[22,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 57.87380300096044 x[0,7]
   - 57.87380300096044 x[1,7] - 119.0218115925569 x[2,7]
   - 62.45584606825994 x[3,7] - 71.55268861777165 x[4,7]
   - 159.3867803582194 x[5,7] - 179.2089341173587 x[6,7]
   - 57.87380300096044 x[7,7] - 93.52683026189504 x[8,7]
   - 113.0156829602987 x[9,7] - 140.2219651453552 x[10,7]
   - 70.67830549278665 x[11,7] - 119.0218115925569 x[12,7]
   - 62.45584606825994 x[13,7] - 71.55268861777165 x[14,7]
   - 159.3867803582194 x[15,7] - 179.2089341173587 x[16,7]
   - 57.87380300096044 x[17,7] - 93.52683026189504 x[18,7]
   - 113.0156829602987 x[19,7] - 140.2219651453552 x[20,7]
   - 70.67830549278665 x[21,7] - 57.87380300096044 x[22,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 47.23794919579096 x[0,8]
   - 47.23794919579096 x[1,8] - 72.78504036992479 x[2,8]
   - 61.63079355230989 x[3,8] - 102.4609121530546 x[4,8]
   - 131.9381178538207 x[5,8] - 146.5762589240649 x[6,8]
   - 104.1626840670645 x[7,8] - 47.23794919579096 x[8,8]
   - 95.55865185899698 x[9,8] - 94.08423476485852 x[10,8]
   - 72.03412128869272 x[11,8] - 72.78504036992479 x[12,8]
   - 61.63079355230989 x[13,8] - 102.4609121530546 x[14,8]
   - 131.9381178538207 x[15,8] - 146.5762589240649 x[16,8]
   - 104.1626840670645 x[17,8] - 47.23794919579096 x[18,8]
   - 95.55865185899698 x[19,8] - 94.08423476485852 x[20,8]
   - 72.03412128869272 x[21,8] - 47.23794919579096 x[22,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 27.57099861520914 x[0,9]
   - 27.57099861520914 x[1,9] - 120.9666089057959 x[2,9]
   - 98.88292532694487 x[3,9] - 150.150921545391 x[4,9]
   - 79.04163072670467 x[5,9] - 104.4661005772751 x[6,9]
   - 143.31848734605 x[7,9] - 115.2256024395788 x[8,9]
   - 27.57099861520914 x[9,9] - 139.6877477907033 x[10,9]
   - 49.4488141703958 x[11,9] - 120.9666089057959 x[12,9]
   - 98.88292532694487 x[13,9] - 150.150921545391 x[14,9]
   - 79.04163072670467 x[15,9] - 104.4661005772751 x[16,9]
   - 143.31848734605 x[17,9] - 115.2256024395788 x[18,9]
   - 27.57099861520914 x[19,9] - 139.6877477907033 x[20,9]
   - 49.4488141703958 x[21,9] - 27.57099861520914 x[22,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 64.25548400553517 x[0,10]
   - 64.25548400553517 x[1,10] - 65.38201410926739 x[2,10]
   - 90.62226373826451 x[3,10] - 131.3600800428116 x[4,10]
   - 128.9149567367603 x[5,10] - 137.862453310178 x[6,10]
   - 133.8402841407805 x[7,10] - 77.06669995511432 x[8,10]
   - 103.0032624003772 x[9,10] - 64.25548400553517 x[10,10]
   - 92.45367461499335 x[11,10] - 65.38201410926739 x[12,10]
   - 90.62226373826451 x[13,10] - 131.3600800428116 x[14,10]
   - 128.9149567367603 x[15,10] - 137.862453310178 x[16,10]
   - 133.8402841407805 x[17,10] - 77.06669995511432 x[18,10]
   - 103.0032624003772 x[19,10] - 64.25548400553517 x[20,10]
   - 92.45367461499335 x[21,10] - 64.25548400553517 x[22,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 14.83642448030273 x[0,11]
   - 14.83642448030273 x[1,11] - 120.0300037471409 x[2,11]
   - 75.91652799707691 x[3,11] - 123.048847801723 x[4,11]
   - 113.0584734797924 x[5,11] - 137.1214906650389 x[6,11]
   - 113.7156840134443 x[7,11] - 104.4356460041809 x[8,11]
   - 62.1833883053022 x[9,11] - 141.8727341402258 x[10,11]
   - 14.83642448030273 x[11,11] - 120.0300037471409 x[12,11]
   - 75.91652799707691 x[13,11] - 123.048847801723 x[14,11]
   - 113.0584734797924 x[15,11] - 137.1214906650389 x[16,11]
   - 113.7156840134443 x[17,11] - 104.4356460041809 x[18,11]
   - 62.1833883053022 x[19,11] - 141.8727341402258 x[20,11]
   - 14.83642448030273 x[21,11] - 14.83642448030273 x[22,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 53.5712142633938 x[0,12]
   - 53.5712142633938 x[1,12] - 53.5712142633938 x[2,12]
   - 79.10028566988538 x[3,12] - 121.5930986212971 x[4,12]
   - 124.5894800992938 x[5,12] - 135.8793981421117 x[6,12]
   - 123.3244003301235 x[7,12] - 66.45177530232195 x[8,12]
   - 94.96639325761123 x[9,12] - 76.06628385140876 x[10,12]
   - 81.29521396404986 x[11,12] - 53.5712142633938 x[12,12]
   - 79.10028566988538 x[13,12] - 121.5930986212971 x[14,12]
   - 124.5894800992938 x[15,12] - 135.8793981421117 x[16,12]
   - 123.3244003301235 x[17,12] - 66.45177530232195 x[18,12]
   - 94.96639325761123 x[19,12] - 76.06628385140876 x[20,12]
   - 81.29521396404986 x[21,12] - 53.5712142633938 x[22,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 36.08069954320856 x[0,13]
   - 36.08069954320856 x[1,13] - 96.59080039007061 x[2,13]
   - 36.08069954320856 x[3,13] - 87.94202540000012 x[4,13]
   - 134.4274070401528 x[5,13] - 153.340453885165 x[6,13]
   - 84.24894952601183 x[7,13] - 72.78804320489229 x[8,13]
   - 90.37322439894545 x[9,13] - 118.7970482005911 x[10,13]
   - 54.67225293417108 x[11,13] - 96.59080039007061 x[12,13]
   - 36.08069954320856 x[13,13] - 87.94202540000012 x[14,13]
   - 134.4274070401528 x[15,13] - 153.340453885165 x[16,13]
   - 84.24894952601183 x[17,13] - 72.78804320489229 x[18,13]
   - 90.37322439894545 x[19,13] - 118.7970482005911 x[20,13]
   - 54.67225293417108 x[21,13] - 36.08069954320856 x[22,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 61.41627378701835 x[0,14]
   - 61.41627378701835 x[1,14] - 113.7480390976726 x[2,14]
   - 62.60645115619034 x[3,14] - 61.41627378701835 x[4,14]
   - 160.9524624811829 x[5,14] - 179.5953438329374 x[6,14]
   - 68.01021783171375 x[7,14] - 88.28258756182716 x[8,14]
   - 116.3056463735818 x[9,14] - 134.1992902613284 x[10,14]
   - 76.46899849500741 x[11,14] - 113.7480390976726 x[12,14]
   - 62.60645115619034 x[13,14] - 61.41627378701835 x[14,14]
   - 160.9524624811829 x[15,14] - 179.5953438329374 x[16,14]
   - 68.01021783171375 x[17,14] - 88.28258756182716 x[18,14]
   - 116.3056463735818 x[19,14] - 134.1992902613284 x[20,14]
   - 76.46899849500741 x[21,14] - 61.41627378701835 x[22,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 51.6146145691929 x[0,15]
   - 51.6146145691929 x[1,15] - 126.5460797934947 x[2,15]
   - 118.8934920141684 x[3,15] - 170.7541216990084 x[4,15]
   - 51.6146145691929 x[5,15] - 78.2434393370437 x[6,15]
   - 165.645968789987 x[7,15] - 127.5614524804188 x[8,15]
   - 54.9980147727209 x[9,15] - 141.5558261731025 x[10,15]
   - 76.28028339090221 x[11,15] - 126.5460797934947 x[12,15]
   - 118.8934920141684 x[13,15] - 170.7541216990084 x[14,15]
   - 51.6146145691929 x[15,15] - 78.2434393370437 x[16,15]
   - 165.645968789987 x[17,15] - 127.5614524804188 x[18,15]
   - 54.9980147727209 x[19,15] - 141.5558261731025 x[20,15]
   - 76.28028339090221 x[21,15] - 51.6146145691929 x[22,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 62.85156780331815 x[0,16]
   - 62.85156780331815 x[1,16] - 126.5990446021873 x[2,16]
   - 126.5695856250554 x[3,16] - 178.1600498166376 x[4,16]
   - 67.00648610291844 x[5,16] - 62.85156780331815 x[6,16]
   - 174.231169315001 x[7,16] - 130.9626403165377 x[8,16]
   - 69.1855313891661 x[9,16] - 139.266369512395 x[10,16]
   - 89.10634734202344 x[11,16] - 126.5990446021873 x[12,16]
   - 126.5695856250554 x[13,16] - 178.1600498166376 x[14,16]
   - 67.00648610291844 x[15,16] - 62.85156780331815 x[16,16]
   - 174.231169315001 x[17,16] - 130.9626403165377 x[18,16]
   - 69.1855313891661 x[19,16] - 139.266369512395 x[20,16]
   - 89.10634734202344 x[21,16] - 62.85156780331815 x[22,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 57.87380300096044 x[0,17]
   - 57.87380300096044 x[1,17] - 119.0218115925569 x[2,17]
   - 62.45584606825994 x[3,17] - 71.55268861777165 x[4,17]
   - 159.3867803582194 x[5,17] - 179.2089341173587 x[6,17]
   - 57.87380300096044 x[7,17] - 93.52683026189504 x[8,17]
   - 113.0156829602987 x[9,17] - 140.2219651453552 x[10,17]
   - 70.67830549278665 x[11,17] - 119.0218115925569 x[12,17]
   - 62.45584606825994 x[13,17] - 71.55268861777165 x[14,17]
   - 159.3867803582194 x[15,17] - 179.2089341173587 x[16,17]
   - 57.87380300096044 x[17,17] - 93.52683026189504 x[18,17]
   - 113.0156829602987 x[19,17] - 140.2219651453552 x[20,17]
   - 70.67830549278665 x[21,17] - 57.87380300096044 x[22,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 47.23794919579096 x[0,18]
   - 47.23794919579096 x[1,18] - 72.78504036992479 x[2,18]
   - 61.63079355230989 x[3,18] - 102.4609121530546 x[4,18]
   - 131.9381178538207 x[5,18] - 146.5762589240649 x[6,18]
   - 104.1626840670645 x[7,18] - 47.23794919579096 x[8,18]
   - 95.55865185899698 x[9,18] - 94.08423476485852 x[10,18]
   - 72.03412128869272 x[11,18] - 72.78504036992479 x[12,18]
   - 61.63079355230989 x[13,18] - 102.4609121530546 x[14,18]
   - 131.9381178538207 x[15,18] - 146.5762589240649 x[16,18]
   - 104.1626840670645 x[17,18] - 47.23794919579096 x[18,18]
   - 95.55865185899698 x[19,18] - 94.08423476485852 x[20,18]
   - 72.03412128869272 x[21,18] - 47.23794919579096 x[22,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 27.57099861520914 x[0,19]
   - 27.57099861520914 x[1,19] - 120.9666089057959 x[2,19]
   - 98.88292532694487 x[3,19] - 150.150921545391 x[4,19]
   - 79.04163072670467 x[5,19] - 104.4661005772751 x[6,19]
   - 143.31848734605 x[7,19] - 115.2256024395788 x[8,19]
   - 27.57099861520914 x[9,19] - 139.6877477907033 x[10,19]
   - 49.4488141703958 x[11,19] - 120.9666089057959 x[12,19]
   - 98.88292532694487 x[13,19] - 150.150921545391 x[14,19]
   - 79.04163072670467 x[15,19] - 104.4661005772751 x[16,19]
   - 143.31848734605 x[17,19] - 115.2256024395788 x[18,19]
   - 27.57099861520914 x[19,19] - 139.6877477907033 x[20,19]
   - 49.4488141703958 x[21,19] - 27.57099861520914 x[22,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 64.25548400553517 x[0,20]
   - 64.25548400553517 x[1,20] - 65.38201410926739 x[2,20]
   - 90.62226373826451 x[3,20] - 131.3600800428116 x[4,20]
   - 128.9149567367603 x[5,20] - 137.862453310178 x[6,20]
   - 133.8402841407805 x[7,20] - 77.06669995511432 x[8,20]
   - 103.0032624003772 x[9,20] - 64.25548400553517 x[10,20]
   - 92.45367461499335 x[11,20] - 65.38201410926739 x[12,20]
   - 90.62226373826451 x[13,20] - 131.3600800428116 x[14,20]
   - 128.9149567367603 x[15,20] - 137.862453310178 x[16,20]
   - 133.8402841407805 x[17,20] - 77.06669995511432 x[18,20]
   - 103.0032624003772 x[19,20] - 64.25548400553517 x[20,20]
   - 92.45367461499335 x[21,20] - 64.25548400553517 x[22,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 14.83642448030273 x[0,21]
   - 14.83642448030273 x[1,21] - 120.0300037471409 x[2,21]
   - 75.91652799707691 x[3,21] - 123.048847801723 x[4,21]
   - 113.0584734797924 x[5,21] - 137.1214906650389 x[6,21]
   - 113.7156840134443 x[7,21] - 104.4356460041809 x[8,21]
   - 62.1833883053022 x[9,21] - 141.8727341402258 x[10,21]
   - 14.83642448030273 x[11,21] - 120.0300037471409 x[12,21]
   - 75.91652799707691 x[13,21] - 123.048847801723 x[14,21]
   - 113.0584734797924 x[15,21] - 137.1214906650389 x[16,21]
   - 113.7156840134443 x[17,21] - 104.4356460041809 x[18,21]
   - 62.1833883053022 x[19,21] - 141.8727341402258 x[20,21]
   - 14.83642448030273 x[21,21] - 14.83642448030273 x[22,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 107.1424285267876 x[2,22]
   - 72.16139908641712 x[3,22] - 122.8325475740367 x[4,22]
   - 103.2292291383858 x[5,22] - 125.7031356066363 x[6,22]
   - 115.7476060019209 x[7,22] - 94.47589839158192 x[8,22]
   - 55.14199723041828 x[9,22] - 128.5109680110703 x[10,22]
   - 29.67284896060547 x[11,22] - 107.1424285267876 x[12,22]
   - 72.16139908641712 x[13,22] - 122.8325475740367 x[14,22]
   - 103.2292291383858 x[15,22] - 125.7031356066363 x[16,22]
   - 115.7476060019209 x[17,22] - 94.47589839158192 x[18,22]
   - 55.14199723041828 x[19,22] - 128.5109680110703 x[20,22]
   - 29.67284896060547 x[21,22] - start_service[0] + start_service[22]
   >= 0
 LowerBoundArrivalVehicle[23]: - 107.1424285267876 x[2,23]
   - 72.16139908641712 x[3,23] - 122.8325475740367 x[4,23]
   - 103.2292291383858 x[5,23] - 125.7031356066363 x[6,23]
   - 115.7476060019209 x[7,23] - 94.47589839158192 x[8,23]
   - 55.14199723041828 x[9,23] - 128.5109680110703 x[10,23]
   - 29.67284896060547 x[11,23] - 107.1424285267876 x[12,23]
   - 72.16139908641712 x[13,23] - 122.8325475740367 x[14,23]
   - 103.2292291383858 x[15,23] - 125.7031356066363 x[16,23]
   - 115.7476060019209 x[17,23] - 94.47589839158192 x[18,23]
   - 55.14199723041828 x[19,23] - 128.5109680110703 x[20,23]
   - 29.67284896060547 x[21,23] - start_service[0] + start_service[23]
   >= 0
 VisitArrivalNodesNotEmpty[2]: w[0,2] + w[1,2] + w[2,2] + w[3,2] + w[4,2]
   + w[5,2] + w[6,2] + w[7,2] + w[8,2] + w[9,2] + w[10,2] + w[11,2]
   + w[12,2] + w[13,2] + w[14,2] + w[15,2] + w[16,2] + w[17,2] + w[18,2]
   + w[19,2] + w[20,2] + w[21,2] + w[22,2] + w[23,2] >= 1
 VisitArrivalNodesNotEmpty[3]: w[0,3] + w[1,3] + w[2,3] + w[3,3] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] + w[13,3] + w[14,3] + w[15,3] + w[16,3] + w[17,3] + w[18,3]
   + w[19,3] + w[20,3] + w[21,3] + w[22,3] + w[23,3] >= 1
 VisitArrivalNodesNotEmpty[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] + w[4,4]
   + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4]
   + w[12,4] + w[13,4] + w[14,4] + w[15,4] + w[16,4] + w[17,4] + w[18,4]
   + w[19,4] + w[20,4] + w[21,4] + w[22,4] + w[23,4] >= 1
 VisitArrivalNodesNotEmpty[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   + w[5,5] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5]
   + w[12,5] + w[13,5] + w[14,5] + w[15,5] + w[16,5] + w[17,5] + w[18,5]
   + w[19,5] + w[20,5] + w[21,5] + w[22,5] + w[23,5] >= 1
 VisitArrivalNodesNotEmpty[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] + w[6,6] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6]
   + w[12,6] + w[13,6] + w[14,6] + w[15,6] + w[16,6] + w[17,6] + w[18,6]
   + w[19,6] + w[20,6] + w[21,6] + w[22,6] + w[23,6] >= 1
 VisitArrivalNodesNotEmpty[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] + w[7,7] + w[8,7] + w[9,7] + w[10,7] + w[11,7]
   + w[12,7] + w[13,7] + w[14,7] + w[15,7] + w[16,7] + w[17,7] + w[18,7]
   + w[19,7] + w[20,7] + w[21,7] + w[22,7] + w[23,7] >= 1
 VisitArrivalNodesNotEmpty[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] + w[8,8] + w[9,8] + w[10,8] + w[11,8]
   + w[12,8] + w[13,8] + w[14,8] + w[15,8] + w[16,8] + w[17,8] + w[18,8]
   + w[19,8] + w[20,8] + w[21,8] + w[22,8] + w[23,8] >= 1
 VisitArrivalNodesNotEmpty[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] + w[9,9] + w[10,9] + w[11,9]
   + w[12,9] + w[13,9] + w[14,9] + w[15,9] + w[16,9] + w[17,9] + w[18,9]
   + w[19,9] + w[20,9] + w[21,9] + w[22,9] + w[23,9] >= 1
 VisitArrivalNodesNotEmpty[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] + w[10,10]
   + w[11,10] + w[12,10] + w[13,10] + w[14,10] + w[15,10] + w[16,10]
   + w[17,10] + w[18,10] + w[19,10] + w[20,10] + w[21,10] + w[22,10]
   + w[23,10] >= 1
 VisitArrivalNodesNotEmpty[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   + w[11,11] + w[12,11] + w[13,11] + w[14,11] + w[15,11] + w[16,11]
   + w[17,11] + w[18,11] + w[19,11] + w[20,11] + w[21,11] + w[22,11]
   + w[23,11] >= 1
 LeaveArrivalNodesNotFullyLoaded[2]: w[2,0] + w[2,1] + w[2,3] + w[2,4]
   + w[2,5] + w[2,6] + w[2,7] + w[2,8] + w[2,9] + w[2,10] + w[2,11]
   + w[2,13] + w[2,14] + w[2,15] + w[2,16] + w[2,17] + w[2,18] + w[2,19]
   + w[2,20] + w[2,21] + w[2,22] + w[2,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[3]: w[3,0] + w[3,1] + w[3,2] + w[3,4]
   + w[3,5] + w[3,6] + w[3,7] + w[3,8] + w[3,9] + w[3,10] + w[3,11]
   + w[3,12] + w[3,14] + w[3,15] + w[3,16] + w[3,17] + w[3,18] + w[3,19]
   + w[3,20] + w[3,21] + w[3,22] + w[3,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[4]: w[4,0] + w[4,1] + w[4,2] + w[4,3]
   + w[4,5] + w[4,6] + w[4,7] + w[4,8] + w[4,9] + w[4,10] + w[4,11]
   + w[4,12] + w[4,13] + w[4,15] + w[4,16] + w[4,17] + w[4,18] + w[4,19]
   + w[4,20] + w[4,21] + w[4,22] + w[4,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[5]: w[5,0] + w[5,1] + w[5,2] + w[5,3]
   + w[5,4] + w[5,6] + w[5,7] + w[5,8] + w[5,9] + w[5,10] + w[5,11]
   + w[5,12] + w[5,13] + w[5,14] + w[5,16] + w[5,17] + w[5,18] + w[5,19]
   + w[5,20] + w[5,21] + w[5,22] + w[5,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[6]: w[6,0] + w[6,1] + w[6,2] + w[6,3]
   + w[6,4] + w[6,5] + w[6,7] + w[6,8] + w[6,9] + w[6,10] + w[6,11]
   + w[6,12] + w[6,13] + w[6,14] + w[6,15] + w[6,17] + w[6,18] + w[6,19]
   + w[6,20] + w[6,21] + w[6,22] + w[6,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[7]: w[7,0] + w[7,1] + w[7,2] + w[7,3]
   + w[7,4] + w[7,5] + w[7,6] + w[7,8] + w[7,9] + w[7,10] + w[7,11]
   + w[7,12] + w[7,13] + w[7,14] + w[7,15] + w[7,16] + w[7,18] + w[7,19]
   + w[7,20] + w[7,21] + w[7,22] + w[7,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[8]: w[8,0] + w[8,1] + w[8,2] + w[8,3]
   + w[8,4] + w[8,5] + w[8,6] + w[8,7] + w[8,9] + w[8,10] + w[8,11]
   + w[8,12] + w[8,13] + w[8,14] + w[8,15] + w[8,16] + w[8,17] + w[8,19]
   + w[8,20] + w[8,21] + w[8,22] + w[8,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[9]: w[9,0] + w[9,1] + w[9,2] + w[9,3]
   + w[9,4] + w[9,5] + w[9,6] + w[9,7] + w[9,8] + w[9,10] + w[9,11]
   + w[9,12] + w[9,13] + w[9,14] + w[9,15] + w[9,16] + w[9,17] + w[9,18]
   + w[9,20] + w[9,21] + w[9,22] + w[9,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[10]: w[10,0] + w[10,1] + w[10,2] + w[10,3]
   + w[10,4] + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,11]
   + w[10,12] + w[10,13] + w[10,14] + w[10,15] + w[10,16] + w[10,17]
   + w[10,18] + w[10,19] + w[10,21] + w[10,22] + w[10,23] <= 1
 LeaveArrivalNodesNotFullyLoaded[11]: w[11,0] + w[11,1] + w[11,2] + w[11,3]
   + w[11,4] + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10]
   + w[11,12] + w[11,13] + w[11,14] + w[11,15] + w[11,16] + w[11,17]
   + w[11,18] + w[11,19] + w[11,20] + w[11,22] + w[11,23] <= 1
 VisitDepartureNodesNotFullyLoaded[12]: w[0,12] + w[1,12] + w[3,12]
   + w[4,12] + w[5,12] + w[6,12] + w[7,12] + w[8,12] + w[9,12] + w[10,12]
   + w[11,12] + w[13,12] + w[14,12] + w[15,12] + w[16,12] + w[17,12]
   + w[18,12] + w[19,12] + w[20,12] + w[21,12] + w[22,12] + w[23,12] <= 1
 VisitDepartureNodesNotFullyLoaded[13]: w[0,13] + w[1,13] + w[2,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] + w[14,13] + w[15,13] + w[16,13] + w[17,13]
   + w[18,13] + w[19,13] + w[20,13] + w[21,13] + w[22,13] + w[23,13] <= 1
 VisitDepartureNodesNotFullyLoaded[14]: w[0,14] + w[1,14] + w[2,14]
   + w[3,14] + w[5,14] + w[6,14] + w[7,14] + w[8,14] + w[9,14] + w[10,14]
   + w[11,14] + w[12,14] + w[13,14] + w[15,14] + w[16,14] + w[17,14]
   + w[18,14] + w[19,14] + w[20,14] + w[21,14] + w[22,14] + w[23,14] <= 1
 VisitDepartureNodesNotFullyLoaded[15]: w[0,15] + w[1,15] + w[2,15]
   + w[3,15] + w[4,15] + w[6,15] + w[7,15] + w[8,15] + w[9,15] + w[10,15]
   + w[11,15] + w[12,15] + w[13,15] + w[14,15] + w[16,15] + w[17,15]
   + w[18,15] + w[19,15] + w[20,15] + w[21,15] + w[22,15] + w[23,15] <= 1
 VisitDepartureNodesNotFullyLoaded[16]: w[0,16] + w[1,16] + w[2,16]
   + w[3,16] + w[4,16] + w[5,16] + w[7,16] + w[8,16] + w[9,16] + w[10,16]
   + w[11,16] + w[12,16] + w[13,16] + w[14,16] + w[15,16] + w[17,16]
   + w[18,16] + w[19,16] + w[20,16] + w[21,16] + w[22,16] + w[23,16] <= 1
 VisitDepartureNodesNotFullyLoaded[17]: w[0,17] + w[1,17] + w[2,17]
   + w[3,17] + w[4,17] + w[5,17] + w[6,17] + w[8,17] + w[9,17] + w[10,17]
   + w[11,17] + w[12,17] + w[13,17] + w[14,17] + w[15,17] + w[16,17]
   + w[18,17] + w[19,17] + w[20,17] + w[21,17] + w[22,17] + w[23,17] <= 1
 VisitDepartureNodesNotFullyLoaded[18]: w[0,18] + w[1,18] + w[2,18]
   + w[3,18] + w[4,18] + w[5,18] + w[6,18] + w[7,18] + w[9,18] + w[10,18]
   + w[11,18] + w[12,18] + w[13,18] + w[14,18] + w[15,18] + w[16,18]
   + w[17,18] + w[19,18] + w[20,18] + w[21,18] + w[22,18] + w[23,18] <= 1
 VisitDepartureNodesNotFullyLoaded[19]: w[0,19] + w[1,19] + w[2,19]
   + w[3,19] + w[4,19] + w[5,19] + w[6,19] + w[7,19] + w[8,19] + w[10,19]
   + w[11,19] + w[12,19] + w[13,19] + w[14,19] + w[15,19] + w[16,19]
   + w[17,19] + w[18,19] + w[20,19] + w[21,19] + w[22,19] + w[23,19] <= 1
 VisitDepartureNodesNotFullyLoaded[20]: w[0,20] + w[1,20] + w[2,20]
   + w[3,20] + w[4,20] + w[5,20] + w[6,20] + w[7,20] + w[8,20] + w[9,20]
   + w[11,20] + w[12,20] + w[13,20] + w[14,20] + w[15,20] + w[16,20]
   + w[17,20] + w[18,20] + w[19,20] + w[21,20] + w[22,20] + w[23,20] <= 1
 VisitDepartureNodesNotFullyLoaded[21]: w[0,21] + w[1,21] + w[2,21]
   + w[3,21] + w[4,21] + w[5,21] + w[6,21] + w[7,21] + w[8,21] + w[9,21]
   + w[10,21] + w[12,21] + w[13,21] + w[14,21] + w[15,21] + w[16,21]
   + w[17,21] + w[18,21] + w[19,21] + w[20,21] + w[22,21] + w[23,21] <= 1
 LeaveDepartureNodesNotEmpty[12]: w[12,0] + w[12,1] + w[12,2] + w[12,3]
   + w[12,4] + w[12,5] + w[12,6] + w[12,7] + w[12,8] + w[12,9] + w[12,10]
   + w[12,11] + w[12,12] + w[12,13] + w[12,14] + w[12,15] + w[12,16]
   + w[12,17] + w[12,18] + w[12,19] + w[12,20] + w[12,21] + w[12,22]
   + w[12,23] >= 1
 LeaveDepartureNodesNotEmpty[13]: w[13,0] + w[13,1] + w[13,2] + w[13,3]
   + w[13,4] + w[13,5] + w[13,6] + w[13,7] + w[13,8] + w[13,9] + w[13,10]
   + w[13,11] + w[13,12] + w[13,13] + w[13,14] + w[13,15] + w[13,16]
   + w[13,17] + w[13,18] + w[13,19] + w[13,20] + w[13,21] + w[13,22]
   + w[13,23] >= 1
 LeaveDepartureNodesNotEmpty[14]: w[14,0] + w[14,1] + w[14,2] + w[14,3]
   + w[14,4] + w[14,5] + w[14,6] + w[14,7] + w[14,8] + w[14,9] + w[14,10]
   + w[14,11] + w[14,12] + w[14,13] + w[14,14] + w[14,15] + w[14,16]
   + w[14,17] + w[14,18] + w[14,19] + w[14,20] + w[14,21] + w[14,22]
   + w[14,23] >= 1
 LeaveDepartureNodesNotEmpty[15]: w[15,0] + w[15,1] + w[15,2] + w[15,3]
   + w[15,4] + w[15,5] + w[15,6] + w[15,7] + w[15,8] + w[15,9] + w[15,10]
   + w[15,11] + w[15,12] + w[15,13] + w[15,14] + w[15,15] + w[15,16]
   + w[15,17] + w[15,18] + w[15,19] + w[15,20] + w[15,21] + w[15,22]
   + w[15,23] >= 1
 LeaveDepartureNodesNotEmpty[16]: w[16,0] + w[16,1] + w[16,2] + w[16,3]
   + w[16,4] + w[16,5] + w[16,6] + w[16,7] + w[16,8] + w[16,9] + w[16,10]
   + w[16,11] + w[16,12] + w[16,13] + w[16,14] + w[16,15] + w[16,16]
   + w[16,17] + w[16,18] + w[16,19] + w[16,20] + w[16,21] + w[16,22]
   + w[16,23] >= 1
 LeaveDepartureNodesNotEmpty[17]: w[17,0] + w[17,1] + w[17,2] + w[17,3]
   + w[17,4] + w[17,5] + w[17,6] + w[17,7] + w[17,8] + w[17,9] + w[17,10]
   + w[17,11] + w[17,12] + w[17,13] + w[17,14] + w[17,15] + w[17,16]
   + w[17,17] + w[17,18] + w[17,19] + w[17,20] + w[17,21] + w[17,22]
   + w[17,23] >= 1
 LeaveDepartureNodesNotEmpty[18]: w[18,0] + w[18,1] + w[18,2] + w[18,3]
   + w[18,4] + w[18,5] + w[18,6] + w[18,7] + w[18,8] + w[18,9] + w[18,10]
   + w[18,11] + w[18,12] + w[18,13] + w[18,14] + w[18,15] + w[18,16]
   + w[18,17] + w[18,18] + w[18,19] + w[18,20] + w[18,21] + w[18,22]
   + w[18,23] >= 1
 LeaveDepartureNodesNotEmpty[19]: w[19,0] + w[19,1] + w[19,2] + w[19,3]
   + w[19,4] + w[19,5] + w[19,6] + w[19,7] + w[19,8] + w[19,9] + w[19,10]
   + w[19,11] + w[19,12] + w[19,13] + w[19,14] + w[19,15] + w[19,16]
   + w[19,17] + w[19,18] + w[19,19] + w[19,20] + w[19,21] + w[19,22]
   + w[19,23] >= 1
 LeaveDepartureNodesNotEmpty[20]: w[20,0] + w[20,1] + w[20,2] + w[20,3]
   + w[20,4] + w[20,5] + w[20,6] + w[20,7] + w[20,8] + w[20,9] + w[20,10]
   + w[20,11] + w[20,12] + w[20,13] + w[20,14] + w[20,15] + w[20,16]
   + w[20,17] + w[20,18] + w[20,19] + w[20,20] + w[20,21] + w[20,22]
   + w[20,23] >= 1
 LeaveDepartureNodesNotEmpty[21]: w[21,0] + w[21,1] + w[21,2] + w[21,3]
   + w[21,4] + w[21,5] + w[21,6] + w[21,7] + w[21,8] + w[21,9] + w[21,10]
   + w[21,11] + w[21,12] + w[21,13] + w[21,14] + w[21,15] + w[21,16]
   + w[21,17] + w[21,18] + w[21,19] + w[21,20] + w[21,21] + w[21,22]
   + w[21,23] >= 1
Bounds
 x[0,0] = 0
 x[1,0] = 0
 x[1,1] = 0
 x[2,0] = 0
 x[2,2] = 0
 x[3,0] = 0
 x[3,3] = 0
 x[4,0] = 0
 x[4,4] = 0
 x[5,0] = 0
 x[5,5] = 0
 x[6,0] = 0
 x[6,6] = 0
 x[7,0] = 0
 x[7,7] = 0
 x[8,0] = 0
 x[8,8] = 0
 x[9,0] = 0
 x[9,9] = 0
 x[10,0] = 0
 x[10,10] = 0
 x[11,0] = 0
 x[11,11] = 0
 x[12,0] = 0
 x[12,2] = 0
 x[12,12] = 0
 x[13,0] = 0
 x[13,3] = 0
 x[13,13] = 0
 x[14,0] = 0
 x[14,4] = 0
 x[14,14] = 0
 x[15,0] = 0
 x[15,5] = 0
 x[15,15] = 0
 x[16,0] = 0
 x[16,6] = 0
 x[16,16] = 0
 x[17,0] = 0
 x[17,7] = 0
 x[17,17] = 0
 x[18,0] = 0
 x[18,8] = 0
 x[18,18] = 0
 x[19,0] = 0
 x[19,9] = 0
 x[19,19] = 0
 x[20,0] = 0
 x[20,10] = 0
 x[20,20] = 0
 x[21,0] = 0
 x[21,11] = 0
 x[21,21] = 0
 x[22,0] = 0
 x[22,22] = 0
 x[23,0] = 0
 x[23,1] = 0
 x[23,2] = 0
 x[23,3] = 0
 x[23,4] = 0
 x[23,5] = 0
 x[23,6] = 0
 x[23,7] = 0
 x[23,8] = 0
 x[23,9] = 0
 x[23,10] = 0
 x[23,11] = 0
 x[23,12] = 0
 x[23,13] = 0
 x[23,14] = 0
 x[23,15] = 0
 x[23,16] = 0
 x[23,17] = 0
 x[23,18] = 0
 x[23,19] = 0
 x[23,20] = 0
 x[23,21] = 0
 x[23,22] = 0
 x[23,23] = 0
 w[0,0] = -0
 w[0,1] = -0
 w[0,2] <= 2
 w[0,3] <= 2
 w[0,4] <= 2
 w[0,5] <= 2
 w[0,6] <= 2
 w[0,7] <= 2
 w[0,8] <= 2
 w[0,9] <= 2
 w[0,10] <= 2
 w[0,11] <= 2
 w[0,12] <= 2
 w[0,13] <= 2
 w[0,14] <= 2
 w[0,15] <= 2
 w[0,16] <= 2
 w[0,17] <= 2
 w[0,18] <= 2
 w[0,19] <= 2
 w[0,20] <= 2
 w[0,21] <= 2
 w[0,22] <= 2
 w[0,23] <= 2
 w[1,0] = -0
 w[1,1] = -0
 w[1,2] <= 2
 w[1,3] <= 2
 w[1,4] <= 2
 w[1,5] <= 2
 w[1,6] <= 2
 w[1,7] <= 2
 w[1,8] <= 2
 w[1,9] <= 2
 w[1,10] <= 2
 w[1,11] <= 2
 w[1,12] <= 2
 w[1,13] <= 2
 w[1,14] <= 2
 w[1,15] <= 2
 w[1,16] <= 2
 w[1,17] <= 2
 w[1,18] <= 2
 w[1,19] <= 2
 w[1,20] <= 2
 w[1,21] <= 2
 w[1,22] <= 2
 w[1,23] <= 2
 w[2,0] = -0
 w[2,1] = -0
 w[2,2] = -0
 w[2,3] <= 2
 w[2,4] <= 2
 w[2,5] <= 2
 w[2,6] <= 2
 w[2,7] <= 2
 w[2,8] <= 2
 w[2,9] <= 2
 w[2,10] <= 2
 w[2,11] <= 2
 w[2,12] <= 2
 w[2,13] <= 2
 w[2,14] <= 2
 w[2,15] <= 2
 w[2,16] <= 2
 w[2,17] <= 2
 w[2,18] <= 2
 w[2,19] <= 2
 w[2,20] <= 2
 w[2,21] <= 2
 w[2,22] <= 2
 w[2,23] <= 2
 w[3,0] = -0
 w[3,1] = -0
 w[3,2] <= 2
 w[3,3] = -0
 w[3,4] <= 2
 w[3,5] <= 2
 w[3,6] <= 2
 w[3,7] <= 2
 w[3,8] <= 2
 w[3,9] <= 2
 w[3,10] <= 2
 w[3,11] <= 2
 w[3,12] <= 2
 w[3,13] <= 2
 w[3,14] <= 2
 w[3,15] <= 2
 w[3,16] <= 2
 w[3,17] <= 2
 w[3,18] <= 2
 w[3,19] <= 2
 w[3,20] <= 2
 w[3,21] <= 2
 w[3,22] <= 2
 w[3,23] <= 2
 w[4,0] = -0
 w[4,1] = -0
 w[4,2] <= 2
 w[4,3] <= 2
 w[4,4] = -0
 w[4,5] <= 2
 w[4,6] <= 2
 w[4,7] <= 2
 w[4,8] <= 2
 w[4,9] <= 2
 w[4,10] <= 2
 w[4,11] <= 2
 w[4,12] <= 2
 w[4,13] <= 2
 w[4,14] <= 2
 w[4,15] <= 2
 w[4,16] <= 2
 w[4,17] <= 2
 w[4,18] <= 2
 w[4,19] <= 2
 w[4,20] <= 2
 w[4,21] <= 2
 w[4,22] <= 2
 w[4,23] <= 2
 w[5,0] = -0
 w[5,1] = -0
 w[5,2] <= 2
 w[5,3] <= 2
 w[5,4] <= 2
 w[5,5] = -0
 w[5,6] <= 2
 w[5,7] <= 2
 w[5,8] <= 2
 w[5,9] <= 2
 w[5,10] <= 2
 w[5,11] <= 2
 w[5,12] <= 2
 w[5,13] <= 2
 w[5,14] <= 2
 w[5,15] <= 2
 w[5,16] <= 2
 w[5,17] <= 2
 w[5,18] <= 2
 w[5,19] <= 2
 w[5,20] <= 2
 w[5,21] <= 2
 w[5,22] <= 2
 w[5,23] <= 2
 w[6,0] = -0
 w[6,1] = -0
 w[6,2] <= 2
 w[6,3] <= 2
 w[6,4] <= 2
 w[6,5] <= 2
 w[6,6] = -0
 w[6,7] <= 2
 w[6,8] <= 2
 w[6,9] <= 2
 w[6,10] <= 2
 w[6,11] <= 2
 w[6,12] <= 2
 w[6,13] <= 2
 w[6,14] <= 2
 w[6,15] <= 2
 w[6,16] <= 2
 w[6,17] <= 2
 w[6,18] <= 2
 w[6,19] <= 2
 w[6,20] <= 2
 w[6,21] <= 2
 w[6,22] <= 2
 w[6,23] <= 2
 w[7,0] = -0
 w[7,1] = -0
 w[7,2] <= 2
 w[7,3] <= 2
 w[7,4] <= 2
 w[7,5] <= 2
 w[7,6] <= 2
 w[7,7] = -0
 w[7,8] <= 2
 w[7,9] <= 2
 w[7,10] <= 2
 w[7,11] <= 2
 w[7,12] <= 2
 w[7,13] <= 2
 w[7,14] <= 2
 w[7,15] <= 2
 w[7,16] <= 2
 w[7,17] <= 2
 w[7,18] <= 2
 w[7,19] <= 2
 w[7,20] <= 2
 w[7,21] <= 2
 w[7,22] <= 2
 w[7,23] <= 2
 w[8,0] = -0
 w[8,1] = -0
 w[8,2] <= 2
 w[8,3] <= 2
 w[8,4] <= 2
 w[8,5] <= 2
 w[8,6] <= 2
 w[8,7] <= 2
 w[8,8] = -0
 w[8,9] <= 2
 w[8,10] <= 2
 w[8,11] <= 2
 w[8,12] <= 2
 w[8,13] <= 2
 w[8,14] <= 2
 w[8,15] <= 2
 w[8,16] <= 2
 w[8,17] <= 2
 w[8,18] <= 2
 w[8,19] <= 2
 w[8,20] <= 2
 w[8,21] <= 2
 w[8,22] <= 2
 w[8,23] <= 2
 w[9,0] = -0
 w[9,1] = -0
 w[9,2] <= 2
 w[9,3] <= 2
 w[9,4] <= 2
 w[9,5] <= 2
 w[9,6] <= 2
 w[9,7] <= 2
 w[9,8] <= 2
 w[9,9] = -0
 w[9,10] <= 2
 w[9,11] <= 2
 w[9,12] <= 2
 w[9,13] <= 2
 w[9,14] <= 2
 w[9,15] <= 2
 w[9,16] <= 2
 w[9,17] <= 2
 w[9,18] <= 2
 w[9,19] <= 2
 w[9,20] <= 2
 w[9,21] <= 2
 w[9,22] <= 2
 w[9,23] <= 2
 w[10,0] = -0
 w[10,1] = -0
 w[10,2] <= 2
 w[10,3] <= 2
 w[10,4] <= 2
 w[10,5] <= 2
 w[10,6] <= 2
 w[10,7] <= 2
 w[10,8] <= 2
 w[10,9] <= 2
 w[10,10] = -0
 w[10,11] <= 2
 w[10,12] <= 2
 w[10,13] <= 2
 w[10,14] <= 2
 w[10,15] <= 2
 w[10,16] <= 2
 w[10,17] <= 2
 w[10,18] <= 2
 w[10,19] <= 2
 w[10,20] <= 2
 w[10,21] <= 2
 w[10,22] <= 2
 w[10,23] <= 2
 w[11,0] = -0
 w[11,1] = -0
 w[11,2] <= 2
 w[11,3] <= 2
 w[11,4] <= 2
 w[11,5] <= 2
 w[11,6] <= 2
 w[11,7] <= 2
 w[11,8] <= 2
 w[11,9] <= 2
 w[11,10] <= 2
 w[11,11] = -0
 w[11,12] <= 2
 w[11,13] <= 2
 w[11,14] <= 2
 w[11,15] <= 2
 w[11,16] <= 2
 w[11,17] <= 2
 w[11,18] <= 2
 w[11,19] <= 2
 w[11,20] <= 2
 w[11,21] <= 2
 w[11,22] <= 2
 w[11,23] <= 2
 w[12,0] = -0
 w[12,1] = -0
 w[12,2] = -0
 w[12,3] <= 2
 w[12,4] <= 2
 w[12,5] <= 2
 w[12,6] <= 2
 w[12,7] <= 2
 w[12,8] <= 2
 w[12,9] <= 2
 w[12,10] <= 2
 w[12,11] <= 2
 w[12,12] = -0
 w[12,13] <= 2
 w[12,14] <= 2
 w[12,15] <= 2
 w[12,16] <= 2
 w[12,17] <= 2
 w[12,18] <= 2
 w[12,19] <= 2
 w[12,20] <= 2
 w[12,21] <= 2
 w[12,22] <= 2
 w[12,23] <= 2
 w[13,0] = -0
 w[13,1] = -0
 w[13,2] <= 2
 w[13,3] = -0
 w[13,4] <= 2
 w[13,5] <= 2
 w[13,6] <= 2
 w[13,7] <= 2
 w[13,8] <= 2
 w[13,9] <= 2
 w[13,10] <= 2
 w[13,11] <= 2
 w[13,12] <= 2
 w[13,13] = -0
 w[13,14] <= 2
 w[13,15] <= 2
 w[13,16] <= 2
 w[13,17] <= 2
 w[13,18] <= 2
 w[13,19] <= 2
 w[13,20] <= 2
 w[13,21] <= 2
 w[13,22] <= 2
 w[13,23] <= 2
 w[14,0] = -0
 w[14,1] = -0
 w[14,2] <= 2
 w[14,3] <= 2
 w[14,4] = -0
 w[14,5] <= 2
 w[14,6] <= 2
 w[14,7] <= 2
 w[14,8] <= 2
 w[14,9] <= 2
 w[14,10] <= 2
 w[14,11] <= 2
 w[14,12] <= 2
 w[14,13] <= 2
 w[14,14] = -0
 w[14,15] <= 2
 w[14,16] <= 2
 w[14,17] <= 2
 w[14,18] <= 2
 w[14,19] <= 2
 w[14,20] <= 2
 w[14,21] <= 2
 w[14,22] <= 2
 w[14,23] <= 2
 w[15,0] = -0
 w[15,1] = -0
 w[15,2] <= 2
 w[15,3] <= 2
 w[15,4] <= 2
 w[15,5] = -0
 w[15,6] <= 2
 w[15,7] <= 2
 w[15,8] <= 2
 w[15,9] <= 2
 w[15,10] <= 2
 w[15,11] <= 2
 w[15,12] <= 2
 w[15,13] <= 2
 w[15,14] <= 2
 w[15,15] = -0
 w[15,16] <= 2
 w[15,17] <= 2
 w[15,18] <= 2
 w[15,19] <= 2
 w[15,20] <= 2
 w[15,21] <= 2
 w[15,22] <= 2
 w[15,23] <= 2
 w[16,0] = -0
 w[16,1] = -0
 w[16,2] <= 2
 w[16,3] <= 2
 w[16,4] <= 2
 w[16,5] <= 2
 w[16,6] = -0
 w[16,7] <= 2
 w[16,8] <= 2
 w[16,9] <= 2
 w[16,10] <= 2
 w[16,11] <= 2
 w[16,12] <= 2
 w[16,13] <= 2
 w[16,14] <= 2
 w[16,15] <= 2
 w[16,16] = -0
 w[16,17] <= 2
 w[16,18] <= 2
 w[16,19] <= 2
 w[16,20] <= 2
 w[16,21] <= 2
 w[16,22] <= 2
 w[16,23] <= 2
 w[17,0] = -0
 w[17,1] = -0
 w[17,2] <= 2
 w[17,3] <= 2
 w[17,4] <= 2
 w[17,5] <= 2
 w[17,6] <= 2
 w[17,7] = -0
 w[17,8] <= 2
 w[17,9] <= 2
 w[17,10] <= 2
 w[17,11] <= 2
 w[17,12] <= 2
 w[17,13] <= 2
 w[17,14] <= 2
 w[17,15] <= 2
 w[17,16] <= 2
 w[17,17] = -0
 w[17,18] <= 2
 w[17,19] <= 2
 w[17,20] <= 2
 w[17,21] <= 2
 w[17,22] <= 2
 w[17,23] <= 2
 w[18,0] = -0
 w[18,1] = -0
 w[18,2] <= 2
 w[18,3] <= 2
 w[18,4] <= 2
 w[18,5] <= 2
 w[18,6] <= 2
 w[18,7] <= 2
 w[18,8] = -0
 w[18,9] <= 2
 w[18,10] <= 2
 w[18,11] <= 2
 w[18,12] <= 2
 w[18,13] <= 2
 w[18,14] <= 2
 w[18,15] <= 2
 w[18,16] <= 2
 w[18,17] <= 2
 w[18,18] = -0
 w[18,19] <= 2
 w[18,20] <= 2
 w[18,21] <= 2
 w[18,22] <= 2
 w[18,23] <= 2
 w[19,0] = -0
 w[19,1] = -0
 w[19,2] <= 2
 w[19,3] <= 2
 w[19,4] <= 2
 w[19,5] <= 2
 w[19,6] <= 2
 w[19,7] <= 2
 w[19,8] <= 2
 w[19,9] = -0
 w[19,10] <= 2
 w[19,11] <= 2
 w[19,12] <= 2
 w[19,13] <= 2
 w[19,14] <= 2
 w[19,15] <= 2
 w[19,16] <= 2
 w[19,17] <= 2
 w[19,18] <= 2
 w[19,19] = -0
 w[19,20] <= 2
 w[19,21] <= 2
 w[19,22] <= 2
 w[19,23] <= 2
 w[20,0] = -0
 w[20,1] = -0
 w[20,2] <= 2
 w[20,3] <= 2
 w[20,4] <= 2
 w[20,5] <= 2
 w[20,6] <= 2
 w[20,7] <= 2
 w[20,8] <= 2
 w[20,9] <= 2
 w[20,10] = -0
 w[20,11] <= 2
 w[20,12] <= 2
 w[20,13] <= 2
 w[20,14] <= 2
 w[20,15] <= 2
 w[20,16] <= 2
 w[20,17] <= 2
 w[20,18] <= 2
 w[20,19] <= 2
 w[20,20] = -0
 w[20,21] <= 2
 w[20,22] <= 2
 w[20,23] <= 2
 w[21,0] = -0
 w[21,1] = -0
 w[21,2] <= 2
 w[21,3] <= 2
 w[21,4] <= 2
 w[21,5] <= 2
 w[21,6] <= 2
 w[21,7] <= 2
 w[21,8] <= 2
 w[21,9] <= 2
 w[21,10] <= 2
 w[21,11] = -0
 w[21,12] <= 2
 w[21,13] <= 2
 w[21,14] <= 2
 w[21,15] <= 2
 w[21,16] <= 2
 w[21,17] <= 2
 w[21,18] <= 2
 w[21,19] <= 2
 w[21,20] <= 2
 w[21,21] = -0
 w[21,22] <= 2
 w[21,23] <= 2
 w[22,0] = -0
 w[22,1] = -0
 w[22,2] = -0
 w[22,3] = -0
 w[22,4] = -0
 w[22,5] = -0
 w[22,6] = -0
 w[22,7] = -0
 w[22,8] = -0
 w[22,9] = -0
 w[22,10] = -0
 w[22,11] = -0
 w[22,12] = -0
 w[22,13] = -0
 w[22,14] = -0
 w[22,15] = -0
 w[22,16] = -0
 w[22,17] = -0
 w[22,18] = -0
 w[22,19] = -0
 w[22,20] = -0
 w[22,21] = -0
 w[22,22] = -0
 w[22,23] = -0
 w[23,0] = -0
 w[23,1] = -0
 w[23,2] = -0
 w[23,3] = -0
 w[23,4] = -0
 w[23,5] = -0
 w[23,6] = -0
 w[23,7] = -0
 w[23,8] = -0
 w[23,9] = -0
 w[23,10] = -0
 w[23,11] = -0
 w[23,12] = -0
 w[23,13] = -0
 w[23,14] = -0
 w[23,15] = -0
 w[23,16] = -0
 w[23,17] = -0
 w[23,18] = -0
 w[23,19] = -0
 w[23,20] = -0
 w[23,21] = -0
 w[23,22] = -0
 w[23,23] = -0
 v[0,0] = 0
 v[0,1] = 0
 v[1,0] = 0
 v[1,1] = 0
 v[2,0] = 0
 v[2,1] = 0
 v[2,2] = 0
 v[2,12] = 1
 v[3,0] = 0
 v[3,1] = 0
 v[3,3] = 0
 v[3,13] = 1
 v[4,0] = 0
 v[4,1] = 0
 v[4,4] = 0
 v[4,14] = 1
 v[5,0] = 0
 v[5,1] = 0
 v[5,5] = 0
 v[5,15] = 1
 v[6,0] = 0
 v[6,1] = 0
 v[6,6] = 0
 v[6,16] = 1
 v[7,0] = 0
 v[7,1] = 0
 v[7,7] = 0
 v[7,17] = 1
 v[8,0] = 0
 v[8,1] = 0
 v[8,8] = 0
 v[8,18] = 1
 v[9,0] = 0
 v[9,1] = 0
 v[9,9] = 0
 v[9,19] = 1
 v[10,0] = 0
 v[10,1] = 0
 v[10,10] = 0
 v[10,20] = 1
 v[11,0] = 0
 v[11,1] = 0
 v[11,11] = 0
 v[11,21] = 1
 v[12,0] = 0
 v[12,1] = 0
 v[12,2] = 0
 v[12,12] = 0
 v[13,0] = 0
 v[13,1] = 0
 v[13,3] = 0
 v[13,13] = 0
 v[14,0] = 0
 v[14,1] = 0
 v[14,4] = 0
 v[14,14] = 0
 v[15,0] = 0
 v[15,1] = 0
 v[15,5] = 0
 v[15,15] = 0
 v[16,0] = 0
 v[16,1] = 0
 v[16,6] = 0
 v[16,16] = 0
 v[17,0] = 0
 v[17,1] = 0
 v[17,7] = 0
 v[17,17] = 0
 v[18,0] = 0
 v[18,1] = 0
 v[18,8] = 0
 v[18,18] = 0
 v[19,0] = 0
 v[19,1] = 0
 v[19,9] = 0
 v[19,19] = 0
 v[20,0] = 0
 v[20,1] = 0
 v[20,10] = 0
 v[20,20] = 0
 v[21,0] = 0
 v[21,1] = 0
 v[21,11] = 0
 v[21,21] = 0
 v[22,0] = 0
 v[22,1] = 0
 v[22,2] = 0
 v[22,3] = 0
 v[22,4] = 0
 v[22,5] = 0
 v[22,6] = 0
 v[22,7] = 0
 v[22,8] = 0
 v[22,9] = 0
 v[22,10] = 0
 v[22,11] = 0
 v[22,12] = 0
 v[22,13] = 0
 v[22,14] = 0
 v[22,15] = 0
 v[22,16] = 0
 v[22,17] = 0
 v[22,18] = 0
 v[22,19] = 0
 v[22,20] = 0
 v[22,21] = 0
 v[22,22] = 0
 v[22,23] = 0
 v[23,0] = 0
 v[23,1] = 0
 v[23,2] = 0
 v[23,3] = 0
 v[23,4] = 0
 v[23,5] = 0
 v[23,6] = 0
 v[23,7] = 0
 v[23,8] = 0
 v[23,9] = 0
 v[23,10] = 0
 v[23,11] = 0
 v[23,12] = 0
 v[23,13] = 0
 v[23,14] = 0
 v[23,15] = 0
 v[23,16] = 0
 v[23,17] = 0
 v[23,18] = 0
 v[23,19] = 0
 v[23,20] = 0
 v[23,21] = 0
 v[23,22] = 0
 v[23,23] = 0
 service_time[0] = 0
 service_time[1] = 0
 service_time[12] = 0
 service_time[13] = 0
 service_time[14] = 0
 service_time[15] = 0
 service_time[16] = 0
 service_time[17] = 0
 service_time[18] = 0
 service_time[19] = 0
 service_time[20] = 0
 service_time[21] = 0
 service_time[22] = 0
 service_time[23] = 0
 start_service[0] = 0
 start_service[1] <= 10000
 start_service[2] <= 10000
 start_service[3] <= 10000
 start_service[4] <= 10000
 start_service[5] <= 10000
 start_service[6] <= 10000
 start_service[7] <= 10000
 start_service[8] <= 10000
 start_service[9] <= 10000
 start_service[10] <= 10000
 start_service[11] <= 10000
 start_service[12] <= 10000
 start_service[13] <= 10000
 start_service[14] <= 10000
 start_service[15] <= 10000
 start_service[16] <= 10000
 start_service[17] <= 10000
 start_service[18] <= 10000
 start_service[19] <= 10000
 start_service[20] <= 10000
 start_service[21] <= 10000
 start_service[22] <= 10000
 start_service[23] <= 10000
Binaries
 x[0,0] x[0,1] x[0,2] x[0,3] x[0,4] x[0,5] x[0,6] x[0,7] x[0,8] x[0,9]
 x[0,10] x[0,11] x[0,12] x[0,13] x[0,14] x[0,15] x[0,16] x[0,17] x[0,18]
 x[0,19] x[0,20] x[0,21] x[0,22] x[0,23] x[1,0] x[1,1] x[1,2] x[1,3] x[1,4]
 x[1,5] x[1,6] x[1,7] x[1,8] x[1,9] x[1,10] x[1,11] x[1,12] x[1,13] x[1,14]
 x[1,15] x[1,16] x[1,17] x[1,18] x[1,19] x[1,20] x[1,21] x[1,22] x[1,23]
 x[2,0] x[2,1] x[2,2] x[2,3] x[2,4] x[2,5] x[2,6] x[2,7] x[2,8] x[2,9]
 x[2,10] x[2,11] x[2,12] x[2,13] x[2,14] x[2,15] x[2,16] x[2,17] x[2,18]
 x[2,19] x[2,20] x[2,21] x[2,22] x[2,23] x[3,0] x[3,1] x[3,2] x[3,3] x[3,4]
 x[3,5] x[3,6] x[3,7] x[3,8] x[3,9] x[3,10] x[3,11] x[3,12] x[3,13] x[3,14]
 x[3,15] x[3,16] x[3,17] x[3,18] x[3,19] x[3,20] x[3,21] x[3,22] x[3,23]
 x[4,0] x[4,1] x[4,2] x[4,3] x[4,4] x[4,5] x[4,6] x[4,7] x[4,8] x[4,9]
 x[4,10] x[4,11] x[4,12] x[4,13] x[4,14] x[4,15] x[4,16] x[4,17] x[4,18]
 x[4,19] x[4,20] x[4,21] x[4,22] x[4,23] x[5,0] x[5,1] x[5,2] x[5,3] x[5,4]
 x[5,5] x[5,6] x[5,7] x[5,8] x[5,9] x[5,10] x[5,11] x[5,12] x[5,13] x[5,14]
 x[5,15] x[5,16] x[5,17] x[5,18] x[5,19] x[5,20] x[5,21] x[5,22] x[5,23]
 x[6,0] x[6,1] x[6,2] x[6,3] x[6,4] x[6,5] x[6,6] x[6,7] x[6,8] x[6,9]
 x[6,10] x[6,11] x[6,12] x[6,13] x[6,14] x[6,15] x[6,16] x[6,17] x[6,18]
 x[6,19] x[6,20] x[6,21] x[6,22] x[6,23] x[7,0] x[7,1] x[7,2] x[7,3] x[7,4]
 x[7,5] x[7,6] x[7,7] x[7,8] x[7,9] x[7,10] x[7,11] x[7,12] x[7,13] x[7,14]
 x[7,15] x[7,16] x[7,17] x[7,18] x[7,19] x[7,20] x[7,21] x[7,22] x[7,23]
 x[8,0] x[8,1] x[8,2] x[8,3] x[8,4] x[8,5] x[8,6] x[8,7] x[8,8] x[8,9]
 x[8,10] x[8,11] x[8,12] x[8,13] x[8,14] x[8,15] x[8,16] x[8,17] x[8,18]
 x[8,19] x[8,20] x[8,21] x[8,22] x[8,23] x[9,0] x[9,1] x[9,2] x[9,3] x[9,4]
 x[9,5] x[9,6] x[9,7] x[9,8] x[9,9] x[9,10] x[9,11] x[9,12] x[9,13] x[9,14]
 x[9,15] x[9,16] x[9,17] x[9,18] x[9,19] x[9,20] x[9,21] x[9,22] x[9,23]
 x[10,0] x[10,1] x[10,2] x[10,3] x[10,4] x[10,5] x[10,6] x[10,7] x[10,8]
 x[10,9] x[10,10] x[10,11] x[10,12] x[10,13] x[10,14] x[10,15] x[10,16]
 x[10,17] x[10,18] x[10,19] x[10,20] x[10,21] x[10,22] x[10,23] x[11,0]
 x[11,1] x[11,2] x[11,3] x[11,4] x[11,5] x[11,6] x[11,7] x[11,8] x[11,9]
 x[11,10] x[11,11] x[11,12] x[11,13] x[11,14] x[11,15] x[11,16] x[11,17]
 x[11,18] x[11,19] x[11,20] x[11,21] x[11,22] x[11,23] x[12,0] x[12,1]
 x[12,2] x[12,3] x[12,4] x[12,5] x[12,6] x[12,7] x[12,8] x[12,9] x[12,10]
 x[12,11] x[12,12] x[12,13] x[12,14] x[12,15] x[12,16] x[12,17] x[12,18]
 x[12,19] x[12,20] x[12,21] x[12,22] x[12,23] x[13,0] x[13,1] x[13,2]
 x[13,3] x[13,4] x[13,5] x[13,6] x[13,7] x[13,8] x[13,9] x[13,10] x[13,11]
 x[13,12] x[13,13] x[13,14] x[13,15] x[13,16] x[13,17] x[13,18] x[13,19]
 x[13,20] x[13,21] x[13,22] x[13,23] x[14,0] x[14,1] x[14,2] x[14,3]
 x[14,4] x[14,5] x[14,6] x[14,7] x[14,8] x[14,9] x[14,10] x[14,11] x[14,12]
 x[14,13] x[14,14] x[14,15] x[14,16] x[14,17] x[14,18] x[14,19] x[14,20]
 x[14,21] x[14,22] x[14,23] x[15,0] x[15,1] x[15,2] x[15,3] x[15,4] x[15,5]
 x[15,6] x[15,7] x[15,8] x[15,9] x[15,10] x[15,11] x[15,12] x[15,13]
 x[15,14] x[15,15] x[15,16] x[15,17] x[15,18] x[15,19] x[15,20] x[15,21]
 x[15,22] x[15,23] x[16,0] x[16,1] x[16,2] x[16,3] x[16,4] x[16,5] x[16,6]
 x[16,7] x[16,8] x[16,9] x[16,10] x[16,11] x[16,12] x[16,13] x[16,14]
 x[16,15] x[16,16] x[16,17] x[16,18] x[16,19] x[16,20] x[16,21] x[16,22]
 x[16,23] x[17,0] x[17,1] x[17,2] x[17,3] x[17,4] x[17,5] x[17,6] x[17,7]
 x[17,8] x[17,9] x[17,10] x[17,11] x[17,12] x[17,13] x[17,14] x[17,15]
 x[17,16] x[17,17] x[17,18] x[17,19] x[17,20] x[17,21] x[17,22] x[17,23]
 x[18,0] x[18,1] x[18,2] x[18,3] x[18,4] x[18,5] x[18,6] x[18,7] x[18,8]
 x[18,9] x[18,10] x[18,11] x[18,12] x[18,13] x[18,14] x[18,15] x[18,16]
 x[18,17] x[18,18] x[18,19] x[18,20] x[18,21] x[18,22] x[18,23] x[19,0]
 x[19,1] x[19,2] x[19,3] x[19,4] x[19,5] x[19,6] x[19,7] x[19,8] x[19,9]
 x[19,10] x[19,11] x[19,12] x[19,13] x[19,14] x[19,15] x[19,16] x[19,17]
 x[19,18] x[19,19] x[19,20] x[19,21] x[19,22] x[19,23] x[20,0] x[20,1]
 x[20,2] x[20,3] x[20,4] x[20,5] x[20,6] x[20,7] x[20,8] x[20,9] x[20,10]
 x[20,11] x[20,12] x[20,13] x[20,14] x[20,15] x[20,16] x[20,17] x[20,18]
 x[20,19] x[20,20] x[20,21] x[20,22] x[20,23] x[21,0] x[21,1] x[21,2]
 x[21,3] x[21,4] x[21,5] x[21,6] x[21,7] x[21,8] x[21,9] x[21,10] x[21,11]
 x[21,12] x[21,13] x[21,14] x[21,15] x[21,16] x[21,17] x[21,18] x[21,19]
 x[21,20] x[21,21] x[21,22] x[21,23] x[22,0] x[22,1] x[22,2] x[22,3]
 x[22,4] x[22,5] x[22,6] x[22,7] x[22,8] x[22,9] x[22,10] x[22,11] x[22,12]
 x[22,13] x[22,14] x[22,15] x[22,16] x[22,17] x[22,18] x[22,19] x[22,20]
 x[22,21] x[22,22] x[22,23] x[23,0] x[23,1] x[23,2] x[23,3] x[23,4] x[23,5]
 x[23,6] x[23,7] x[23,8] x[23,9] x[23,10] x[23,11] x[23,12] x[23,13]
 x[23,14] x[23,15] x[23,16] x[23,17] x[23,18] x[23,19] x[23,20] x[23,21]
 x[23,22] x[23,23] v[0,0] v[0,1] v[0,2] v[0,3] v[0,4] v[0,5] v[0,6] v[0,7]
 v[0,8] v[0,9] v[0,10] v[0,11] v[0,12] v[0,13] v[0,14] v[0,15] v[0,16]
 v[0,17] v[0,18] v[0,19] v[0,20] v[0,21] v[0,22] v[0,23] v[1,0] v[1,1]
 v[1,2] v[1,3] v[1,4] v[1,5] v[1,6] v[1,7] v[1,8] v[1,9] v[1,10] v[1,11]
 v[1,12] v[1,13] v[1,14] v[1,15] v[1,16] v[1,17] v[1,18] v[1,19] v[1,20]
 v[1,21] v[1,22] v[1,23] v[2,0] v[2,1] v[2,2] v[2,3] v[2,4] v[2,5] v[2,6]
 v[2,7] v[2,8] v[2,9] v[2,10] v[2,11] v[2,12] v[2,13] v[2,14] v[2,15]
 v[2,16] v[2,17] v[2,18] v[2,19] v[2,20] v[2,21] v[2,22] v[2,23] v[3,0]
 v[3,1] v[3,2] v[3,3] v[3,4] v[3,5] v[3,6] v[3,7] v[3,8] v[3,9] v[3,10]
 v[3,11] v[3,12] v[3,13] v[3,14] v[3,15] v[3,16] v[3,17] v[3,18] v[3,19]
 v[3,20] v[3,21] v[3,22] v[3,23] v[4,0] v[4,1] v[4,2] v[4,3] v[4,4] v[4,5]
 v[4,6] v[4,7] v[4,8] v[4,9] v[4,10] v[4,11] v[4,12] v[4,13] v[4,14]
 v[4,15] v[4,16] v[4,17] v[4,18] v[4,19] v[4,20] v[4,21] v[4,22] v[4,23]
 v[5,0] v[5,1] v[5,2] v[5,3] v[5,4] v[5,5] v[5,6] v[5,7] v[5,8] v[5,9]
 v[5,10] v[5,11] v[5,12] v[5,13] v[5,14] v[5,15] v[5,16] v[5,17] v[5,18]
 v[5,19] v[5,20] v[5,21] v[5,22] v[5,23] v[6,0] v[6,1] v[6,2] v[6,3] v[6,4]
 v[6,5] v[6,6] v[6,7] v[6,8] v[6,9] v[6,10] v[6,11] v[6,12] v[6,13] v[6,14]
 v[6,15] v[6,16] v[6,17] v[6,18] v[6,19] v[6,20] v[6,21] v[6,22] v[6,23]
 v[7,0] v[7,1] v[7,2] v[7,3] v[7,4] v[7,5] v[7,6] v[7,7] v[7,8] v[7,9]
 v[7,10] v[7,11] v[7,12] v[7,13] v[7,14] v[7,15] v[7,16] v[7,17] v[7,18]
 v[7,19] v[7,20] v[7,21] v[7,22] v[7,23] v[8,0] v[8,1] v[8,2] v[8,3] v[8,4]
 v[8,5] v[8,6] v[8,7] v[8,8] v[8,9] v[8,10] v[8,11] v[8,12] v[8,13] v[8,14]
 v[8,15] v[8,16] v[8,17] v[8,18] v[8,19] v[8,20] v[8,21] v[8,22] v[8,23]
 v[9,0] v[9,1] v[9,2] v[9,3] v[9,4] v[9,5] v[9,6] v[9,7] v[9,8] v[9,9]
 v[9,10] v[9,11] v[9,12] v[9,13] v[9,14] v[9,15] v[9,16] v[9,17] v[9,18]
 v[9,19] v[9,20] v[9,21] v[9,22] v[9,23] v[10,0] v[10,1] v[10,2] v[10,3]
 v[10,4] v[10,5] v[10,6] v[10,7] v[10,8] v[10,9] v[10,10] v[10,11] v[10,12]
 v[10,13] v[10,14] v[10,15] v[10,16] v[10,17] v[10,18] v[10,19] v[10,20]
 v[10,21] v[10,22] v[10,23] v[11,0] v[11,1] v[11,2] v[11,3] v[11,4] v[11,5]
 v[11,6] v[11,7] v[11,8] v[11,9] v[11,10] v[11,11] v[11,12] v[11,13]
 v[11,14] v[11,15] v[11,16] v[11,17] v[11,18] v[11,19] v[11,20] v[11,21]
 v[11,22] v[11,23] v[12,0] v[12,1] v[12,2] v[12,3] v[12,4] v[12,5] v[12,6]
 v[12,7] v[12,8] v[12,9] v[12,10] v[12,11] v[12,12] v[12,13] v[12,14]
 v[12,15] v[12,16] v[12,17] v[12,18] v[12,19] v[12,20] v[12,21] v[12,22]
 v[12,23] v[13,0] v[13,1] v[13,2] v[13,3] v[13,4] v[13,5] v[13,6] v[13,7]
 v[13,8] v[13,9] v[13,10] v[13,11] v[13,12] v[13,13] v[13,14] v[13,15]
 v[13,16] v[13,17] v[13,18] v[13,19] v[13,20] v[13,21] v[13,22] v[13,23]
 v[14,0] v[14,1] v[14,2] v[14,3] v[14,4] v[14,5] v[14,6] v[14,7] v[14,8]
 v[14,9] v[14,10] v[14,11] v[14,12] v[14,13] v[14,14] v[14,15] v[14,16]
 v[14,17] v[14,18] v[14,19] v[14,20] v[14,21] v[14,22] v[14,23] v[15,0]
 v[15,1] v[15,2] v[15,3] v[15,4] v[15,5] v[15,6] v[15,7] v[15,8] v[15,9]
 v[15,10] v[15,11] v[15,12] v[15,13] v[15,14] v[15,15] v[15,16] v[15,17]
 v[15,18] v[15,19] v[15,20] v[15,21] v[15,22] v[15,23] v[16,0] v[16,1]
 v[16,2] v[16,3] v[16,4] v[16,5] v[16,6] v[16,7] v[16,8] v[16,9] v[16,10]
 v[16,11] v[16,12] v[16,13] v[16,14] v[16,15] v[16,16] v[16,17] v[16,18]
 v[16,19] v[16,20] v[16,21] v[16,22] v[16,23] v[17,0] v[17,1] v[17,2]
 v[17,3] v[17,4] v[17,5] v[17,6] v[17,7] v[17,8] v[17,9] v[17,10] v[17,11]
 v[17,12] v[17,13] v[17,14] v[17,15] v[17,16] v[17,17] v[17,18] v[17,19]
 v[17,20] v[17,21] v[17,22] v[17,23] v[18,0] v[18,1] v[18,2] v[18,3]
 v[18,4] v[18,5] v[18,6] v[18,7] v[18,8] v[18,9] v[18,10] v[18,11] v[18,12]
 v[18,13] v[18,14] v[18,15] v[18,16] v[18,17] v[18,18] v[18,19] v[18,20]
 v[18,21] v[18,22] v[18,23] v[19,0] v[19,1] v[19,2] v[19,3] v[19,4] v[19,5]
 v[19,6] v[19,7] v[19,8] v[19,9] v[19,10] v[19,11] v[19,12] v[19,13]
 v[19,14] v[19,15] v[19,16] v[19,17] v[19,18] v[19,19] v[19,20] v[19,21]
 v[19,22] v[19,23] v[20,0] v[20,1] v[20,2] v[20,3] v[20,4] v[20,5] v[20,6]
 v[20,7] v[20,8] v[20,9] v[20,10] v[20,11] v[20,12] v[20,13] v[20,14]
 v[20,15] v[20,16] v[20,17] v[20,18] v[20,19] v[20,20] v[20,21] v[20,22]
 v[20,23] v[21,0] v[21,1] v[21,2] v[21,3] v[21,4] v[21,5] v[21,6] v[21,7]
 v[21,8] v[21,9] v[21,10] v[21,11] v[21,12] v[21,13] v[21,14] v[21,15]
 v[21,16] v[21,17] v[21,18] v[21,19] v[21,20] v[21,21] v[21,22] v[21,23]
 v[22,0] v[22,1] v[22,2] v[22,3] v[22,4] v[22,5] v[22,6] v[22,7] v[22,8]
 v[22,9] v[22,10] v[22,11] v[22,12] v[22,13] v[22,14] v[22,15] v[22,16]
 v[22,17] v[22,18] v[22,19] v[22,20] v[22,21] v[22,22] v[22,23] v[23,0]
 v[23,1] v[23,2] v[23,3] v[23,4] v[23,5] v[23,6] v[23,7] v[23,8] v[23,9]
 v[23,10] v[23,11] v[23,12] v[23,13] v[23,14] v[23,15] v[23,16] v[23,17]
 v[23,18] v[23,19] v[23,20] v[23,21] v[23,22] v[23,23] y[2,1] y[2,2] y[3,1]
 y[3,2] y[4,1] y[4,2] y[5,1] y[5,2] y[6,1] y[6,2] y[7,1] y[7,2] y[8,1]
 y[8,2] y[9,1] y[9,2] y[10,1] y[10,2] y[11,1] y[11,2]
Generals
 w[0,0] w[0,1] w[0,2] w[0,3] w[0,4] w[0,5] w[0,6] w[0,7] w[0,8] w[0,9]
 w[0,10] w[0,11] w[0,12] w[0,13] w[0,14] w[0,15] w[0,16] w[0,17] w[0,18]
 w[0,19] w[0,20] w[0,21] w[0,22] w[0,23] w[1,0] w[1,1] w[1,2] w[1,3] w[1,4]
 w[1,5] w[1,6] w[1,7] w[1,8] w[1,9] w[1,10] w[1,11] w[1,12] w[1,13] w[1,14]
 w[1,15] w[1,16] w[1,17] w[1,18] w[1,19] w[1,20] w[1,21] w[1,22] w[1,23]
 w[2,0] w[2,1] w[2,2] w[2,3] w[2,4] w[2,5] w[2,6] w[2,7] w[2,8] w[2,9]
 w[2,10] w[2,11] w[2,12] w[2,13] w[2,14] w[2,15] w[2,16] w[2,17] w[2,18]
 w[2,19] w[2,20] w[2,21] w[2,22] w[2,23] w[3,0] w[3,1] w[3,2] w[3,3] w[3,4]
 w[3,5] w[3,6] w[3,7] w[3,8] w[3,9] w[3,10] w[3,11] w[3,12] w[3,13] w[3,14]
 w[3,15] w[3,16] w[3,17] w[3,18] w[3,19] w[3,20] w[3,21] w[3,22] w[3,23]
 w[4,0] w[4,1] w[4,2] w[4,3] w[4,4] w[4,5] w[4,6] w[4,7] w[4,8] w[4,9]
 w[4,10] w[4,11] w[4,12] w[4,13] w[4,14] w[4,15] w[4,16] w[4,17] w[4,18]
 w[4,19] w[4,20] w[4,21] w[4,22] w[4,23] w[5,0] w[5,1] w[5,2] w[5,3] w[5,4]
 w[5,5] w[5,6] w[5,7] w[5,8] w[5,9] w[5,10] w[5,11] w[5,12] w[5,13] w[5,14]
 w[5,15] w[5,16] w[5,17] w[5,18] w[5,19] w[5,20] w[5,21] w[5,22] w[5,23]
 w[6,0] w[6,1] w[6,2] w[6,3] w[6,4] w[6,5] w[6,6] w[6,7] w[6,8] w[6,9]
 w[6,10] w[6,11] w[6,12] w[6,13] w[6,14] w[6,15] w[6,16] w[6,17] w[6,18]
 w[6,19] w[6,20] w[6,21] w[6,22] w[6,23] w[7,0] w[7,1] w[7,2] w[7,3] w[7,4]
 w[7,5] w[7,6] w[7,7] w[7,8] w[7,9] w[7,10] w[7,11] w[7,12] w[7,13] w[7,14]
 w[7,15] w[7,16] w[7,17] w[7,18] w[7,19] w[7,20] w[7,21] w[7,22] w[7,23]
 w[8,0] w[8,1] w[8,2] w[8,3] w[8,4] w[8,5] w[8,6] w[8,7] w[8,8] w[8,9]
 w[8,10] w[8,11] w[8,12] w[8,13] w[8,14] w[8,15] w[8,16] w[8,17] w[8,18]
 w[8,19] w[8,20] w[8,21] w[8,22] w[8,23] w[9,0] w[9,1] w[9,2] w[9,3] w[9,4]
 w[9,5] w[9,6] w[9,7] w[9,8] w[9,9] w[9,10] w[9,11] w[9,12] w[9,13] w[9,14]
 w[9,15] w[9,16] w[9,17] w[9,18] w[9,19] w[9,20] w[9,21] w[9,22] w[9,23]
 w[10,0] w[10,1] w[10,2] w[10,3] w[10,4] w[10,5] w[10,6] w[10,7] w[10,8]
 w[10,9] w[10,10] w[10,11] w[10,12] w[10,13] w[10,14] w[10,15] w[10,16]
 w[10,17] w[10,18] w[10,19] w[10,20] w[10,21] w[10,22] w[10,23] w[11,0]
 w[11,1] w[11,2] w[11,3] w[11,4] w[11,5] w[11,6] w[11,7] w[11,8] w[11,9]
 w[11,10] w[11,11] w[11,12] w[11,13] w[11,14] w[11,15] w[11,16] w[11,17]
 w[11,18] w[11,19] w[11,20] w[11,21] w[11,22] w[11,23] w[12,0] w[12,1]
 w[12,2] w[12,3] w[12,4] w[12,5] w[12,6] w[12,7] w[12,8] w[12,9] w[12,10]
 w[12,11] w[12,12] w[12,13] w[12,14] w[12,15] w[12,16] w[12,17] w[12,18]
 w[12,19] w[12,20] w[12,21] w[12,22] w[12,23] w[13,0] w[13,1] w[13,2]
 w[13,3] w[13,4] w[13,5] w[13,6] w[13,7] w[13,8] w[13,9] w[13,10] w[13,11]
 w[13,12] w[13,13] w[13,14] w[13,15] w[13,16] w[13,17] w[13,18] w[13,19]
 w[13,20] w[13,21] w[13,22] w[13,23] w[14,0] w[14,1] w[14,2] w[14,3]
 w[14,4] w[14,5] w[14,6] w[14,7] w[14,8] w[14,9] w[14,10] w[14,11] w[14,12]
 w[14,13] w[14,14] w[14,15] w[14,16] w[14,17] w[14,18] w[14,19] w[14,20]
 w[14,21] w[14,22] w[14,23] w[15,0] w[15,1] w[15,2] w[15,3] w[15,4] w[15,5]
 w[15,6] w[15,7] w[15,8] w[15,9] w[15,10] w[15,11] w[15,12] w[15,13]
 w[15,14] w[15,15] w[15,16] w[15,17] w[15,18] w[15,19] w[15,20] w[15,21]
 w[15,22] w[15,23] w[16,0] w[16,1] w[16,2] w[16,3] w[16,4] w[16,5] w[16,6]
 w[16,7] w[16,8] w[16,9] w[16,10] w[16,11] w[16,12] w[16,13] w[16,14]
 w[16,15] w[16,16] w[16,17] w[16,18] w[16,19] w[16,20] w[16,21] w[16,22]
 w[16,23] w[17,0] w[17,1] w[17,2] w[17,3] w[17,4] w[17,5] w[17,6] w[17,7]
 w[17,8] w[17,9] w[17,10] w[17,11] w[17,12] w[17,13] w[17,14] w[17,15]
 w[17,16] w[17,17] w[17,18] w[17,19] w[17,20] w[17,21] w[17,22] w[17,23]
 w[18,0] w[18,1] w[18,2] w[18,3] w[18,4] w[18,5] w[18,6] w[18,7] w[18,8]
 w[18,9] w[18,10] w[18,11] w[18,12] w[18,13] w[18,14] w[18,15] w[18,16]
 w[18,17] w[18,18] w[18,19] w[18,20] w[18,21] w[18,22] w[18,23] w[19,0]
 w[19,1] w[19,2] w[19,3] w[19,4] w[19,5] w[19,6] w[19,7] w[19,8] w[19,9]
 w[19,10] w[19,11] w[19,12] w[19,13] w[19,14] w[19,15] w[19,16] w[19,17]
 w[19,18] w[19,19] w[19,20] w[19,21] w[19,22] w[19,23] w[20,0] w[20,1]
 w[20,2] w[20,3] w[20,4] w[20,5] w[20,6] w[20,7] w[20,8] w[20,9] w[20,10]
 w[20,11] w[20,12] w[20,13] w[20,14] w[20,15] w[20,16] w[20,17] w[20,18]
 w[20,19] w[20,20] w[20,21] w[20,22] w[20,23] w[21,0] w[21,1] w[21,2]
 w[21,3] w[21,4] w[21,5] w[21,6] w[21,7] w[21,8] w[21,9] w[21,10] w[21,11]
 w[21,12] w[21,13] w[21,14] w[21,15] w[21,16] w[21,17] w[21,18] w[21,19]
 w[21,20] w[21,21] w[21,22] w[21,23] w[22,0] w[22,1] w[22,2] w[22,3]
 w[22,4] w[22,5] w[22,6] w[22,7] w[22,8] w[22,9] w[22,10] w[22,11] w[22,12]
 w[22,13] w[22,14] w[22,15] w[22,16] w[22,17] w[22,18] w[22,19] w[22,20]
 w[22,21] w[22,22] w[22,23] w[23,0] w[23,1] w[23,2] w[23,3] w[23,4] w[23,5]
 w[23,6] w[23,7] w[23,8] w[23,9] w[23,10] w[23,11] w[23,12] w[23,13]
 w[23,14] w[23,15] w[23,16] w[23,17] w[23,18] w[23,19] w[23,20] w[23,21]
 w[23,22] w[23,23]
End
