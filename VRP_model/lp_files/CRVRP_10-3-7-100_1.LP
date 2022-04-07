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
   + start_service[2] >= -999968.5896813161
 NodeArrivalVehicles[0,3]: - 999999 x[0,3] - start_service[0]
   + start_service[3] >= -999952.9831366809
 NodeArrivalVehicles[0,4]: - 999999 x[0,4] - start_service[0]
   + start_service[4] >= -999918.8440504119
 NodeArrivalVehicles[0,5]: - 999999 x[0,5] - start_service[0]
   + start_service[5] >= -999906.8636006397
 NodeArrivalVehicles[0,6]: - 999999 x[0,6] - start_service[0]
   + start_service[6] >= -999931.7152103848
 NodeArrivalVehicles[0,7]: - 999999 x[0,7] - start_service[0]
   + start_service[7] >= -999935.9092929828
 NodeArrivalVehicles[0,8]: - 999999 x[0,8] - start_service[0]
   + start_service[8] >= -999948.4803299305
 NodeArrivalVehicles[0,9]: - 999999 x[0,9] - start_service[0]
   + start_service[9] >= -999979.8163579979
 NodeArrivalVehicles[0,10]: - 999999 x[0,10] - start_service[0]
   + start_service[10] >= -999968.7963188644
 NodeArrivalVehicles[0,11]: - 999999 x[0,11] - start_service[0]
   + start_service[11] >= -999911.607195191
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999968.5896813161
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999952.9831366809
 NodeArrivalVehicles[0,14]: - 999999 x[0,14] - start_service[0]
   + start_service[14] >= -999918.8440504119
 NodeArrivalVehicles[0,15]: - 999999 x[0,15] - start_service[0]
   + start_service[15] >= -999906.8636006397
 NodeArrivalVehicles[0,16]: - 999999 x[0,16] - start_service[0]
   + start_service[16] >= -999931.7152103848
 NodeArrivalVehicles[0,17]: - 999999 x[0,17] - start_service[0]
   + start_service[17] >= -999935.9092929828
 NodeArrivalVehicles[0,18]: - 999999 x[0,18] - start_service[0]
   + start_service[18] >= -999948.4803299305
 NodeArrivalVehicles[0,19]: - 999999 x[0,19] - start_service[0]
   + start_service[19] >= -999979.8163579979
 NodeArrivalVehicles[0,20]: - 999999 x[0,20] - start_service[0]
   + start_service[20] >= -999968.7963188644
 NodeArrivalVehicles[0,21]: - 999999 x[0,21] - start_service[0]
   + start_service[21] >= -999911.607195191
 NodeArrivalVehicles[0,22]: - 999999 x[0,22] - start_service[0]
   + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999 x[0,23] - start_service[0]
   + start_service[23] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
   + start_service[2] >= -999968.5896813161
 NodeArrivalVehicles[1,3]: - 999999 x[1,3] - start_service[1]
   + start_service[3] >= -999952.9831366809
 NodeArrivalVehicles[1,4]: - 999999 x[1,4] - start_service[1]
   + start_service[4] >= -999918.8440504119
 NodeArrivalVehicles[1,5]: - 999999 x[1,5] - start_service[1]
   + start_service[5] >= -999906.8636006397
 NodeArrivalVehicles[1,6]: - 999999 x[1,6] - start_service[1]
   + start_service[6] >= -999931.7152103848
 NodeArrivalVehicles[1,7]: - 999999 x[1,7] - start_service[1]
   + start_service[7] >= -999935.9092929828
 NodeArrivalVehicles[1,8]: - 999999 x[1,8] - start_service[1]
   + start_service[8] >= -999948.4803299305
 NodeArrivalVehicles[1,9]: - 999999 x[1,9] - start_service[1]
   + start_service[9] >= -999979.8163579979
 NodeArrivalVehicles[1,10]: - 999999 x[1,10] - start_service[1]
   + start_service[10] >= -999968.7963188644
 NodeArrivalVehicles[1,11]: - 999999 x[1,11] - start_service[1]
   + start_service[11] >= -999911.607195191
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999968.5896813161
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999952.9831366809
 NodeArrivalVehicles[1,14]: - 999999 x[1,14] - start_service[1]
   + start_service[14] >= -999918.8440504119
 NodeArrivalVehicles[1,15]: - 999999 x[1,15] - start_service[1]
   + start_service[15] >= -999906.8636006397
 NodeArrivalVehicles[1,16]: - 999999 x[1,16] - start_service[1]
   + start_service[16] >= -999931.7152103848
 NodeArrivalVehicles[1,17]: - 999999 x[1,17] - start_service[1]
   + start_service[17] >= -999935.9092929828
 NodeArrivalVehicles[1,18]: - 999999 x[1,18] - start_service[1]
   + start_service[18] >= -999948.4803299305
 NodeArrivalVehicles[1,19]: - 999999 x[1,19] - start_service[1]
   + start_service[19] >= -999979.8163579979
 NodeArrivalVehicles[1,20]: - 999999 x[1,20] - start_service[1]
   + start_service[20] >= -999968.7963188644
 NodeArrivalVehicles[1,21]: - 999999 x[1,21] - start_service[1]
   + start_service[21] >= -999911.607195191
 NodeArrivalVehicles[1,22]: - 999999 x[1,22] - start_service[1]
   + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999 x[1,23] - start_service[1]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999968.5896813161
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 999999 x[2,3] - start_service[2]
   + start_service[3] >= -999980.7297370874
 NodeArrivalVehicles[2,4]: - 999999 x[2,4] - start_service[2]
   + start_service[4] >= -999925.7860261707
 NodeArrivalVehicles[2,5]: - 999999 x[2,5] - start_service[2]
   + start_service[5] >= -999925.4159407663
 NodeArrivalVehicles[2,6]: - 999999 x[2,6] - start_service[2]
   + start_service[6] >= -999960.8785410399
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999963.4348783819
 NodeArrivalVehicles[2,8]: - 999999 x[2,8] - start_service[2]
   + start_service[8] >= -999954.5687754838
 NodeArrivalVehicles[2,9]: - 999999 x[2,9] - start_service[2]
   + start_service[9] >= -999975.0440079125
 NodeArrivalVehicles[2,10]: - 999999 x[2,10] - start_service[2]
   + start_service[10] >= -999975.5815039668
 NodeArrivalVehicles[2,11]: - 999999 x[2,11] - start_service[2]
   + start_service[11] >= -999930.4704496202
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - start_service[2]
   + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999980.7297370874
 NodeArrivalVehicles[2,14]: - 999999 x[2,14] - start_service[2]
   + start_service[14] >= -999925.7860261707
 NodeArrivalVehicles[2,15]: - 999999 x[2,15] - start_service[2]
   + start_service[15] >= -999925.4159407663
 NodeArrivalVehicles[2,16]: - 999999 x[2,16] - start_service[2]
   + start_service[16] >= -999960.8785410399
 NodeArrivalVehicles[2,17]: - 999999 x[2,17] - start_service[2]
   + start_service[17] >= -999963.4348783819
 NodeArrivalVehicles[2,18]: - 999999 x[2,18] - start_service[2]
   + start_service[18] >= -999954.5687754838
 NodeArrivalVehicles[2,19]: - 999999 x[2,19] - start_service[2]
   + start_service[19] >= -999975.0440079125
 NodeArrivalVehicles[2,20]: - 999999 x[2,20] - start_service[2]
   + start_service[20] >= -999975.5815039668
 NodeArrivalVehicles[2,21]: - 999999 x[2,21] - start_service[2]
   + start_service[21] >= -999930.4704496202
 NodeArrivalVehicles[2,22]: - 999999 x[2,22] - start_service[2]
   + start_service[22] >= -999968.5896813161
 NodeArrivalVehicles[2,23]: - 999999 x[2,23] - start_service[2]
   + start_service[23] >= -999968.5896813161
 NodeArrivalVehicles[3,1]: - 999999 x[3,1] + start_service[1]
   - start_service[3] >= -999952.9831366809
 NodeArrivalVehicles[3,2]: - 999999 x[3,2] + start_service[2]
   - start_service[3] >= -999980.7297370874
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 999999 x[3,4] - start_service[3]
   + start_service[4] >= -999936.2884880449
 NodeArrivalVehicles[3,5]: - 999999 x[3,5] - start_service[3]
   + start_service[5] >= -999941.9567081289
 NodeArrivalVehicles[3,6]: - 999999 x[3,6] - start_service[3]
   + start_service[6] >= -999965.5264110569
 NodeArrivalVehicles[3,7]: - 999999 x[3,7] - start_service[3]
   + start_service[7] >= -999963.9361194944
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999960.9544713456
 NodeArrivalVehicles[3,9]: - 999999 x[3,9] - start_service[3]
   + start_service[9] >= -999956.7738627526
 NodeArrivalVehicles[3,10]: - 999999 x[3,10] - start_service[3]
   + start_service[10] >= -999958.4815577437
 NodeArrivalVehicles[3,11]: - 999999 x[3,11] - start_service[3]
   + start_service[11] >= -999947.0066322301
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - start_service[3]
   + start_service[12] >= -999980.7297370874
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 999999 x[3,14] - start_service[3]
   + start_service[14] >= -999936.2884880449
 NodeArrivalVehicles[3,15]: - 999999 x[3,15] - start_service[3]
   + start_service[15] >= -999941.9567081289
 NodeArrivalVehicles[3,16]: - 999999 x[3,16] - start_service[3]
   + start_service[16] >= -999965.5264110569
 NodeArrivalVehicles[3,17]: - 999999 x[3,17] - start_service[3]
   + start_service[17] >= -999963.9361194944
 NodeArrivalVehicles[3,18]: - 999999 x[3,18] - start_service[3]
   + start_service[18] >= -999960.9544713456
 NodeArrivalVehicles[3,19]: - 999999 x[3,19] - start_service[3]
   + start_service[19] >= -999956.7738627526
 NodeArrivalVehicles[3,20]: - 999999 x[3,20] - start_service[3]
   + start_service[20] >= -999958.4815577437
 NodeArrivalVehicles[3,21]: - 999999 x[3,21] - start_service[3]
   + start_service[21] >= -999947.0066322301
 NodeArrivalVehicles[3,22]: - 999999 x[3,22] - start_service[3]
   + start_service[22] >= -999952.9831366809
 NodeArrivalVehicles[3,23]: - 999999 x[3,23] - start_service[3]
   + start_service[23] >= -999952.9831366809
 NodeArrivalVehicles[4,1]: - 999999 x[4,1] + start_service[1]
   - start_service[4] >= -999918.8440504119
 NodeArrivalVehicles[4,2]: - 999999 x[4,2] + start_service[2]
   - start_service[4] >= -999925.7860261707
 NodeArrivalVehicles[4,3]: - 999999 x[4,3] + start_service[3]
   - start_service[4] >= -999936.2884880449
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 999999 x[4,5] - start_service[4]
   + start_service[5] >= -999964.8446477182
 NodeArrivalVehicles[4,6]: - 999999 x[4,6] - start_service[4]
   + start_service[6] >= -999905.4897305039
 NodeArrivalVehicles[4,7]: - 999999 x[4,7] - start_service[4]
   + start_service[7] >= -999901.9982294367
 NodeArrivalVehicles[4,8]: - 999999 x[4,8] - start_service[4]
   + start_service[8] >= -999968.8070104932
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999908.2748401375
 NodeArrivalVehicles[4,10]: - 999999 x[4,10] - start_service[4]
   + start_service[10] >= -999903.5978504075
 NodeArrivalVehicles[4,11]: - 999999 x[4,11] - start_service[4]
   + start_service[11] >= -999965.4410353305
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - start_service[4]
   + start_service[12] >= -999925.7860261707
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - start_service[4]
   + start_service[13] >= -999936.2884880449
 NodeArrivalVehicles[4,14]: - 999999 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 999999 x[4,15] - start_service[4]
   + start_service[15] >= -999964.8446477182
 NodeArrivalVehicles[4,16]: - 999999 x[4,16] - start_service[4]
   + start_service[16] >= -999905.4897305039
 NodeArrivalVehicles[4,17]: - 999999 x[4,17] - start_service[4]
   + start_service[17] >= -999901.9982294367
 NodeArrivalVehicles[4,18]: - 999999 x[4,18] - start_service[4]
   + start_service[18] >= -999968.8070104932
 NodeArrivalVehicles[4,19]: - 999999 x[4,19] - start_service[4]
   + start_service[19] >= -999908.2748401375
 NodeArrivalVehicles[4,20]: - 999999 x[4,20] - start_service[4]
   + start_service[20] >= -999903.5978504075
 NodeArrivalVehicles[4,21]: - 999999 x[4,21] - start_service[4]
   + start_service[21] >= -999965.4410353305
 NodeArrivalVehicles[4,22]: - 999999 x[4,22] - start_service[4]
   + start_service[22] >= -999918.8440504119
 NodeArrivalVehicles[4,23]: - 999999 x[4,23] - start_service[4]
   + start_service[23] >= -999918.8440504119
 NodeArrivalVehicles[5,1]: - 999999 x[5,1] + start_service[1]
   - start_service[5] >= -999906.8636006397
 NodeArrivalVehicles[5,2]: - 999999 x[5,2] + start_service[2]
   - start_service[5] >= -999925.4159407663
 NodeArrivalVehicles[5,3]: - 999999 x[5,3] + start_service[3]
   - start_service[5] >= -999941.9567081289
 NodeArrivalVehicles[5,4]: - 999999 x[5,4] + start_service[4]
   - start_service[5] >= -999964.8446477182
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 999999 x[5,6] - start_service[5]
   + start_service[6] >= -999920.1584135875
 NodeArrivalVehicles[5,7]: - 999999 x[5,7] - start_service[5]
   + start_service[7] >= -999914.3058314386
 NodeArrivalVehicles[5,8]: - 999999 x[5,8] - start_service[5]
   + start_service[8] >= -999953.1311389591
 NodeArrivalVehicles[5,9]: - 999999 x[5,9] - start_service[5]
   + start_service[9] >= -999902.8077036525
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999902.0796829936
 NodeArrivalVehicles[5,11]: - 999999 x[5,11] - start_service[5]
   + start_service[11] >= -999993.937777768
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - start_service[5]
   + start_service[12] >= -999925.4159407663
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - start_service[5]
   + start_service[13] >= -999941.9567081289
 NodeArrivalVehicles[5,14]: - 999999 x[5,14] - start_service[5]
   + start_service[14] >= -999964.8446477182
 NodeArrivalVehicles[5,15]: - 999999 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 999999 x[5,16] - start_service[5]
   + start_service[16] >= -999920.1584135875
 NodeArrivalVehicles[5,17]: - 999999 x[5,17] - start_service[5]
   + start_service[17] >= -999914.3058314386
 NodeArrivalVehicles[5,18]: - 999999 x[5,18] - start_service[5]
   + start_service[18] >= -999953.1311389591
 NodeArrivalVehicles[5,19]: - 999999 x[5,19] - start_service[5]
   + start_service[19] >= -999902.8077036525
 NodeArrivalVehicles[5,20]: - 999999 x[5,20] - start_service[5]
   + start_service[20] >= -999902.0796829936
 NodeArrivalVehicles[5,21]: - 999999 x[5,21] - start_service[5]
   + start_service[21] >= -999993.937777768
 NodeArrivalVehicles[5,22]: - 999999 x[5,22] - start_service[5]
   + start_service[22] >= -999906.8636006397
 NodeArrivalVehicles[5,23]: - 999999 x[5,23] - start_service[5]
   + start_service[23] >= -999906.8636006397
 NodeArrivalVehicles[6,1]: - 999999 x[6,1] + start_service[1]
   - start_service[6] >= -999931.7152103848
 NodeArrivalVehicles[6,2]: - 999999 x[6,2] + start_service[2]
   - start_service[6] >= -999960.8785410399
 NodeArrivalVehicles[6,3]: - 999999 x[6,3] + start_service[3]
   - start_service[6] >= -999965.5264110569
 NodeArrivalVehicles[6,4]: - 999999 x[6,4] + start_service[4]
   - start_service[6] >= -999905.4897305039
 NodeArrivalVehicles[6,5]: - 999999 x[6,5] + start_service[5]
   - start_service[6] >= -999920.1584135875
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 999999 x[6,7] - start_service[6]
   + start_service[7] >= -999990.8380737376
 NodeArrivalVehicles[6,8]: - 999999 x[6,8] - start_service[6]
   + start_service[8] >= -999927.6511923969
 NodeArrivalVehicles[6,9]: - 999999 x[6,9] - start_service[6]
   + start_service[9] >= -999945.0669546484
 NodeArrivalVehicles[6,10]: - 999999 x[6,10] - start_service[6]
   + start_service[10] >= -999953.8583916504
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999924.6895146155
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - start_service[6]
   + start_service[12] >= -999960.8785410399
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - start_service[6]
   + start_service[13] >= -999965.5264110569
 NodeArrivalVehicles[6,14]: - 999999 x[6,14] - start_service[6]
   + start_service[14] >= -999905.4897305039
 NodeArrivalVehicles[6,15]: - 999999 x[6,15] - start_service[6]
   + start_service[15] >= -999920.1584135875
 NodeArrivalVehicles[6,16]: - 999999 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 999999 x[6,17] - start_service[6]
   + start_service[17] >= -999990.8380737376
 NodeArrivalVehicles[6,18]: - 999999 x[6,18] - start_service[6]
   + start_service[18] >= -999927.6511923969
 NodeArrivalVehicles[6,19]: - 999999 x[6,19] - start_service[6]
   + start_service[19] >= -999945.0669546484
 NodeArrivalVehicles[6,20]: - 999999 x[6,20] - start_service[6]
   + start_service[20] >= -999953.8583916504
 NodeArrivalVehicles[6,21]: - 999999 x[6,21] - start_service[6]
   + start_service[21] >= -999924.6895146155
 NodeArrivalVehicles[6,22]: - 999999 x[6,22] - start_service[6]
   + start_service[22] >= -999931.7152103848
 NodeArrivalVehicles[6,23]: - 999999 x[6,23] - start_service[6]
   + start_service[23] >= -999931.7152103848
 NodeArrivalVehicles[7,1]: - 999999 x[7,1] + start_service[1]
   - start_service[7] >= -999935.9092929828
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999963.4348783819
 NodeArrivalVehicles[7,3]: - 999999 x[7,3] + start_service[3]
   - start_service[7] >= -999963.9361194944
 NodeArrivalVehicles[7,4]: - 999999 x[7,4] + start_service[4]
   - start_service[7] >= -999901.9982294367
 NodeArrivalVehicles[7,5]: - 999999 x[7,5] + start_service[5]
   - start_service[7] >= -999914.3058314386
 NodeArrivalVehicles[7,6]: - 999999 x[7,6] + start_service[6]
   - start_service[7] >= -999990.8380737376
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 999999 x[7,8] - start_service[7]
   + start_service[8] >= -999925.9744337149
 NodeArrivalVehicles[7,9]: - 999999 x[7,9] - start_service[7]
   + start_service[9] >= -999950.7231816619
 NodeArrivalVehicles[7,10]: - 999999 x[7,10] - start_service[7]
   + start_service[10] >= -999960.3013267835
 NodeArrivalVehicles[7,11]: - 999999 x[7,11] - start_service[7]
   + start_service[11] >= -999918.9888256516
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999963.4348783819
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - start_service[7]
   + start_service[13] >= -999963.9361194944
 NodeArrivalVehicles[7,14]: - 999999 x[7,14] - start_service[7]
   + start_service[14] >= -999901.9982294367
 NodeArrivalVehicles[7,15]: - 999999 x[7,15] - start_service[7]
   + start_service[15] >= -999914.3058314386
 NodeArrivalVehicles[7,16]: - 999999 x[7,16] - start_service[7]
   + start_service[16] >= -999990.8380737376
 NodeArrivalVehicles[7,17]: - 999999 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 999999 x[7,18] - start_service[7]
   + start_service[18] >= -999925.9744337149
 NodeArrivalVehicles[7,19]: - 999999 x[7,19] - start_service[7]
   + start_service[19] >= -999950.7231816619
 NodeArrivalVehicles[7,20]: - 999999 x[7,20] - start_service[7]
   + start_service[20] >= -999960.3013267835
 NodeArrivalVehicles[7,21]: - 999999 x[7,21] - start_service[7]
   + start_service[21] >= -999918.9888256516
 NodeArrivalVehicles[7,22]: - 999999 x[7,22] - start_service[7]
   + start_service[22] >= -999935.9092929828
 NodeArrivalVehicles[7,23]: - 999999 x[7,23] - start_service[7]
   + start_service[23] >= -999935.9092929828
 NodeArrivalVehicles[8,1]: - 999999 x[8,1] + start_service[1]
   - start_service[8] >= -999948.4803299305
 NodeArrivalVehicles[8,2]: - 999999 x[8,2] + start_service[2]
   - start_service[8] >= -999954.5687754838
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999960.9544713456
 NodeArrivalVehicles[8,4]: - 999999 x[8,4] + start_service[4]
   - start_service[8] >= -999968.8070104932
 NodeArrivalVehicles[8,5]: - 999999 x[8,5] + start_service[5]
   - start_service[8] >= -999953.1311389591
 NodeArrivalVehicles[8,6]: - 999999 x[8,6] + start_service[6]
   - start_service[8] >= -999927.6511923969
 NodeArrivalVehicles[8,7]: - 999999 x[8,7] + start_service[7]
   - start_service[8] >= -999925.9744337149
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 999999 x[8,9] - start_service[8]
   + start_service[9] >= -999938.4387890749
 NodeArrivalVehicles[8,10]: - 999999 x[8,10] - start_service[8]
   + start_service[10] >= -999933.3155508919
 NodeArrivalVehicles[8,11]: - 999999 x[8,11] - start_service[8]
   + start_service[11] >= -999957.0143215929
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - start_service[8]
   + start_service[12] >= -999954.5687754838
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - start_service[8]
   + start_service[13] >= -999960.9544713456
 NodeArrivalVehicles[8,14]: - 999999 x[8,14] - start_service[8]
   + start_service[14] >= -999968.8070104932
 NodeArrivalVehicles[8,15]: - 999999 x[8,15] - start_service[8]
   + start_service[15] >= -999953.1311389591
 NodeArrivalVehicles[8,16]: - 999999 x[8,16] - start_service[8]
   + start_service[16] >= -999927.6511923969
 NodeArrivalVehicles[8,17]: - 999999 x[8,17] - start_service[8]
   + start_service[17] >= -999925.9744337149
 NodeArrivalVehicles[8,18]: - 999999 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 999999 x[8,19] - start_service[8]
   + start_service[19] >= -999938.4387890749
 NodeArrivalVehicles[8,20]: - 999999 x[8,20] - start_service[8]
   + start_service[20] >= -999933.3155508919
 NodeArrivalVehicles[8,21]: - 999999 x[8,21] - start_service[8]
   + start_service[21] >= -999957.0143215929
 NodeArrivalVehicles[8,22]: - 999999 x[8,22] - start_service[8]
   + start_service[22] >= -999948.4803299305
 NodeArrivalVehicles[8,23]: - 999999 x[8,23] - start_service[8]
   + start_service[23] >= -999948.4803299305
 NodeArrivalVehicles[9,1]: - 999999 x[9,1] + start_service[1]
   - start_service[9] >= -999979.8163579979
 NodeArrivalVehicles[9,2]: - 999999 x[9,2] + start_service[2]
   - start_service[9] >= -999975.0440079125
 NodeArrivalVehicles[9,3]: - 999999 x[9,3] + start_service[3]
   - start_service[9] >= -999956.7738627526
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999908.2748401375
 NodeArrivalVehicles[9,5]: - 999999 x[9,5] + start_service[5]
   - start_service[9] >= -999902.8077036525
 NodeArrivalVehicles[9,6]: - 999999 x[9,6] + start_service[6]
   - start_service[9] >= -999945.0669546484
 NodeArrivalVehicles[9,7]: - 999999 x[9,7] + start_service[7]
   - start_service[9] >= -999950.7231816619
 NodeArrivalVehicles[9,8]: - 999999 x[9,8] + start_service[8]
   - start_service[9] >= -999938.4387890749
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 999999 x[9,10] - start_service[9]
   + start_service[10] >= -999987.6460284748
 NodeArrivalVehicles[9,11]: - 999999 x[9,11] - start_service[9]
   + start_service[11] >= -999907.8119434982
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - start_service[9]
   + start_service[12] >= -999975.0440079125
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - start_service[9]
   + start_service[13] >= -999956.7738627526
 NodeArrivalVehicles[9,14]: - 999999 x[9,14] - start_service[9]
   + start_service[14] >= -999908.2748401375
 NodeArrivalVehicles[9,15]: - 999999 x[9,15] - start_service[9]
   + start_service[15] >= -999902.8077036525
 NodeArrivalVehicles[9,16]: - 999999 x[9,16] - start_service[9]
   + start_service[16] >= -999945.0669546484
 NodeArrivalVehicles[9,17]: - 999999 x[9,17] - start_service[9]
   + start_service[17] >= -999950.7231816619
 NodeArrivalVehicles[9,18]: - 999999 x[9,18] - start_service[9]
   + start_service[18] >= -999938.4387890749
 NodeArrivalVehicles[9,19]: - 999999 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 999999 x[9,20] - start_service[9]
   + start_service[20] >= -999987.6460284748
 NodeArrivalVehicles[9,21]: - 999999 x[9,21] - start_service[9]
   + start_service[21] >= -999907.8119434982
 NodeArrivalVehicles[9,22]: - 999999 x[9,22] - start_service[9]
   + start_service[22] >= -999979.8163579979
 NodeArrivalVehicles[9,23]: - 999999 x[9,23] - start_service[9]
   + start_service[23] >= -999979.8163579979
 NodeArrivalVehicles[10,1]: - 999999 x[10,1] + start_service[1]
   - start_service[10] >= -999968.7963188644
 NodeArrivalVehicles[10,2]: - 999999 x[10,2] + start_service[2]
   - start_service[10] >= -999975.5815039668
 NodeArrivalVehicles[10,3]: - 999999 x[10,3] + start_service[3]
   - start_service[10] >= -999958.4815577437
 NodeArrivalVehicles[10,4]: - 999999 x[10,4] + start_service[4]
   - start_service[10] >= -999903.5978504075
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999902.0796829936
 NodeArrivalVehicles[10,6]: - 999999 x[10,6] + start_service[6]
   - start_service[10] >= -999953.8583916504
 NodeArrivalVehicles[10,7]: - 999999 x[10,7] + start_service[7]
   - start_service[10] >= -999960.3013267835
 NodeArrivalVehicles[10,8]: - 999999 x[10,8] + start_service[8]
   - start_service[10] >= -999933.3155508919
 NodeArrivalVehicles[10,9]: - 999999 x[10,9] + start_service[9]
   - start_service[10] >= -999987.6460284748
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 999999 x[10,11] - start_service[10]
   + start_service[11] >= -999907.1394924825
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - start_service[10]
   + start_service[12] >= -999975.5815039668
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - start_service[10]
   + start_service[13] >= -999958.4815577437
 NodeArrivalVehicles[10,14]: - 999999 x[10,14] - start_service[10]
   + start_service[14] >= -999903.5978504075
 NodeArrivalVehicles[10,15]: - 999999 x[10,15] - start_service[10]
   + start_service[15] >= -999902.0796829936
 NodeArrivalVehicles[10,16]: - 999999 x[10,16] - start_service[10]
   + start_service[16] >= -999953.8583916504
 NodeArrivalVehicles[10,17]: - 999999 x[10,17] - start_service[10]
   + start_service[17] >= -999960.3013267835
 NodeArrivalVehicles[10,18]: - 999999 x[10,18] - start_service[10]
   + start_service[18] >= -999933.3155508919
 NodeArrivalVehicles[10,19]: - 999999 x[10,19] - start_service[10]
   + start_service[19] >= -999987.6460284748
 NodeArrivalVehicles[10,20]: - 999999 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 999999 x[10,21] - start_service[10]
   + start_service[21] >= -999907.1394924825
 NodeArrivalVehicles[10,22]: - 999999 x[10,22] - start_service[10]
   + start_service[22] >= -999968.7963188644
 NodeArrivalVehicles[10,23]: - 999999 x[10,23] - start_service[10]
   + start_service[23] >= -999968.7963188644
 NodeArrivalVehicles[11,1]: - 999999 x[11,1] + start_service[1]
   - start_service[11] >= -999911.607195191
 NodeArrivalVehicles[11,2]: - 999999 x[11,2] + start_service[2]
   - start_service[11] >= -999930.4704496202
 NodeArrivalVehicles[11,3]: - 999999 x[11,3] + start_service[3]
   - start_service[11] >= -999947.0066322301
 NodeArrivalVehicles[11,4]: - 999999 x[11,4] + start_service[4]
   - start_service[11] >= -999965.4410353305
 NodeArrivalVehicles[11,5]: - 999999 x[11,5] + start_service[5]
   - start_service[11] >= -999993.937777768
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999924.6895146155
 NodeArrivalVehicles[11,7]: - 999999 x[11,7] + start_service[7]
   - start_service[11] >= -999918.9888256516
 NodeArrivalVehicles[11,8]: - 999999 x[11,8] + start_service[8]
   - start_service[11] >= -999957.0143215929
 NodeArrivalVehicles[11,9]: - 999999 x[11,9] + start_service[9]
   - start_service[11] >= -999907.8119434982
 NodeArrivalVehicles[11,10]: - 999999 x[11,10] + start_service[10]
   - start_service[11] >= -999907.1394924825
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - start_service[11]
   + start_service[12] >= -999930.4704496202
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - start_service[11]
   + start_service[13] >= -999947.0066322301
 NodeArrivalVehicles[11,14]: - 999999 x[11,14] - start_service[11]
   + start_service[14] >= -999965.4410353305
 NodeArrivalVehicles[11,15]: - 999999 x[11,15] - start_service[11]
   + start_service[15] >= -999993.937777768
 NodeArrivalVehicles[11,16]: - 999999 x[11,16] - start_service[11]
   + start_service[16] >= -999924.6895146155
 NodeArrivalVehicles[11,17]: - 999999 x[11,17] - start_service[11]
   + start_service[17] >= -999918.9888256516
 NodeArrivalVehicles[11,18]: - 999999 x[11,18] - start_service[11]
   + start_service[18] >= -999957.0143215929
 NodeArrivalVehicles[11,19]: - 999999 x[11,19] - start_service[11]
   + start_service[19] >= -999907.8119434982
 NodeArrivalVehicles[11,20]: - 999999 x[11,20] - start_service[11]
   + start_service[20] >= -999907.1394924825
 NodeArrivalVehicles[11,21]: - 999999 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 999999 x[11,22] - start_service[11]
   + start_service[22] >= -999911.607195191
 NodeArrivalVehicles[11,23]: - 999999 x[11,23] - start_service[11]
   + start_service[23] >= -999911.607195191
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999968.5896813161
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] + start_service[2]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 999999 x[12,3] + start_service[3]
   - start_service[12] >= -999980.7297370874
 NodeArrivalVehicles[12,4]: - 999999 x[12,4] + start_service[4]
   - start_service[12] >= -999925.7860261707
 NodeArrivalVehicles[12,5]: - 999999 x[12,5] + start_service[5]
   - start_service[12] >= -999925.4159407663
 NodeArrivalVehicles[12,6]: - 999999 x[12,6] + start_service[6]
   - start_service[12] >= -999960.8785410399
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999963.4348783819
 NodeArrivalVehicles[12,8]: - 999999 x[12,8] + start_service[8]
   - start_service[12] >= -999954.5687754838
 NodeArrivalVehicles[12,9]: - 999999 x[12,9] + start_service[9]
   - start_service[12] >= -999975.0440079125
 NodeArrivalVehicles[12,10]: - 999999 x[12,10] + start_service[10]
   - start_service[12] >= -999975.5815039668
 NodeArrivalVehicles[12,11]: - 999999 x[12,11] + start_service[11]
   - start_service[12] >= -999930.4704496202
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999980.7297370874
 NodeArrivalVehicles[12,14]: - 999999 x[12,14] - start_service[12]
   + start_service[14] >= -999925.7860261707
 NodeArrivalVehicles[12,15]: - 999999 x[12,15] - start_service[12]
   + start_service[15] >= -999925.4159407663
 NodeArrivalVehicles[12,16]: - 999999 x[12,16] - start_service[12]
   + start_service[16] >= -999960.8785410399
 NodeArrivalVehicles[12,17]: - 999999 x[12,17] - start_service[12]
   + start_service[17] >= -999963.4348783819
 NodeArrivalVehicles[12,18]: - 999999 x[12,18] - start_service[12]
   + start_service[18] >= -999954.5687754838
 NodeArrivalVehicles[12,19]: - 999999 x[12,19] - start_service[12]
   + start_service[19] >= -999975.0440079125
 NodeArrivalVehicles[12,20]: - 999999 x[12,20] - start_service[12]
   + start_service[20] >= -999975.5815039668
 NodeArrivalVehicles[12,21]: - 999999 x[12,21] - start_service[12]
   + start_service[21] >= -999930.4704496202
 NodeArrivalVehicles[12,22]: - 999999 x[12,22] - start_service[12]
   + start_service[22] >= -999968.5896813161
 NodeArrivalVehicles[12,23]: - 999999 x[12,23] - start_service[12]
   + start_service[23] >= -999968.5896813161
 NodeArrivalVehicles[13,1]: - 999999 x[13,1] + start_service[1]
   - start_service[13] >= -999952.9831366809
 NodeArrivalVehicles[13,2]: - 999999 x[13,2] + start_service[2]
   - start_service[13] >= -999980.7297370874
 NodeArrivalVehicles[13,3]: - 999999 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 999999 x[13,4] + start_service[4]
   - start_service[13] >= -999936.2884880449
 NodeArrivalVehicles[13,5]: - 999999 x[13,5] + start_service[5]
   - start_service[13] >= -999941.9567081289
 NodeArrivalVehicles[13,6]: - 999999 x[13,6] + start_service[6]
   - start_service[13] >= -999965.5264110569
 NodeArrivalVehicles[13,7]: - 999999 x[13,7] + start_service[7]
   - start_service[13] >= -999963.9361194944
 NodeArrivalVehicles[13,8]: - 999999 x[13,8] + start_service[8]
   - start_service[13] >= -999960.9544713456
 NodeArrivalVehicles[13,9]: - 999999 x[13,9] + start_service[9]
   - start_service[13] >= -999956.7738627526
 NodeArrivalVehicles[13,10]: - 999999 x[13,10] + start_service[10]
   - start_service[13] >= -999958.4815577437
 NodeArrivalVehicles[13,11]: - 999999 x[13,11] + start_service[11]
   - start_service[13] >= -999947.0066322301
 NodeArrivalVehicles[13,12]: - 999999 x[13,12] + start_service[12]
   - start_service[13] >= -999980.7297370874
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 999999 x[13,14] - start_service[13]
   + start_service[14] >= -999936.2884880449
 NodeArrivalVehicles[13,15]: - 999999 x[13,15] - start_service[13]
   + start_service[15] >= -999941.9567081289
 NodeArrivalVehicles[13,16]: - 999999 x[13,16] - start_service[13]
   + start_service[16] >= -999965.5264110569
 NodeArrivalVehicles[13,17]: - 999999 x[13,17] - start_service[13]
   + start_service[17] >= -999963.9361194944
 NodeArrivalVehicles[13,18]: - 999999 x[13,18] - start_service[13]
   + start_service[18] >= -999960.9544713456
 NodeArrivalVehicles[13,19]: - 999999 x[13,19] - start_service[13]
   + start_service[19] >= -999956.7738627526
 NodeArrivalVehicles[13,20]: - 999999 x[13,20] - start_service[13]
   + start_service[20] >= -999958.4815577437
 NodeArrivalVehicles[13,21]: - 999999 x[13,21] - start_service[13]
   + start_service[21] >= -999947.0066322301
 NodeArrivalVehicles[13,22]: - 999999 x[13,22] - start_service[13]
   + start_service[22] >= -999952.9831366809
 NodeArrivalVehicles[13,23]: - 999999 x[13,23] - start_service[13]
   + start_service[23] >= -999952.9831366809
 NodeArrivalVehicles[14,1]: - 999999 x[14,1] + start_service[1]
   - start_service[14] >= -999918.8440504119
 NodeArrivalVehicles[14,2]: - 999999 x[14,2] + start_service[2]
   - start_service[14] >= -999925.7860261707
 NodeArrivalVehicles[14,3]: - 999999 x[14,3] + start_service[3]
   - start_service[14] >= -999936.2884880449
 NodeArrivalVehicles[14,4]: - 999999 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 999999 x[14,5] + start_service[5]
   - start_service[14] >= -999964.8446477182
 NodeArrivalVehicles[14,6]: - 999999 x[14,6] + start_service[6]
   - start_service[14] >= -999905.4897305039
 NodeArrivalVehicles[14,7]: - 999999 x[14,7] + start_service[7]
   - start_service[14] >= -999901.9982294367
 NodeArrivalVehicles[14,8]: - 999999 x[14,8] + start_service[8]
   - start_service[14] >= -999968.8070104932
 NodeArrivalVehicles[14,9]: - 999999 x[14,9] + start_service[9]
   - start_service[14] >= -999908.2748401375
 NodeArrivalVehicles[14,10]: - 999999 x[14,10] + start_service[10]
   - start_service[14] >= -999903.5978504075
 NodeArrivalVehicles[14,11]: - 999999 x[14,11] + start_service[11]
   - start_service[14] >= -999965.4410353305
 NodeArrivalVehicles[14,12]: - 999999 x[14,12] + start_service[12]
   - start_service[14] >= -999925.7860261707
 NodeArrivalVehicles[14,13]: - 999999 x[14,13] + start_service[13]
   - start_service[14] >= -999936.2884880449
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 999999 x[14,15] - start_service[14]
   + start_service[15] >= -999964.8446477182
 NodeArrivalVehicles[14,16]: - 999999 x[14,16] - start_service[14]
   + start_service[16] >= -999905.4897305039
 NodeArrivalVehicles[14,17]: - 999999 x[14,17] - start_service[14]
   + start_service[17] >= -999901.9982294367
 NodeArrivalVehicles[14,18]: - 999999 x[14,18] - start_service[14]
   + start_service[18] >= -999968.8070104932
 NodeArrivalVehicles[14,19]: - 999999 x[14,19] - start_service[14]
   + start_service[19] >= -999908.2748401375
 NodeArrivalVehicles[14,20]: - 999999 x[14,20] - start_service[14]
   + start_service[20] >= -999903.5978504075
 NodeArrivalVehicles[14,21]: - 999999 x[14,21] - start_service[14]
   + start_service[21] >= -999965.4410353305
 NodeArrivalVehicles[14,22]: - 999999 x[14,22] - start_service[14]
   + start_service[22] >= -999918.8440504119
 NodeArrivalVehicles[14,23]: - 999999 x[14,23] - start_service[14]
   + start_service[23] >= -999918.8440504119
 NodeArrivalVehicles[15,1]: - 999999 x[15,1] + start_service[1]
   - start_service[15] >= -999906.8636006397
 NodeArrivalVehicles[15,2]: - 999999 x[15,2] + start_service[2]
   - start_service[15] >= -999925.4159407663
 NodeArrivalVehicles[15,3]: - 999999 x[15,3] + start_service[3]
   - start_service[15] >= -999941.9567081289
 NodeArrivalVehicles[15,4]: - 999999 x[15,4] + start_service[4]
   - start_service[15] >= -999964.8446477182
 NodeArrivalVehicles[15,5]: - 999999 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 999999 x[15,6] + start_service[6]
   - start_service[15] >= -999920.1584135875
 NodeArrivalVehicles[15,7]: - 999999 x[15,7] + start_service[7]
   - start_service[15] >= -999914.3058314386
 NodeArrivalVehicles[15,8]: - 999999 x[15,8] + start_service[8]
   - start_service[15] >= -999953.1311389591
 NodeArrivalVehicles[15,9]: - 999999 x[15,9] + start_service[9]
   - start_service[15] >= -999902.8077036525
 NodeArrivalVehicles[15,10]: - 999999 x[15,10] + start_service[10]
   - start_service[15] >= -999902.0796829936
 NodeArrivalVehicles[15,11]: - 999999 x[15,11] + start_service[11]
   - start_service[15] >= -999993.937777768
 NodeArrivalVehicles[15,12]: - 999999 x[15,12] + start_service[12]
   - start_service[15] >= -999925.4159407663
 NodeArrivalVehicles[15,13]: - 999999 x[15,13] + start_service[13]
   - start_service[15] >= -999941.9567081289
 NodeArrivalVehicles[15,14]: - 999999 x[15,14] + start_service[14]
   - start_service[15] >= -999964.8446477182
 NodeArrivalVehicles[15,15]: - 999999 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 999999 x[15,16] - start_service[15]
   + start_service[16] >= -999920.1584135875
 NodeArrivalVehicles[15,17]: - 999999 x[15,17] - start_service[15]
   + start_service[17] >= -999914.3058314386
 NodeArrivalVehicles[15,18]: - 999999 x[15,18] - start_service[15]
   + start_service[18] >= -999953.1311389591
 NodeArrivalVehicles[15,19]: - 999999 x[15,19] - start_service[15]
   + start_service[19] >= -999902.8077036525
 NodeArrivalVehicles[15,20]: - 999999 x[15,20] - start_service[15]
   + start_service[20] >= -999902.0796829936
 NodeArrivalVehicles[15,21]: - 999999 x[15,21] - start_service[15]
   + start_service[21] >= -999993.937777768
 NodeArrivalVehicles[15,22]: - 999999 x[15,22] - start_service[15]
   + start_service[22] >= -999906.8636006397
 NodeArrivalVehicles[15,23]: - 999999 x[15,23] - start_service[15]
   + start_service[23] >= -999906.8636006397
 NodeArrivalVehicles[16,1]: - 999999 x[16,1] + start_service[1]
   - start_service[16] >= -999931.7152103848
 NodeArrivalVehicles[16,2]: - 999999 x[16,2] + start_service[2]
   - start_service[16] >= -999960.8785410399
 NodeArrivalVehicles[16,3]: - 999999 x[16,3] + start_service[3]
   - start_service[16] >= -999965.5264110569
 NodeArrivalVehicles[16,4]: - 999999 x[16,4] + start_service[4]
   - start_service[16] >= -999905.4897305039
 NodeArrivalVehicles[16,5]: - 999999 x[16,5] + start_service[5]
   - start_service[16] >= -999920.1584135875
 NodeArrivalVehicles[16,6]: - 999999 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 999999 x[16,7] + start_service[7]
   - start_service[16] >= -999990.8380737376
 NodeArrivalVehicles[16,8]: - 999999 x[16,8] + start_service[8]
   - start_service[16] >= -999927.6511923969
 NodeArrivalVehicles[16,9]: - 999999 x[16,9] + start_service[9]
   - start_service[16] >= -999945.0669546484
 NodeArrivalVehicles[16,10]: - 999999 x[16,10] + start_service[10]
   - start_service[16] >= -999953.8583916504
 NodeArrivalVehicles[16,11]: - 999999 x[16,11] + start_service[11]
   - start_service[16] >= -999924.6895146155
 NodeArrivalVehicles[16,12]: - 999999 x[16,12] + start_service[12]
   - start_service[16] >= -999960.8785410399
 NodeArrivalVehicles[16,13]: - 999999 x[16,13] + start_service[13]
   - start_service[16] >= -999965.5264110569
 NodeArrivalVehicles[16,14]: - 999999 x[16,14] + start_service[14]
   - start_service[16] >= -999905.4897305039
 NodeArrivalVehicles[16,15]: - 999999 x[16,15] + start_service[15]
   - start_service[16] >= -999920.1584135875
 NodeArrivalVehicles[16,16]: - 999999 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 999999 x[16,17] - start_service[16]
   + start_service[17] >= -999990.8380737376
 NodeArrivalVehicles[16,18]: - 999999 x[16,18] - start_service[16]
   + start_service[18] >= -999927.6511923969
 NodeArrivalVehicles[16,19]: - 999999 x[16,19] - start_service[16]
   + start_service[19] >= -999945.0669546484
 NodeArrivalVehicles[16,20]: - 999999 x[16,20] - start_service[16]
   + start_service[20] >= -999953.8583916504
 NodeArrivalVehicles[16,21]: - 999999 x[16,21] - start_service[16]
   + start_service[21] >= -999924.6895146155
 NodeArrivalVehicles[16,22]: - 999999 x[16,22] - start_service[16]
   + start_service[22] >= -999931.7152103848
 NodeArrivalVehicles[16,23]: - 999999 x[16,23] - start_service[16]
   + start_service[23] >= -999931.7152103848
 NodeArrivalVehicles[17,1]: - 999999 x[17,1] + start_service[1]
   - start_service[17] >= -999935.9092929828
 NodeArrivalVehicles[17,2]: - 999999 x[17,2] + start_service[2]
   - start_service[17] >= -999963.4348783819
 NodeArrivalVehicles[17,3]: - 999999 x[17,3] + start_service[3]
   - start_service[17] >= -999963.9361194944
 NodeArrivalVehicles[17,4]: - 999999 x[17,4] + start_service[4]
   - start_service[17] >= -999901.9982294367
 NodeArrivalVehicles[17,5]: - 999999 x[17,5] + start_service[5]
   - start_service[17] >= -999914.3058314386
 NodeArrivalVehicles[17,6]: - 999999 x[17,6] + start_service[6]
   - start_service[17] >= -999990.8380737376
 NodeArrivalVehicles[17,7]: - 999999 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 999999 x[17,8] + start_service[8]
   - start_service[17] >= -999925.9744337149
 NodeArrivalVehicles[17,9]: - 999999 x[17,9] + start_service[9]
   - start_service[17] >= -999950.7231816619
 NodeArrivalVehicles[17,10]: - 999999 x[17,10] + start_service[10]
   - start_service[17] >= -999960.3013267835
 NodeArrivalVehicles[17,11]: - 999999 x[17,11] + start_service[11]
   - start_service[17] >= -999918.9888256516
 NodeArrivalVehicles[17,12]: - 999999 x[17,12] + start_service[12]
   - start_service[17] >= -999963.4348783819
 NodeArrivalVehicles[17,13]: - 999999 x[17,13] + start_service[13]
   - start_service[17] >= -999963.9361194944
 NodeArrivalVehicles[17,14]: - 999999 x[17,14] + start_service[14]
   - start_service[17] >= -999901.9982294367
 NodeArrivalVehicles[17,15]: - 999999 x[17,15] + start_service[15]
   - start_service[17] >= -999914.3058314386
 NodeArrivalVehicles[17,16]: - 999999 x[17,16] + start_service[16]
   - start_service[17] >= -999990.8380737376
 NodeArrivalVehicles[17,17]: - 999999 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 999999 x[17,18] - start_service[17]
   + start_service[18] >= -999925.9744337149
 NodeArrivalVehicles[17,19]: - 999999 x[17,19] - start_service[17]
   + start_service[19] >= -999950.7231816619
 NodeArrivalVehicles[17,20]: - 999999 x[17,20] - start_service[17]
   + start_service[20] >= -999960.3013267835
 NodeArrivalVehicles[17,21]: - 999999 x[17,21] - start_service[17]
   + start_service[21] >= -999918.9888256516
 NodeArrivalVehicles[17,22]: - 999999 x[17,22] - start_service[17]
   + start_service[22] >= -999935.9092929828
 NodeArrivalVehicles[17,23]: - 999999 x[17,23] - start_service[17]
   + start_service[23] >= -999935.9092929828
 NodeArrivalVehicles[18,1]: - 999999 x[18,1] + start_service[1]
   - start_service[18] >= -999948.4803299305
 NodeArrivalVehicles[18,2]: - 999999 x[18,2] + start_service[2]
   - start_service[18] >= -999954.5687754838
 NodeArrivalVehicles[18,3]: - 999999 x[18,3] + start_service[3]
   - start_service[18] >= -999960.9544713456
 NodeArrivalVehicles[18,4]: - 999999 x[18,4] + start_service[4]
   - start_service[18] >= -999968.8070104932
 NodeArrivalVehicles[18,5]: - 999999 x[18,5] + start_service[5]
   - start_service[18] >= -999953.1311389591
 NodeArrivalVehicles[18,6]: - 999999 x[18,6] + start_service[6]
   - start_service[18] >= -999927.6511923969
 NodeArrivalVehicles[18,7]: - 999999 x[18,7] + start_service[7]
   - start_service[18] >= -999925.9744337149
 NodeArrivalVehicles[18,8]: - 999999 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 999999 x[18,9] + start_service[9]
   - start_service[18] >= -999938.4387890749
 NodeArrivalVehicles[18,10]: - 999999 x[18,10] + start_service[10]
   - start_service[18] >= -999933.3155508919
 NodeArrivalVehicles[18,11]: - 999999 x[18,11] + start_service[11]
   - start_service[18] >= -999957.0143215929
 NodeArrivalVehicles[18,12]: - 999999 x[18,12] + start_service[12]
   - start_service[18] >= -999954.5687754838
 NodeArrivalVehicles[18,13]: - 999999 x[18,13] + start_service[13]
   - start_service[18] >= -999960.9544713456
 NodeArrivalVehicles[18,14]: - 999999 x[18,14] + start_service[14]
   - start_service[18] >= -999968.8070104932
 NodeArrivalVehicles[18,15]: - 999999 x[18,15] + start_service[15]
   - start_service[18] >= -999953.1311389591
 NodeArrivalVehicles[18,16]: - 999999 x[18,16] + start_service[16]
   - start_service[18] >= -999927.6511923969
 NodeArrivalVehicles[18,17]: - 999999 x[18,17] + start_service[17]
   - start_service[18] >= -999925.9744337149
 NodeArrivalVehicles[18,18]: - 999999 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 999999 x[18,19] - start_service[18]
   + start_service[19] >= -999938.4387890749
 NodeArrivalVehicles[18,20]: - 999999 x[18,20] - start_service[18]
   + start_service[20] >= -999933.3155508919
 NodeArrivalVehicles[18,21]: - 999999 x[18,21] - start_service[18]
   + start_service[21] >= -999957.0143215929
 NodeArrivalVehicles[18,22]: - 999999 x[18,22] - start_service[18]
   + start_service[22] >= -999948.4803299305
 NodeArrivalVehicles[18,23]: - 999999 x[18,23] - start_service[18]
   + start_service[23] >= -999948.4803299305
 NodeArrivalVehicles[19,1]: - 999999 x[19,1] + start_service[1]
   - start_service[19] >= -999979.8163579979
 NodeArrivalVehicles[19,2]: - 999999 x[19,2] + start_service[2]
   - start_service[19] >= -999975.0440079125
 NodeArrivalVehicles[19,3]: - 999999 x[19,3] + start_service[3]
   - start_service[19] >= -999956.7738627526
 NodeArrivalVehicles[19,4]: - 999999 x[19,4] + start_service[4]
   - start_service[19] >= -999908.2748401375
 NodeArrivalVehicles[19,5]: - 999999 x[19,5] + start_service[5]
   - start_service[19] >= -999902.8077036525
 NodeArrivalVehicles[19,6]: - 999999 x[19,6] + start_service[6]
   - start_service[19] >= -999945.0669546484
 NodeArrivalVehicles[19,7]: - 999999 x[19,7] + start_service[7]
   - start_service[19] >= -999950.7231816619
 NodeArrivalVehicles[19,8]: - 999999 x[19,8] + start_service[8]
   - start_service[19] >= -999938.4387890749
 NodeArrivalVehicles[19,9]: - 999999 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 999999 x[19,10] + start_service[10]
   - start_service[19] >= -999987.6460284748
 NodeArrivalVehicles[19,11]: - 999999 x[19,11] + start_service[11]
   - start_service[19] >= -999907.8119434982
 NodeArrivalVehicles[19,12]: - 999999 x[19,12] + start_service[12]
   - start_service[19] >= -999975.0440079125
 NodeArrivalVehicles[19,13]: - 999999 x[19,13] + start_service[13]
   - start_service[19] >= -999956.7738627526
 NodeArrivalVehicles[19,14]: - 999999 x[19,14] + start_service[14]
   - start_service[19] >= -999908.2748401375
 NodeArrivalVehicles[19,15]: - 999999 x[19,15] + start_service[15]
   - start_service[19] >= -999902.8077036525
 NodeArrivalVehicles[19,16]: - 999999 x[19,16] + start_service[16]
   - start_service[19] >= -999945.0669546484
 NodeArrivalVehicles[19,17]: - 999999 x[19,17] + start_service[17]
   - start_service[19] >= -999950.7231816619
 NodeArrivalVehicles[19,18]: - 999999 x[19,18] + start_service[18]
   - start_service[19] >= -999938.4387890749
 NodeArrivalVehicles[19,19]: - 999999 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 999999 x[19,20] - start_service[19]
   + start_service[20] >= -999987.6460284748
 NodeArrivalVehicles[19,21]: - 999999 x[19,21] - start_service[19]
   + start_service[21] >= -999907.8119434982
 NodeArrivalVehicles[19,22]: - 999999 x[19,22] - start_service[19]
   + start_service[22] >= -999979.8163579979
 NodeArrivalVehicles[19,23]: - 999999 x[19,23] - start_service[19]
   + start_service[23] >= -999979.8163579979
 NodeArrivalVehicles[20,1]: - 999999 x[20,1] + start_service[1]
   - start_service[20] >= -999968.7963188644
 NodeArrivalVehicles[20,2]: - 999999 x[20,2] + start_service[2]
   - start_service[20] >= -999975.5815039668
 NodeArrivalVehicles[20,3]: - 999999 x[20,3] + start_service[3]
   - start_service[20] >= -999958.4815577437
 NodeArrivalVehicles[20,4]: - 999999 x[20,4] + start_service[4]
   - start_service[20] >= -999903.5978504075
 NodeArrivalVehicles[20,5]: - 999999 x[20,5] + start_service[5]
   - start_service[20] >= -999902.0796829936
 NodeArrivalVehicles[20,6]: - 999999 x[20,6] + start_service[6]
   - start_service[20] >= -999953.8583916504
 NodeArrivalVehicles[20,7]: - 999999 x[20,7] + start_service[7]
   - start_service[20] >= -999960.3013267835
 NodeArrivalVehicles[20,8]: - 999999 x[20,8] + start_service[8]
   - start_service[20] >= -999933.3155508919
 NodeArrivalVehicles[20,9]: - 999999 x[20,9] + start_service[9]
   - start_service[20] >= -999987.6460284748
 NodeArrivalVehicles[20,10]: - 999999 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 999999 x[20,11] + start_service[11]
   - start_service[20] >= -999907.1394924825
 NodeArrivalVehicles[20,12]: - 999999 x[20,12] + start_service[12]
   - start_service[20] >= -999975.5815039668
 NodeArrivalVehicles[20,13]: - 999999 x[20,13] + start_service[13]
   - start_service[20] >= -999958.4815577437
 NodeArrivalVehicles[20,14]: - 999999 x[20,14] + start_service[14]
   - start_service[20] >= -999903.5978504075
 NodeArrivalVehicles[20,15]: - 999999 x[20,15] + start_service[15]
   - start_service[20] >= -999902.0796829936
 NodeArrivalVehicles[20,16]: - 999999 x[20,16] + start_service[16]
   - start_service[20] >= -999953.8583916504
 NodeArrivalVehicles[20,17]: - 999999 x[20,17] + start_service[17]
   - start_service[20] >= -999960.3013267835
 NodeArrivalVehicles[20,18]: - 999999 x[20,18] + start_service[18]
   - start_service[20] >= -999933.3155508919
 NodeArrivalVehicles[20,19]: - 999999 x[20,19] + start_service[19]
   - start_service[20] >= -999987.6460284748
 NodeArrivalVehicles[20,20]: - 999999 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 999999 x[20,21] - start_service[20]
   + start_service[21] >= -999907.1394924825
 NodeArrivalVehicles[20,22]: - 999999 x[20,22] - start_service[20]
   + start_service[22] >= -999968.7963188644
 NodeArrivalVehicles[20,23]: - 999999 x[20,23] - start_service[20]
   + start_service[23] >= -999968.7963188644
 NodeArrivalVehicles[21,1]: - 999999 x[21,1] + start_service[1]
   - start_service[21] >= -999911.607195191
 NodeArrivalVehicles[21,2]: - 999999 x[21,2] + start_service[2]
   - start_service[21] >= -999930.4704496202
 NodeArrivalVehicles[21,3]: - 999999 x[21,3] + start_service[3]
   - start_service[21] >= -999947.0066322301
 NodeArrivalVehicles[21,4]: - 999999 x[21,4] + start_service[4]
   - start_service[21] >= -999965.4410353305
 NodeArrivalVehicles[21,5]: - 999999 x[21,5] + start_service[5]
   - start_service[21] >= -999993.937777768
 NodeArrivalVehicles[21,6]: - 999999 x[21,6] + start_service[6]
   - start_service[21] >= -999924.6895146155
 NodeArrivalVehicles[21,7]: - 999999 x[21,7] + start_service[7]
   - start_service[21] >= -999918.9888256516
 NodeArrivalVehicles[21,8]: - 999999 x[21,8] + start_service[8]
   - start_service[21] >= -999957.0143215929
 NodeArrivalVehicles[21,9]: - 999999 x[21,9] + start_service[9]
   - start_service[21] >= -999907.8119434982
 NodeArrivalVehicles[21,10]: - 999999 x[21,10] + start_service[10]
   - start_service[21] >= -999907.1394924825
 NodeArrivalVehicles[21,11]: - 999999 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 999999 x[21,12] + start_service[12]
   - start_service[21] >= -999930.4704496202
 NodeArrivalVehicles[21,13]: - 999999 x[21,13] + start_service[13]
   - start_service[21] >= -999947.0066322301
 NodeArrivalVehicles[21,14]: - 999999 x[21,14] + start_service[14]
   - start_service[21] >= -999965.4410353305
 NodeArrivalVehicles[21,15]: - 999999 x[21,15] + start_service[15]
   - start_service[21] >= -999993.937777768
 NodeArrivalVehicles[21,16]: - 999999 x[21,16] + start_service[16]
   - start_service[21] >= -999924.6895146155
 NodeArrivalVehicles[21,17]: - 999999 x[21,17] + start_service[17]
   - start_service[21] >= -999918.9888256516
 NodeArrivalVehicles[21,18]: - 999999 x[21,18] + start_service[18]
   - start_service[21] >= -999957.0143215929
 NodeArrivalVehicles[21,19]: - 999999 x[21,19] + start_service[19]
   - start_service[21] >= -999907.8119434982
 NodeArrivalVehicles[21,20]: - 999999 x[21,20] + start_service[20]
   - start_service[21] >= -999907.1394924825
 NodeArrivalVehicles[21,21]: - 999999 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 999999 x[21,22] - start_service[21]
   + start_service[22] >= -999911.607195191
 NodeArrivalVehicles[21,23]: - 999999 x[21,23] - start_service[21]
   + start_service[23] >= -999911.607195191
 NodeArrivalVehicles[22,1]: - 999999 x[22,1] + start_service[1]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 999999 x[22,2] + start_service[2]
   - start_service[22] >= -999968.5896813161
 NodeArrivalVehicles[22,3]: - 999999 x[22,3] + start_service[3]
   - start_service[22] >= -999952.9831366809
 NodeArrivalVehicles[22,4]: - 999999 x[22,4] + start_service[4]
   - start_service[22] >= -999918.8440504119
 NodeArrivalVehicles[22,5]: - 999999 x[22,5] + start_service[5]
   - start_service[22] >= -999906.8636006397
 NodeArrivalVehicles[22,6]: - 999999 x[22,6] + start_service[6]
   - start_service[22] >= -999931.7152103848
 NodeArrivalVehicles[22,7]: - 999999 x[22,7] + start_service[7]
   - start_service[22] >= -999935.9092929828
 NodeArrivalVehicles[22,8]: - 999999 x[22,8] + start_service[8]
   - start_service[22] >= -999948.4803299305
 NodeArrivalVehicles[22,9]: - 999999 x[22,9] + start_service[9]
   - start_service[22] >= -999979.8163579979
 NodeArrivalVehicles[22,10]: - 999999 x[22,10] + start_service[10]
   - start_service[22] >= -999968.7963188644
 NodeArrivalVehicles[22,11]: - 999999 x[22,11] + start_service[11]
   - start_service[22] >= -999911.607195191
 NodeArrivalVehicles[22,12]: - 999999 x[22,12] + start_service[12]
   - start_service[22] >= -999968.5896813161
 NodeArrivalVehicles[22,13]: - 999999 x[22,13] + start_service[13]
   - start_service[22] >= -999952.9831366809
 NodeArrivalVehicles[22,14]: - 999999 x[22,14] + start_service[14]
   - start_service[22] >= -999918.8440504119
 NodeArrivalVehicles[22,15]: - 999999 x[22,15] + start_service[15]
   - start_service[22] >= -999906.8636006397
 NodeArrivalVehicles[22,16]: - 999999 x[22,16] + start_service[16]
   - start_service[22] >= -999931.7152103848
 NodeArrivalVehicles[22,17]: - 999999 x[22,17] + start_service[17]
   - start_service[22] >= -999935.9092929828
 NodeArrivalVehicles[22,18]: - 999999 x[22,18] + start_service[18]
   - start_service[22] >= -999948.4803299305
 NodeArrivalVehicles[22,19]: - 999999 x[22,19] + start_service[19]
   - start_service[22] >= -999979.8163579979
 NodeArrivalVehicles[22,20]: - 999999 x[22,20] + start_service[20]
   - start_service[22] >= -999968.7963188644
 NodeArrivalVehicles[22,21]: - 999999 x[22,21] + start_service[21]
   - start_service[22] >= -999911.607195191
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
   - start_service[0] + start_service[2] >= -999968.5896813161
 StartServiceRobots[0,3]: - 999999 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999952.9831366809
 StartServiceRobots[0,4]: - 999999 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999918.8440504119
 StartServiceRobots[0,5]: - 999999 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999906.8636006397
 StartServiceRobots[0,6]: - 999999 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999931.7152103848
 StartServiceRobots[0,7]: - 999999 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999935.9092929828
 StartServiceRobots[0,8]: - 999999 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999948.4803299305
 StartServiceRobots[0,9]: - 999999 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999979.8163579979
 StartServiceRobots[0,10]: - 999999 v[0,10] - service_time[0]
   - start_service[0] + start_service[10] >= -999968.7963188644
 StartServiceRobots[0,11]: - 999999 v[0,11] - service_time[0]
   - start_service[0] + start_service[11] >= -999911.607195191
 StartServiceRobots[0,12]: - 999999 v[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999968.5896813161
 StartServiceRobots[0,13]: - 999999 v[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999952.9831366809
 StartServiceRobots[0,14]: - 999999 v[0,14] - service_time[0]
   - start_service[0] + start_service[14] >= -999918.8440504119
 StartServiceRobots[0,15]: - 999999 v[0,15] - service_time[0]
   - start_service[0] + start_service[15] >= -999906.8636006397
 StartServiceRobots[0,16]: - 999999 v[0,16] - service_time[0]
   - start_service[0] + start_service[16] >= -999931.7152103848
 StartServiceRobots[0,17]: - 999999 v[0,17] - service_time[0]
   - start_service[0] + start_service[17] >= -999935.9092929828
 StartServiceRobots[0,18]: - 999999 v[0,18] - service_time[0]
   - start_service[0] + start_service[18] >= -999948.4803299305
 StartServiceRobots[0,19]: - 999999 v[0,19] - service_time[0]
   - start_service[0] + start_service[19] >= -999979.8163579979
 StartServiceRobots[0,20]: - 999999 v[0,20] - service_time[0]
   - start_service[0] + start_service[20] >= -999968.7963188644
 StartServiceRobots[0,21]: - 999999 v[0,21] - service_time[0]
   - start_service[0] + start_service[21] >= -999911.607195191
 StartServiceRobots[0,22]: - 999999 v[0,22] - service_time[0]
   - start_service[0] + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999 v[0,23] - service_time[0]
   - start_service[0] + start_service[23] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 999999 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999968.5896813161
 StartServiceRobots[1,3]: - 999999 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999952.9831366809
 StartServiceRobots[1,4]: - 999999 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999918.8440504119
 StartServiceRobots[1,5]: - 999999 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999906.8636006397
 StartServiceRobots[1,6]: - 999999 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999931.7152103848
 StartServiceRobots[1,7]: - 999999 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999935.9092929828
 StartServiceRobots[1,8]: - 999999 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999948.4803299305
 StartServiceRobots[1,9]: - 999999 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999979.8163579979
 StartServiceRobots[1,10]: - 999999 v[1,10] - service_time[1]
   - start_service[1] + start_service[10] >= -999968.7963188644
 StartServiceRobots[1,11]: - 999999 v[1,11] - service_time[1]
   - start_service[1] + start_service[11] >= -999911.607195191
 StartServiceRobots[1,12]: - 999999 v[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999968.5896813161
 StartServiceRobots[1,13]: - 999999 v[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999952.9831366809
 StartServiceRobots[1,14]: - 999999 v[1,14] - service_time[1]
   - start_service[1] + start_service[14] >= -999918.8440504119
 StartServiceRobots[1,15]: - 999999 v[1,15] - service_time[1]
   - start_service[1] + start_service[15] >= -999906.8636006397
 StartServiceRobots[1,16]: - 999999 v[1,16] - service_time[1]
   - start_service[1] + start_service[16] >= -999931.7152103848
 StartServiceRobots[1,17]: - 999999 v[1,17] - service_time[1]
   - start_service[1] + start_service[17] >= -999935.9092929828
 StartServiceRobots[1,18]: - 999999 v[1,18] - service_time[1]
   - start_service[1] + start_service[18] >= -999948.4803299305
 StartServiceRobots[1,19]: - 999999 v[1,19] - service_time[1]
   - start_service[1] + start_service[19] >= -999979.8163579979
 StartServiceRobots[1,20]: - 999999 v[1,20] - service_time[1]
   - start_service[1] + start_service[20] >= -999968.7963188644
 StartServiceRobots[1,21]: - 999999 v[1,21] - service_time[1]
   - start_service[1] + start_service[21] >= -999911.607195191
 StartServiceRobots[1,22]: - 999999 v[1,22] - service_time[1]
   - start_service[1] + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999 v[1,23] - service_time[1]
   - start_service[1] + start_service[23] >= -999999
 StartServiceRobots[2,1]: - 999999 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999968.5896813161
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 999999 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999980.7297370874
 StartServiceRobots[2,4]: - 999999 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999925.7860261707
 StartServiceRobots[2,5]: - 999999 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999925.4159407663
 StartServiceRobots[2,6]: - 999999 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999960.8785410399
 StartServiceRobots[2,7]: - 999999 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999963.4348783819
 StartServiceRobots[2,8]: - 999999 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999954.5687754838
 StartServiceRobots[2,9]: - 999999 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999975.0440079125
 StartServiceRobots[2,10]: - 999999 v[2,10] - service_time[2]
   - start_service[2] + start_service[10] >= -999975.5815039668
 StartServiceRobots[2,11]: - 999999 v[2,11] - service_time[2]
   - start_service[2] + start_service[11] >= -999930.4704496202
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 999999 v[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999980.7297370874
 StartServiceRobots[2,14]: - 999999 v[2,14] - service_time[2]
   - start_service[2] + start_service[14] >= -999925.7860261707
 StartServiceRobots[2,15]: - 999999 v[2,15] - service_time[2]
   - start_service[2] + start_service[15] >= -999925.4159407663
 StartServiceRobots[2,16]: - 999999 v[2,16] - service_time[2]
   - start_service[2] + start_service[16] >= -999960.8785410399
 StartServiceRobots[2,17]: - 999999 v[2,17] - service_time[2]
   - start_service[2] + start_service[17] >= -999963.4348783819
 StartServiceRobots[2,18]: - 999999 v[2,18] - service_time[2]
   - start_service[2] + start_service[18] >= -999954.5687754838
 StartServiceRobots[2,19]: - 999999 v[2,19] - service_time[2]
   - start_service[2] + start_service[19] >= -999975.0440079125
 StartServiceRobots[2,20]: - 999999 v[2,20] - service_time[2]
   - start_service[2] + start_service[20] >= -999975.5815039668
 StartServiceRobots[2,21]: - 999999 v[2,21] - service_time[2]
   - start_service[2] + start_service[21] >= -999930.4704496202
 StartServiceRobots[2,22]: - 999999 v[2,22] - service_time[2]
   - start_service[2] + start_service[22] >= -999968.5896813161
 StartServiceRobots[2,23]: - 999999 v[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= -999968.5896813161
 StartServiceRobots[3,1]: - 999999 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999952.9831366809
 StartServiceRobots[3,2]: - 999999 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999980.7297370874
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 999999 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999936.2884880449
 StartServiceRobots[3,5]: - 999999 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999941.9567081289
 StartServiceRobots[3,6]: - 999999 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999965.5264110569
 StartServiceRobots[3,7]: - 999999 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999963.9361194944
 StartServiceRobots[3,8]: - 999999 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999960.9544713456
 StartServiceRobots[3,9]: - 999999 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999956.7738627526
 StartServiceRobots[3,10]: - 999999 v[3,10] - service_time[3]
   - start_service[3] + start_service[10] >= -999958.4815577437
 StartServiceRobots[3,11]: - 999999 v[3,11] - service_time[3]
   - start_service[3] + start_service[11] >= -999947.0066322301
 StartServiceRobots[3,12]: - 999999 v[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999980.7297370874
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 999999 v[3,14] - service_time[3]
   - start_service[3] + start_service[14] >= -999936.2884880449
 StartServiceRobots[3,15]: - 999999 v[3,15] - service_time[3]
   - start_service[3] + start_service[15] >= -999941.9567081289
 StartServiceRobots[3,16]: - 999999 v[3,16] - service_time[3]
   - start_service[3] + start_service[16] >= -999965.5264110569
 StartServiceRobots[3,17]: - 999999 v[3,17] - service_time[3]
   - start_service[3] + start_service[17] >= -999963.9361194944
 StartServiceRobots[3,18]: - 999999 v[3,18] - service_time[3]
   - start_service[3] + start_service[18] >= -999960.9544713456
 StartServiceRobots[3,19]: - 999999 v[3,19] - service_time[3]
   - start_service[3] + start_service[19] >= -999956.7738627526
 StartServiceRobots[3,20]: - 999999 v[3,20] - service_time[3]
   - start_service[3] + start_service[20] >= -999958.4815577437
 StartServiceRobots[3,21]: - 999999 v[3,21] - service_time[3]
   - start_service[3] + start_service[21] >= -999947.0066322301
 StartServiceRobots[3,22]: - 999999 v[3,22] - service_time[3]
   - start_service[3] + start_service[22] >= -999952.9831366809
 StartServiceRobots[3,23]: - 999999 v[3,23] - service_time[3]
   - start_service[3] + start_service[23] >= -999952.9831366809
 StartServiceRobots[4,1]: - 999999 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999918.8440504119
 StartServiceRobots[4,2]: - 999999 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999925.7860261707
 StartServiceRobots[4,3]: - 999999 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999936.2884880449
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 999999 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999964.8446477182
 StartServiceRobots[4,6]: - 999999 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999905.4897305039
 StartServiceRobots[4,7]: - 999999 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999901.9982294367
 StartServiceRobots[4,8]: - 999999 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999968.8070104932
 StartServiceRobots[4,9]: - 999999 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999908.2748401375
 StartServiceRobots[4,10]: - 999999 v[4,10] - service_time[4]
   - start_service[4] + start_service[10] >= -999903.5978504075
 StartServiceRobots[4,11]: - 999999 v[4,11] - service_time[4]
   - start_service[4] + start_service[11] >= -999965.4410353305
 StartServiceRobots[4,12]: - 999999 v[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999925.7860261707
 StartServiceRobots[4,13]: - 999999 v[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999936.2884880449
 StartServiceRobots[4,14]: - 999999 v[4,14] - service_time[4]
   - start_service[4] + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 999999 v[4,15] - service_time[4]
   - start_service[4] + start_service[15] >= -999964.8446477182
 StartServiceRobots[4,16]: - 999999 v[4,16] - service_time[4]
   - start_service[4] + start_service[16] >= -999905.4897305039
 StartServiceRobots[4,17]: - 999999 v[4,17] - service_time[4]
   - start_service[4] + start_service[17] >= -999901.9982294367
 StartServiceRobots[4,18]: - 999999 v[4,18] - service_time[4]
   - start_service[4] + start_service[18] >= -999968.8070104932
 StartServiceRobots[4,19]: - 999999 v[4,19] - service_time[4]
   - start_service[4] + start_service[19] >= -999908.2748401375
 StartServiceRobots[4,20]: - 999999 v[4,20] - service_time[4]
   - start_service[4] + start_service[20] >= -999903.5978504075
 StartServiceRobots[4,21]: - 999999 v[4,21] - service_time[4]
   - start_service[4] + start_service[21] >= -999965.4410353305
 StartServiceRobots[4,22]: - 999999 v[4,22] - service_time[4]
   - start_service[4] + start_service[22] >= -999918.8440504119
 StartServiceRobots[4,23]: - 999999 v[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= -999918.8440504119
 StartServiceRobots[5,1]: - 999999 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999906.8636006397
 StartServiceRobots[5,2]: - 999999 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999925.4159407663
 StartServiceRobots[5,3]: - 999999 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999941.9567081289
 StartServiceRobots[5,4]: - 999999 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999964.8446477182
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 999999 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999920.1584135875
 StartServiceRobots[5,7]: - 999999 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999914.3058314386
 StartServiceRobots[5,8]: - 999999 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999953.1311389591
 StartServiceRobots[5,9]: - 999999 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999902.8077036525
 StartServiceRobots[5,10]: - 999999 v[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999902.0796829936
 StartServiceRobots[5,11]: - 999999 v[5,11] - service_time[5]
   - start_service[5] + start_service[11] >= -999993.937777768
 StartServiceRobots[5,12]: - 999999 v[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999925.4159407663
 StartServiceRobots[5,13]: - 999999 v[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999941.9567081289
 StartServiceRobots[5,14]: - 999999 v[5,14] - service_time[5]
   - start_service[5] + start_service[14] >= -999964.8446477182
 StartServiceRobots[5,15]: - 999999 v[5,15] - service_time[5]
   - start_service[5] + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 999999 v[5,16] - service_time[5]
   - start_service[5] + start_service[16] >= -999920.1584135875
 StartServiceRobots[5,17]: - 999999 v[5,17] - service_time[5]
   - start_service[5] + start_service[17] >= -999914.3058314386
 StartServiceRobots[5,18]: - 999999 v[5,18] - service_time[5]
   - start_service[5] + start_service[18] >= -999953.1311389591
 StartServiceRobots[5,19]: - 999999 v[5,19] - service_time[5]
   - start_service[5] + start_service[19] >= -999902.8077036525
 StartServiceRobots[5,20]: - 999999 v[5,20] - service_time[5]
   - start_service[5] + start_service[20] >= -999902.0796829936
 StartServiceRobots[5,21]: - 999999 v[5,21] - service_time[5]
   - start_service[5] + start_service[21] >= -999993.937777768
 StartServiceRobots[5,22]: - 999999 v[5,22] - service_time[5]
   - start_service[5] + start_service[22] >= -999906.8636006397
 StartServiceRobots[5,23]: - 999999 v[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= -999906.8636006397
 StartServiceRobots[6,1]: - 999999 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999931.7152103848
 StartServiceRobots[6,2]: - 999999 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999960.8785410399
 StartServiceRobots[6,3]: - 999999 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999965.5264110569
 StartServiceRobots[6,4]: - 999999 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999905.4897305039
 StartServiceRobots[6,5]: - 999999 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999920.1584135875
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 999999 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999990.8380737376
 StartServiceRobots[6,8]: - 999999 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999927.6511923969
 StartServiceRobots[6,9]: - 999999 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999945.0669546484
 StartServiceRobots[6,10]: - 999999 v[6,10] - service_time[6]
   - start_service[6] + start_service[10] >= -999953.8583916504
 StartServiceRobots[6,11]: - 999999 v[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999924.6895146155
 StartServiceRobots[6,12]: - 999999 v[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999960.8785410399
 StartServiceRobots[6,13]: - 999999 v[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999965.5264110569
 StartServiceRobots[6,14]: - 999999 v[6,14] - service_time[6]
   - start_service[6] + start_service[14] >= -999905.4897305039
 StartServiceRobots[6,15]: - 999999 v[6,15] - service_time[6]
   - start_service[6] + start_service[15] >= -999920.1584135875
 StartServiceRobots[6,16]: - 999999 v[6,16] - service_time[6]
   - start_service[6] + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 999999 v[6,17] - service_time[6]
   - start_service[6] + start_service[17] >= -999990.8380737376
 StartServiceRobots[6,18]: - 999999 v[6,18] - service_time[6]
   - start_service[6] + start_service[18] >= -999927.6511923969
 StartServiceRobots[6,19]: - 999999 v[6,19] - service_time[6]
   - start_service[6] + start_service[19] >= -999945.0669546484
 StartServiceRobots[6,20]: - 999999 v[6,20] - service_time[6]
   - start_service[6] + start_service[20] >= -999953.8583916504
 StartServiceRobots[6,21]: - 999999 v[6,21] - service_time[6]
   - start_service[6] + start_service[21] >= -999924.6895146155
 StartServiceRobots[6,22]: - 999999 v[6,22] - service_time[6]
   - start_service[6] + start_service[22] >= -999931.7152103848
 StartServiceRobots[6,23]: - 999999 v[6,23] - service_time[6]
   - start_service[6] + start_service[23] >= -999931.7152103848
 StartServiceRobots[7,1]: - 999999 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999935.9092929828
 StartServiceRobots[7,2]: - 999999 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999963.4348783819
 StartServiceRobots[7,3]: - 999999 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999963.9361194944
 StartServiceRobots[7,4]: - 999999 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999901.9982294367
 StartServiceRobots[7,5]: - 999999 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999914.3058314386
 StartServiceRobots[7,6]: - 999999 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999990.8380737376
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 999999 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999925.9744337149
 StartServiceRobots[7,9]: - 999999 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999950.7231816619
 StartServiceRobots[7,10]: - 999999 v[7,10] - service_time[7]
   - start_service[7] + start_service[10] >= -999960.3013267835
 StartServiceRobots[7,11]: - 999999 v[7,11] - service_time[7]
   - start_service[7] + start_service[11] >= -999918.9888256516
 StartServiceRobots[7,12]: - 999999 v[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999963.4348783819
 StartServiceRobots[7,13]: - 999999 v[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999963.9361194944
 StartServiceRobots[7,14]: - 999999 v[7,14] - service_time[7]
   - start_service[7] + start_service[14] >= -999901.9982294367
 StartServiceRobots[7,15]: - 999999 v[7,15] - service_time[7]
   - start_service[7] + start_service[15] >= -999914.3058314386
 StartServiceRobots[7,16]: - 999999 v[7,16] - service_time[7]
   - start_service[7] + start_service[16] >= -999990.8380737376
 StartServiceRobots[7,17]: - 999999 v[7,17] - service_time[7]
   - start_service[7] + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 999999 v[7,18] - service_time[7]
   - start_service[7] + start_service[18] >= -999925.9744337149
 StartServiceRobots[7,19]: - 999999 v[7,19] - service_time[7]
   - start_service[7] + start_service[19] >= -999950.7231816619
 StartServiceRobots[7,20]: - 999999 v[7,20] - service_time[7]
   - start_service[7] + start_service[20] >= -999960.3013267835
 StartServiceRobots[7,21]: - 999999 v[7,21] - service_time[7]
   - start_service[7] + start_service[21] >= -999918.9888256516
 StartServiceRobots[7,22]: - 999999 v[7,22] - service_time[7]
   - start_service[7] + start_service[22] >= -999935.9092929828
 StartServiceRobots[7,23]: - 999999 v[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= -999935.9092929828
 StartServiceRobots[8,1]: - 999999 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999948.4803299305
 StartServiceRobots[8,2]: - 999999 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999954.5687754838
 StartServiceRobots[8,3]: - 999999 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999960.9544713456
 StartServiceRobots[8,4]: - 999999 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999968.8070104932
 StartServiceRobots[8,5]: - 999999 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999953.1311389591
 StartServiceRobots[8,6]: - 999999 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999927.6511923969
 StartServiceRobots[8,7]: - 999999 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999925.9744337149
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 999999 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999938.4387890749
 StartServiceRobots[8,10]: - 999999 v[8,10] - service_time[8]
   - start_service[8] + start_service[10] >= -999933.3155508919
 StartServiceRobots[8,11]: - 999999 v[8,11] - service_time[8]
   - start_service[8] + start_service[11] >= -999957.0143215929
 StartServiceRobots[8,12]: - 999999 v[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999954.5687754838
 StartServiceRobots[8,13]: - 999999 v[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999960.9544713456
 StartServiceRobots[8,14]: - 999999 v[8,14] - service_time[8]
   - start_service[8] + start_service[14] >= -999968.8070104932
 StartServiceRobots[8,15]: - 999999 v[8,15] - service_time[8]
   - start_service[8] + start_service[15] >= -999953.1311389591
 StartServiceRobots[8,16]: - 999999 v[8,16] - service_time[8]
   - start_service[8] + start_service[16] >= -999927.6511923969
 StartServiceRobots[8,17]: - 999999 v[8,17] - service_time[8]
   - start_service[8] + start_service[17] >= -999925.9744337149
 StartServiceRobots[8,18]: - 999999 v[8,18] - service_time[8]
   - start_service[8] + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 999999 v[8,19] - service_time[8]
   - start_service[8] + start_service[19] >= -999938.4387890749
 StartServiceRobots[8,20]: - 999999 v[8,20] - service_time[8]
   - start_service[8] + start_service[20] >= -999933.3155508919
 StartServiceRobots[8,21]: - 999999 v[8,21] - service_time[8]
   - start_service[8] + start_service[21] >= -999957.0143215929
 StartServiceRobots[8,22]: - 999999 v[8,22] - service_time[8]
   - start_service[8] + start_service[22] >= -999948.4803299305
 StartServiceRobots[8,23]: - 999999 v[8,23] - service_time[8]
   - start_service[8] + start_service[23] >= -999948.4803299305
 StartServiceRobots[9,1]: - 999999 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999979.8163579979
 StartServiceRobots[9,2]: - 999999 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999975.0440079125
 StartServiceRobots[9,3]: - 999999 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999956.7738627526
 StartServiceRobots[9,4]: - 999999 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999908.2748401375
 StartServiceRobots[9,5]: - 999999 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999902.8077036525
 StartServiceRobots[9,6]: - 999999 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999945.0669546484
 StartServiceRobots[9,7]: - 999999 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999950.7231816619
 StartServiceRobots[9,8]: - 999999 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999938.4387890749
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 999999 v[9,10] - service_time[9]
   - start_service[9] + start_service[10] >= -999987.6460284748
 StartServiceRobots[9,11]: - 999999 v[9,11] - service_time[9]
   - start_service[9] + start_service[11] >= -999907.8119434982
 StartServiceRobots[9,12]: - 999999 v[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999975.0440079125
 StartServiceRobots[9,13]: - 999999 v[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999956.7738627526
 StartServiceRobots[9,14]: - 999999 v[9,14] - service_time[9]
   - start_service[9] + start_service[14] >= -999908.2748401375
 StartServiceRobots[9,15]: - 999999 v[9,15] - service_time[9]
   - start_service[9] + start_service[15] >= -999902.8077036525
 StartServiceRobots[9,16]: - 999999 v[9,16] - service_time[9]
   - start_service[9] + start_service[16] >= -999945.0669546484
 StartServiceRobots[9,17]: - 999999 v[9,17] - service_time[9]
   - start_service[9] + start_service[17] >= -999950.7231816619
 StartServiceRobots[9,18]: - 999999 v[9,18] - service_time[9]
   - start_service[9] + start_service[18] >= -999938.4387890749
 StartServiceRobots[9,19]: - 999999 v[9,19] - service_time[9]
   - start_service[9] + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 999999 v[9,20] - service_time[9]
   - start_service[9] + start_service[20] >= -999987.6460284748
 StartServiceRobots[9,21]: - 999999 v[9,21] - service_time[9]
   - start_service[9] + start_service[21] >= -999907.8119434982
 StartServiceRobots[9,22]: - 999999 v[9,22] - service_time[9]
   - start_service[9] + start_service[22] >= -999979.8163579979
 StartServiceRobots[9,23]: - 999999 v[9,23] - service_time[9]
   - start_service[9] + start_service[23] >= -999979.8163579979
 StartServiceRobots[10,1]: - 999999 v[10,1] - service_time[10]
   + start_service[1] - start_service[10] >= -999968.7963188644
 StartServiceRobots[10,2]: - 999999 v[10,2] - service_time[10]
   + start_service[2] - start_service[10] >= -999975.5815039668
 StartServiceRobots[10,3]: - 999999 v[10,3] - service_time[10]
   + start_service[3] - start_service[10] >= -999958.4815577437
 StartServiceRobots[10,4]: - 999999 v[10,4] - service_time[10]
   + start_service[4] - start_service[10] >= -999903.5978504075
 StartServiceRobots[10,5]: - 999999 v[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999902.0796829936
 StartServiceRobots[10,6]: - 999999 v[10,6] - service_time[10]
   + start_service[6] - start_service[10] >= -999953.8583916504
 StartServiceRobots[10,7]: - 999999 v[10,7] - service_time[10]
   + start_service[7] - start_service[10] >= -999960.3013267835
 StartServiceRobots[10,8]: - 999999 v[10,8] - service_time[10]
   + start_service[8] - start_service[10] >= -999933.3155508919
 StartServiceRobots[10,9]: - 999999 v[10,9] - service_time[10]
   + start_service[9] - start_service[10] >= -999987.6460284748
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 999999 v[10,11] - service_time[10]
   - start_service[10] + start_service[11] >= -999907.1394924825
 StartServiceRobots[10,12]: - 999999 v[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999975.5815039668
 StartServiceRobots[10,13]: - 999999 v[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999958.4815577437
 StartServiceRobots[10,14]: - 999999 v[10,14] - service_time[10]
   - start_service[10] + start_service[14] >= -999903.5978504075
 StartServiceRobots[10,15]: - 999999 v[10,15] - service_time[10]
   - start_service[10] + start_service[15] >= -999902.0796829936
 StartServiceRobots[10,16]: - 999999 v[10,16] - service_time[10]
   - start_service[10] + start_service[16] >= -999953.8583916504
 StartServiceRobots[10,17]: - 999999 v[10,17] - service_time[10]
   - start_service[10] + start_service[17] >= -999960.3013267835
 StartServiceRobots[10,18]: - 999999 v[10,18] - service_time[10]
   - start_service[10] + start_service[18] >= -999933.3155508919
 StartServiceRobots[10,19]: - 999999 v[10,19] - service_time[10]
   - start_service[10] + start_service[19] >= -999987.6460284748
 StartServiceRobots[10,20]: - 999999 v[10,20] - service_time[10]
   - start_service[10] + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 999999 v[10,21] - service_time[10]
   - start_service[10] + start_service[21] >= -999907.1394924825
 StartServiceRobots[10,22]: - 999999 v[10,22] - service_time[10]
   - start_service[10] + start_service[22] >= -999968.7963188644
 StartServiceRobots[10,23]: - 999999 v[10,23] - service_time[10]
   - start_service[10] + start_service[23] >= -999968.7963188644
 StartServiceRobots[11,1]: - 999999 v[11,1] - service_time[11]
   + start_service[1] - start_service[11] >= -999911.607195191
 StartServiceRobots[11,2]: - 999999 v[11,2] - service_time[11]
   + start_service[2] - start_service[11] >= -999930.4704496202
 StartServiceRobots[11,3]: - 999999 v[11,3] - service_time[11]
   + start_service[3] - start_service[11] >= -999947.0066322301
 StartServiceRobots[11,4]: - 999999 v[11,4] - service_time[11]
   + start_service[4] - start_service[11] >= -999965.4410353305
 StartServiceRobots[11,5]: - 999999 v[11,5] - service_time[11]
   + start_service[5] - start_service[11] >= -999993.937777768
 StartServiceRobots[11,6]: - 999999 v[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999924.6895146155
 StartServiceRobots[11,7]: - 999999 v[11,7] - service_time[11]
   + start_service[7] - start_service[11] >= -999918.9888256516
 StartServiceRobots[11,8]: - 999999 v[11,8] - service_time[11]
   + start_service[8] - start_service[11] >= -999957.0143215929
 StartServiceRobots[11,9]: - 999999 v[11,9] - service_time[11]
   + start_service[9] - start_service[11] >= -999907.8119434982
 StartServiceRobots[11,10]: - 999999 v[11,10] - service_time[11]
   + start_service[10] - start_service[11] >= -999907.1394924825
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 999999 v[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999930.4704496202
 StartServiceRobots[11,13]: - 999999 v[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999947.0066322301
 StartServiceRobots[11,14]: - 999999 v[11,14] - service_time[11]
   - start_service[11] + start_service[14] >= -999965.4410353305
 StartServiceRobots[11,15]: - 999999 v[11,15] - service_time[11]
   - start_service[11] + start_service[15] >= -999993.937777768
 StartServiceRobots[11,16]: - 999999 v[11,16] - service_time[11]
   - start_service[11] + start_service[16] >= -999924.6895146155
 StartServiceRobots[11,17]: - 999999 v[11,17] - service_time[11]
   - start_service[11] + start_service[17] >= -999918.9888256516
 StartServiceRobots[11,18]: - 999999 v[11,18] - service_time[11]
   - start_service[11] + start_service[18] >= -999957.0143215929
 StartServiceRobots[11,19]: - 999999 v[11,19] - service_time[11]
   - start_service[11] + start_service[19] >= -999907.8119434982
 StartServiceRobots[11,20]: - 999999 v[11,20] - service_time[11]
   - start_service[11] + start_service[20] >= -999907.1394924825
 StartServiceRobots[11,21]: - 999999 v[11,21] - service_time[11]
   - start_service[11] + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 999999 v[11,22] - service_time[11]
   - start_service[11] + start_service[22] >= -999911.607195191
 StartServiceRobots[11,23]: - 999999 v[11,23] - service_time[11]
   - start_service[11] + start_service[23] >= -999911.607195191
 StartServiceRobots[12,1]: - 999999 v[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999968.5896813161
 StartServiceRobots[12,2]: - 999999 v[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 999999 v[12,3] - service_time[12]
   + start_service[3] - start_service[12] >= -999980.7297370874
 StartServiceRobots[12,4]: - 999999 v[12,4] - service_time[12]
   + start_service[4] - start_service[12] >= -999925.7860261707
 StartServiceRobots[12,5]: - 999999 v[12,5] - service_time[12]
   + start_service[5] - start_service[12] >= -999925.4159407663
 StartServiceRobots[12,6]: - 999999 v[12,6] - service_time[12]
   + start_service[6] - start_service[12] >= -999960.8785410399
 StartServiceRobots[12,7]: - 999999 v[12,7] - service_time[12]
   + start_service[7] - start_service[12] >= -999963.4348783819
 StartServiceRobots[12,8]: - 999999 v[12,8] - service_time[12]
   + start_service[8] - start_service[12] >= -999954.5687754838
 StartServiceRobots[12,9]: - 999999 v[12,9] - service_time[12]
   + start_service[9] - start_service[12] >= -999975.0440079125
 StartServiceRobots[12,10]: - 999999 v[12,10] - service_time[12]
   + start_service[10] - start_service[12] >= -999975.5815039668
 StartServiceRobots[12,11]: - 999999 v[12,11] - service_time[12]
   + start_service[11] - start_service[12] >= -999930.4704496202
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 999999 v[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999980.7297370874
 StartServiceRobots[12,14]: - 999999 v[12,14] - service_time[12]
   - start_service[12] + start_service[14] >= -999925.7860261707
 StartServiceRobots[12,15]: - 999999 v[12,15] - service_time[12]
   - start_service[12] + start_service[15] >= -999925.4159407663
 StartServiceRobots[12,16]: - 999999 v[12,16] - service_time[12]
   - start_service[12] + start_service[16] >= -999960.8785410399
 StartServiceRobots[12,17]: - 999999 v[12,17] - service_time[12]
   - start_service[12] + start_service[17] >= -999963.4348783819
 StartServiceRobots[12,18]: - 999999 v[12,18] - service_time[12]
   - start_service[12] + start_service[18] >= -999954.5687754838
 StartServiceRobots[12,19]: - 999999 v[12,19] - service_time[12]
   - start_service[12] + start_service[19] >= -999975.0440079125
 StartServiceRobots[12,20]: - 999999 v[12,20] - service_time[12]
   - start_service[12] + start_service[20] >= -999975.5815039668
 StartServiceRobots[12,21]: - 999999 v[12,21] - service_time[12]
   - start_service[12] + start_service[21] >= -999930.4704496202
 StartServiceRobots[12,22]: - 999999 v[12,22] - service_time[12]
   - start_service[12] + start_service[22] >= -999968.5896813161
 StartServiceRobots[12,23]: - 999999 v[12,23] - service_time[12]
   - start_service[12] + start_service[23] >= -999968.5896813161
 StartServiceRobots[13,1]: - 999999 v[13,1] - service_time[13]
   + start_service[1] - start_service[13] >= -999952.9831366809
 StartServiceRobots[13,2]: - 999999 v[13,2] - service_time[13]
   + start_service[2] - start_service[13] >= -999980.7297370874
 StartServiceRobots[13,3]: - 999999 v[13,3] - service_time[13]
   + start_service[3] - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 999999 v[13,4] - service_time[13]
   + start_service[4] - start_service[13] >= -999936.2884880449
 StartServiceRobots[13,5]: - 999999 v[13,5] - service_time[13]
   + start_service[5] - start_service[13] >= -999941.9567081289
 StartServiceRobots[13,6]: - 999999 v[13,6] - service_time[13]
   + start_service[6] - start_service[13] >= -999965.5264110569
 StartServiceRobots[13,7]: - 999999 v[13,7] - service_time[13]
   + start_service[7] - start_service[13] >= -999963.9361194944
 StartServiceRobots[13,8]: - 999999 v[13,8] - service_time[13]
   + start_service[8] - start_service[13] >= -999960.9544713456
 StartServiceRobots[13,9]: - 999999 v[13,9] - service_time[13]
   + start_service[9] - start_service[13] >= -999956.7738627526
 StartServiceRobots[13,10]: - 999999 v[13,10] - service_time[13]
   + start_service[10] - start_service[13] >= -999958.4815577437
 StartServiceRobots[13,11]: - 999999 v[13,11] - service_time[13]
   + start_service[11] - start_service[13] >= -999947.0066322301
 StartServiceRobots[13,12]: - 999999 v[13,12] - service_time[13]
   + start_service[12] - start_service[13] >= -999980.7297370874
 StartServiceRobots[13,13]: - 999999 v[13,13] - service_time[13]
   >= -999999
 StartServiceRobots[13,14]: - 999999 v[13,14] - service_time[13]
   - start_service[13] + start_service[14] >= -999936.2884880449
 StartServiceRobots[13,15]: - 999999 v[13,15] - service_time[13]
   - start_service[13] + start_service[15] >= -999941.9567081289
 StartServiceRobots[13,16]: - 999999 v[13,16] - service_time[13]
   - start_service[13] + start_service[16] >= -999965.5264110569
 StartServiceRobots[13,17]: - 999999 v[13,17] - service_time[13]
   - start_service[13] + start_service[17] >= -999963.9361194944
 StartServiceRobots[13,18]: - 999999 v[13,18] - service_time[13]
   - start_service[13] + start_service[18] >= -999960.9544713456
 StartServiceRobots[13,19]: - 999999 v[13,19] - service_time[13]
   - start_service[13] + start_service[19] >= -999956.7738627526
 StartServiceRobots[13,20]: - 999999 v[13,20] - service_time[13]
   - start_service[13] + start_service[20] >= -999958.4815577437
 StartServiceRobots[13,21]: - 999999 v[13,21] - service_time[13]
   - start_service[13] + start_service[21] >= -999947.0066322301
 StartServiceRobots[13,22]: - 999999 v[13,22] - service_time[13]
   - start_service[13] + start_service[22] >= -999952.9831366809
 StartServiceRobots[13,23]: - 999999 v[13,23] - service_time[13]
   - start_service[13] + start_service[23] >= -999952.9831366809
 StartServiceRobots[14,1]: - 999999 v[14,1] - service_time[14]
   + start_service[1] - start_service[14] >= -999918.8440504119
 StartServiceRobots[14,2]: - 999999 v[14,2] - service_time[14]
   + start_service[2] - start_service[14] >= -999925.7860261707
 StartServiceRobots[14,3]: - 999999 v[14,3] - service_time[14]
   + start_service[3] - start_service[14] >= -999936.2884880449
 StartServiceRobots[14,4]: - 999999 v[14,4] - service_time[14]
   + start_service[4] - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 999999 v[14,5] - service_time[14]
   + start_service[5] - start_service[14] >= -999964.8446477182
 StartServiceRobots[14,6]: - 999999 v[14,6] - service_time[14]
   + start_service[6] - start_service[14] >= -999905.4897305039
 StartServiceRobots[14,7]: - 999999 v[14,7] - service_time[14]
   + start_service[7] - start_service[14] >= -999901.9982294367
 StartServiceRobots[14,8]: - 999999 v[14,8] - service_time[14]
   + start_service[8] - start_service[14] >= -999968.8070104932
 StartServiceRobots[14,9]: - 999999 v[14,9] - service_time[14]
   + start_service[9] - start_service[14] >= -999908.2748401375
 StartServiceRobots[14,10]: - 999999 v[14,10] - service_time[14]
   + start_service[10] - start_service[14] >= -999903.5978504075
 StartServiceRobots[14,11]: - 999999 v[14,11] - service_time[14]
   + start_service[11] - start_service[14] >= -999965.4410353305
 StartServiceRobots[14,12]: - 999999 v[14,12] - service_time[14]
   + start_service[12] - start_service[14] >= -999925.7860261707
 StartServiceRobots[14,13]: - 999999 v[14,13] - service_time[14]
   + start_service[13] - start_service[14] >= -999936.2884880449
 StartServiceRobots[14,14]: - 999999 v[14,14] - service_time[14]
   >= -999999
 StartServiceRobots[14,15]: - 999999 v[14,15] - service_time[14]
   - start_service[14] + start_service[15] >= -999964.8446477182
 StartServiceRobots[14,16]: - 999999 v[14,16] - service_time[14]
   - start_service[14] + start_service[16] >= -999905.4897305039
 StartServiceRobots[14,17]: - 999999 v[14,17] - service_time[14]
   - start_service[14] + start_service[17] >= -999901.9982294367
 StartServiceRobots[14,18]: - 999999 v[14,18] - service_time[14]
   - start_service[14] + start_service[18] >= -999968.8070104932
 StartServiceRobots[14,19]: - 999999 v[14,19] - service_time[14]
   - start_service[14] + start_service[19] >= -999908.2748401375
 StartServiceRobots[14,20]: - 999999 v[14,20] - service_time[14]
   - start_service[14] + start_service[20] >= -999903.5978504075
 StartServiceRobots[14,21]: - 999999 v[14,21] - service_time[14]
   - start_service[14] + start_service[21] >= -999965.4410353305
 StartServiceRobots[14,22]: - 999999 v[14,22] - service_time[14]
   - start_service[14] + start_service[22] >= -999918.8440504119
 StartServiceRobots[14,23]: - 999999 v[14,23] - service_time[14]
   - start_service[14] + start_service[23] >= -999918.8440504119
 StartServiceRobots[15,1]: - 999999 v[15,1] - service_time[15]
   + start_service[1] - start_service[15] >= -999906.8636006397
 StartServiceRobots[15,2]: - 999999 v[15,2] - service_time[15]
   + start_service[2] - start_service[15] >= -999925.4159407663
 StartServiceRobots[15,3]: - 999999 v[15,3] - service_time[15]
   + start_service[3] - start_service[15] >= -999941.9567081289
 StartServiceRobots[15,4]: - 999999 v[15,4] - service_time[15]
   + start_service[4] - start_service[15] >= -999964.8446477182
 StartServiceRobots[15,5]: - 999999 v[15,5] - service_time[15]
   + start_service[5] - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 999999 v[15,6] - service_time[15]
   + start_service[6] - start_service[15] >= -999920.1584135875
 StartServiceRobots[15,7]: - 999999 v[15,7] - service_time[15]
   + start_service[7] - start_service[15] >= -999914.3058314386
 StartServiceRobots[15,8]: - 999999 v[15,8] - service_time[15]
   + start_service[8] - start_service[15] >= -999953.1311389591
 StartServiceRobots[15,9]: - 999999 v[15,9] - service_time[15]
   + start_service[9] - start_service[15] >= -999902.8077036525
 StartServiceRobots[15,10]: - 999999 v[15,10] - service_time[15]
   + start_service[10] - start_service[15] >= -999902.0796829936
 StartServiceRobots[15,11]: - 999999 v[15,11] - service_time[15]
   + start_service[11] - start_service[15] >= -999993.937777768
 StartServiceRobots[15,12]: - 999999 v[15,12] - service_time[15]
   + start_service[12] - start_service[15] >= -999925.4159407663
 StartServiceRobots[15,13]: - 999999 v[15,13] - service_time[15]
   + start_service[13] - start_service[15] >= -999941.9567081289
 StartServiceRobots[15,14]: - 999999 v[15,14] - service_time[15]
   + start_service[14] - start_service[15] >= -999964.8446477182
 StartServiceRobots[15,15]: - 999999 v[15,15] - service_time[15]
   >= -999999
 StartServiceRobots[15,16]: - 999999 v[15,16] - service_time[15]
   - start_service[15] + start_service[16] >= -999920.1584135875
 StartServiceRobots[15,17]: - 999999 v[15,17] - service_time[15]
   - start_service[15] + start_service[17] >= -999914.3058314386
 StartServiceRobots[15,18]: - 999999 v[15,18] - service_time[15]
   - start_service[15] + start_service[18] >= -999953.1311389591
 StartServiceRobots[15,19]: - 999999 v[15,19] - service_time[15]
   - start_service[15] + start_service[19] >= -999902.8077036525
 StartServiceRobots[15,20]: - 999999 v[15,20] - service_time[15]
   - start_service[15] + start_service[20] >= -999902.0796829936
 StartServiceRobots[15,21]: - 999999 v[15,21] - service_time[15]
   - start_service[15] + start_service[21] >= -999993.937777768
 StartServiceRobots[15,22]: - 999999 v[15,22] - service_time[15]
   - start_service[15] + start_service[22] >= -999906.8636006397
 StartServiceRobots[15,23]: - 999999 v[15,23] - service_time[15]
   - start_service[15] + start_service[23] >= -999906.8636006397
 StartServiceRobots[16,1]: - 999999 v[16,1] - service_time[16]
   + start_service[1] - start_service[16] >= -999931.7152103848
 StartServiceRobots[16,2]: - 999999 v[16,2] - service_time[16]
   + start_service[2] - start_service[16] >= -999960.8785410399
 StartServiceRobots[16,3]: - 999999 v[16,3] - service_time[16]
   + start_service[3] - start_service[16] >= -999965.5264110569
 StartServiceRobots[16,4]: - 999999 v[16,4] - service_time[16]
   + start_service[4] - start_service[16] >= -999905.4897305039
 StartServiceRobots[16,5]: - 999999 v[16,5] - service_time[16]
   + start_service[5] - start_service[16] >= -999920.1584135875
 StartServiceRobots[16,6]: - 999999 v[16,6] - service_time[16]
   + start_service[6] - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 999999 v[16,7] - service_time[16]
   + start_service[7] - start_service[16] >= -999990.8380737376
 StartServiceRobots[16,8]: - 999999 v[16,8] - service_time[16]
   + start_service[8] - start_service[16] >= -999927.6511923969
 StartServiceRobots[16,9]: - 999999 v[16,9] - service_time[16]
   + start_service[9] - start_service[16] >= -999945.0669546484
 StartServiceRobots[16,10]: - 999999 v[16,10] - service_time[16]
   + start_service[10] - start_service[16] >= -999953.8583916504
 StartServiceRobots[16,11]: - 999999 v[16,11] - service_time[16]
   + start_service[11] - start_service[16] >= -999924.6895146155
 StartServiceRobots[16,12]: - 999999 v[16,12] - service_time[16]
   + start_service[12] - start_service[16] >= -999960.8785410399
 StartServiceRobots[16,13]: - 999999 v[16,13] - service_time[16]
   + start_service[13] - start_service[16] >= -999965.5264110569
 StartServiceRobots[16,14]: - 999999 v[16,14] - service_time[16]
   + start_service[14] - start_service[16] >= -999905.4897305039
 StartServiceRobots[16,15]: - 999999 v[16,15] - service_time[16]
   + start_service[15] - start_service[16] >= -999920.1584135875
 StartServiceRobots[16,16]: - 999999 v[16,16] - service_time[16]
   >= -999999
 StartServiceRobots[16,17]: - 999999 v[16,17] - service_time[16]
   - start_service[16] + start_service[17] >= -999990.8380737376
 StartServiceRobots[16,18]: - 999999 v[16,18] - service_time[16]
   - start_service[16] + start_service[18] >= -999927.6511923969
 StartServiceRobots[16,19]: - 999999 v[16,19] - service_time[16]
   - start_service[16] + start_service[19] >= -999945.0669546484
 StartServiceRobots[16,20]: - 999999 v[16,20] - service_time[16]
   - start_service[16] + start_service[20] >= -999953.8583916504
 StartServiceRobots[16,21]: - 999999 v[16,21] - service_time[16]
   - start_service[16] + start_service[21] >= -999924.6895146155
 StartServiceRobots[16,22]: - 999999 v[16,22] - service_time[16]
   - start_service[16] + start_service[22] >= -999931.7152103848
 StartServiceRobots[16,23]: - 999999 v[16,23] - service_time[16]
   - start_service[16] + start_service[23] >= -999931.7152103848
 StartServiceRobots[17,1]: - 999999 v[17,1] - service_time[17]
   + start_service[1] - start_service[17] >= -999935.9092929828
 StartServiceRobots[17,2]: - 999999 v[17,2] - service_time[17]
   + start_service[2] - start_service[17] >= -999963.4348783819
 StartServiceRobots[17,3]: - 999999 v[17,3] - service_time[17]
   + start_service[3] - start_service[17] >= -999963.9361194944
 StartServiceRobots[17,4]: - 999999 v[17,4] - service_time[17]
   + start_service[4] - start_service[17] >= -999901.9982294367
 StartServiceRobots[17,5]: - 999999 v[17,5] - service_time[17]
   + start_service[5] - start_service[17] >= -999914.3058314386
 StartServiceRobots[17,6]: - 999999 v[17,6] - service_time[17]
   + start_service[6] - start_service[17] >= -999990.8380737376
 StartServiceRobots[17,7]: - 999999 v[17,7] - service_time[17]
   + start_service[7] - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 999999 v[17,8] - service_time[17]
   + start_service[8] - start_service[17] >= -999925.9744337149
 StartServiceRobots[17,9]: - 999999 v[17,9] - service_time[17]
   + start_service[9] - start_service[17] >= -999950.7231816619
 StartServiceRobots[17,10]: - 999999 v[17,10] - service_time[17]
   + start_service[10] - start_service[17] >= -999960.3013267835
 StartServiceRobots[17,11]: - 999999 v[17,11] - service_time[17]
   + start_service[11] - start_service[17] >= -999918.9888256516
 StartServiceRobots[17,12]: - 999999 v[17,12] - service_time[17]
   + start_service[12] - start_service[17] >= -999963.4348783819
 StartServiceRobots[17,13]: - 999999 v[17,13] - service_time[17]
   + start_service[13] - start_service[17] >= -999963.9361194944
 StartServiceRobots[17,14]: - 999999 v[17,14] - service_time[17]
   + start_service[14] - start_service[17] >= -999901.9982294367
 StartServiceRobots[17,15]: - 999999 v[17,15] - service_time[17]
   + start_service[15] - start_service[17] >= -999914.3058314386
 StartServiceRobots[17,16]: - 999999 v[17,16] - service_time[17]
   + start_service[16] - start_service[17] >= -999990.8380737376
 StartServiceRobots[17,17]: - 999999 v[17,17] - service_time[17]
   >= -999999
 StartServiceRobots[17,18]: - 999999 v[17,18] - service_time[17]
   - start_service[17] + start_service[18] >= -999925.9744337149
 StartServiceRobots[17,19]: - 999999 v[17,19] - service_time[17]
   - start_service[17] + start_service[19] >= -999950.7231816619
 StartServiceRobots[17,20]: - 999999 v[17,20] - service_time[17]
   - start_service[17] + start_service[20] >= -999960.3013267835
 StartServiceRobots[17,21]: - 999999 v[17,21] - service_time[17]
   - start_service[17] + start_service[21] >= -999918.9888256516
 StartServiceRobots[17,22]: - 999999 v[17,22] - service_time[17]
   - start_service[17] + start_service[22] >= -999935.9092929828
 StartServiceRobots[17,23]: - 999999 v[17,23] - service_time[17]
   - start_service[17] + start_service[23] >= -999935.9092929828
 StartServiceRobots[18,1]: - 999999 v[18,1] - service_time[18]
   + start_service[1] - start_service[18] >= -999948.4803299305
 StartServiceRobots[18,2]: - 999999 v[18,2] - service_time[18]
   + start_service[2] - start_service[18] >= -999954.5687754838
 StartServiceRobots[18,3]: - 999999 v[18,3] - service_time[18]
   + start_service[3] - start_service[18] >= -999960.9544713456
 StartServiceRobots[18,4]: - 999999 v[18,4] - service_time[18]
   + start_service[4] - start_service[18] >= -999968.8070104932
 StartServiceRobots[18,5]: - 999999 v[18,5] - service_time[18]
   + start_service[5] - start_service[18] >= -999953.1311389591
 StartServiceRobots[18,6]: - 999999 v[18,6] - service_time[18]
   + start_service[6] - start_service[18] >= -999927.6511923969
 StartServiceRobots[18,7]: - 999999 v[18,7] - service_time[18]
   + start_service[7] - start_service[18] >= -999925.9744337149
 StartServiceRobots[18,8]: - 999999 v[18,8] - service_time[18]
   + start_service[8] - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 999999 v[18,9] - service_time[18]
   + start_service[9] - start_service[18] >= -999938.4387890749
 StartServiceRobots[18,10]: - 999999 v[18,10] - service_time[18]
   + start_service[10] - start_service[18] >= -999933.3155508919
 StartServiceRobots[18,11]: - 999999 v[18,11] - service_time[18]
   + start_service[11] - start_service[18] >= -999957.0143215929
 StartServiceRobots[18,12]: - 999999 v[18,12] - service_time[18]
   + start_service[12] - start_service[18] >= -999954.5687754838
 StartServiceRobots[18,13]: - 999999 v[18,13] - service_time[18]
   + start_service[13] - start_service[18] >= -999960.9544713456
 StartServiceRobots[18,14]: - 999999 v[18,14] - service_time[18]
   + start_service[14] - start_service[18] >= -999968.8070104932
 StartServiceRobots[18,15]: - 999999 v[18,15] - service_time[18]
   + start_service[15] - start_service[18] >= -999953.1311389591
 StartServiceRobots[18,16]: - 999999 v[18,16] - service_time[18]
   + start_service[16] - start_service[18] >= -999927.6511923969
 StartServiceRobots[18,17]: - 999999 v[18,17] - service_time[18]
   + start_service[17] - start_service[18] >= -999925.9744337149
 StartServiceRobots[18,18]: - 999999 v[18,18] - service_time[18]
   >= -999999
 StartServiceRobots[18,19]: - 999999 v[18,19] - service_time[18]
   - start_service[18] + start_service[19] >= -999938.4387890749
 StartServiceRobots[18,20]: - 999999 v[18,20] - service_time[18]
   - start_service[18] + start_service[20] >= -999933.3155508919
 StartServiceRobots[18,21]: - 999999 v[18,21] - service_time[18]
   - start_service[18] + start_service[21] >= -999957.0143215929
 StartServiceRobots[18,22]: - 999999 v[18,22] - service_time[18]
   - start_service[18] + start_service[22] >= -999948.4803299305
 StartServiceRobots[18,23]: - 999999 v[18,23] - service_time[18]
   - start_service[18] + start_service[23] >= -999948.4803299305
 StartServiceRobots[19,1]: - 999999 v[19,1] - service_time[19]
   + start_service[1] - start_service[19] >= -999979.8163579979
 StartServiceRobots[19,2]: - 999999 v[19,2] - service_time[19]
   + start_service[2] - start_service[19] >= -999975.0440079125
 StartServiceRobots[19,3]: - 999999 v[19,3] - service_time[19]
   + start_service[3] - start_service[19] >= -999956.7738627526
 StartServiceRobots[19,4]: - 999999 v[19,4] - service_time[19]
   + start_service[4] - start_service[19] >= -999908.2748401375
 StartServiceRobots[19,5]: - 999999 v[19,5] - service_time[19]
   + start_service[5] - start_service[19] >= -999902.8077036525
 StartServiceRobots[19,6]: - 999999 v[19,6] - service_time[19]
   + start_service[6] - start_service[19] >= -999945.0669546484
 StartServiceRobots[19,7]: - 999999 v[19,7] - service_time[19]
   + start_service[7] - start_service[19] >= -999950.7231816619
 StartServiceRobots[19,8]: - 999999 v[19,8] - service_time[19]
   + start_service[8] - start_service[19] >= -999938.4387890749
 StartServiceRobots[19,9]: - 999999 v[19,9] - service_time[19]
   + start_service[9] - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 999999 v[19,10] - service_time[19]
   + start_service[10] - start_service[19] >= -999987.6460284748
 StartServiceRobots[19,11]: - 999999 v[19,11] - service_time[19]
   + start_service[11] - start_service[19] >= -999907.8119434982
 StartServiceRobots[19,12]: - 999999 v[19,12] - service_time[19]
   + start_service[12] - start_service[19] >= -999975.0440079125
 StartServiceRobots[19,13]: - 999999 v[19,13] - service_time[19]
   + start_service[13] - start_service[19] >= -999956.7738627526
 StartServiceRobots[19,14]: - 999999 v[19,14] - service_time[19]
   + start_service[14] - start_service[19] >= -999908.2748401375
 StartServiceRobots[19,15]: - 999999 v[19,15] - service_time[19]
   + start_service[15] - start_service[19] >= -999902.8077036525
 StartServiceRobots[19,16]: - 999999 v[19,16] - service_time[19]
   + start_service[16] - start_service[19] >= -999945.0669546484
 StartServiceRobots[19,17]: - 999999 v[19,17] - service_time[19]
   + start_service[17] - start_service[19] >= -999950.7231816619
 StartServiceRobots[19,18]: - 999999 v[19,18] - service_time[19]
   + start_service[18] - start_service[19] >= -999938.4387890749
 StartServiceRobots[19,19]: - 999999 v[19,19] - service_time[19]
   >= -999999
 StartServiceRobots[19,20]: - 999999 v[19,20] - service_time[19]
   - start_service[19] + start_service[20] >= -999987.6460284748
 StartServiceRobots[19,21]: - 999999 v[19,21] - service_time[19]
   - start_service[19] + start_service[21] >= -999907.8119434982
 StartServiceRobots[19,22]: - 999999 v[19,22] - service_time[19]
   - start_service[19] + start_service[22] >= -999979.8163579979
 StartServiceRobots[19,23]: - 999999 v[19,23] - service_time[19]
   - start_service[19] + start_service[23] >= -999979.8163579979
 StartServiceRobots[20,1]: - 999999 v[20,1] - service_time[20]
   + start_service[1] - start_service[20] >= -999968.7963188644
 StartServiceRobots[20,2]: - 999999 v[20,2] - service_time[20]
   + start_service[2] - start_service[20] >= -999975.5815039668
 StartServiceRobots[20,3]: - 999999 v[20,3] - service_time[20]
   + start_service[3] - start_service[20] >= -999958.4815577437
 StartServiceRobots[20,4]: - 999999 v[20,4] - service_time[20]
   + start_service[4] - start_service[20] >= -999903.5978504075
 StartServiceRobots[20,5]: - 999999 v[20,5] - service_time[20]
   + start_service[5] - start_service[20] >= -999902.0796829936
 StartServiceRobots[20,6]: - 999999 v[20,6] - service_time[20]
   + start_service[6] - start_service[20] >= -999953.8583916504
 StartServiceRobots[20,7]: - 999999 v[20,7] - service_time[20]
   + start_service[7] - start_service[20] >= -999960.3013267835
 StartServiceRobots[20,8]: - 999999 v[20,8] - service_time[20]
   + start_service[8] - start_service[20] >= -999933.3155508919
 StartServiceRobots[20,9]: - 999999 v[20,9] - service_time[20]
   + start_service[9] - start_service[20] >= -999987.6460284748
 StartServiceRobots[20,10]: - 999999 v[20,10] - service_time[20]
   + start_service[10] - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 999999 v[20,11] - service_time[20]
   + start_service[11] - start_service[20] >= -999907.1394924825
 StartServiceRobots[20,12]: - 999999 v[20,12] - service_time[20]
   + start_service[12] - start_service[20] >= -999975.5815039668
 StartServiceRobots[20,13]: - 999999 v[20,13] - service_time[20]
   + start_service[13] - start_service[20] >= -999958.4815577437
 StartServiceRobots[20,14]: - 999999 v[20,14] - service_time[20]
   + start_service[14] - start_service[20] >= -999903.5978504075
 StartServiceRobots[20,15]: - 999999 v[20,15] - service_time[20]
   + start_service[15] - start_service[20] >= -999902.0796829936
 StartServiceRobots[20,16]: - 999999 v[20,16] - service_time[20]
   + start_service[16] - start_service[20] >= -999953.8583916504
 StartServiceRobots[20,17]: - 999999 v[20,17] - service_time[20]
   + start_service[17] - start_service[20] >= -999960.3013267835
 StartServiceRobots[20,18]: - 999999 v[20,18] - service_time[20]
   + start_service[18] - start_service[20] >= -999933.3155508919
 StartServiceRobots[20,19]: - 999999 v[20,19] - service_time[20]
   + start_service[19] - start_service[20] >= -999987.6460284748
 StartServiceRobots[20,20]: - 999999 v[20,20] - service_time[20]
   >= -999999
 StartServiceRobots[20,21]: - 999999 v[20,21] - service_time[20]
   - start_service[20] + start_service[21] >= -999907.1394924825
 StartServiceRobots[20,22]: - 999999 v[20,22] - service_time[20]
   - start_service[20] + start_service[22] >= -999968.7963188644
 StartServiceRobots[20,23]: - 999999 v[20,23] - service_time[20]
   - start_service[20] + start_service[23] >= -999968.7963188644
 StartServiceRobots[21,1]: - 999999 v[21,1] - service_time[21]
   + start_service[1] - start_service[21] >= -999911.607195191
 StartServiceRobots[21,2]: - 999999 v[21,2] - service_time[21]
   + start_service[2] - start_service[21] >= -999930.4704496202
 StartServiceRobots[21,3]: - 999999 v[21,3] - service_time[21]
   + start_service[3] - start_service[21] >= -999947.0066322301
 StartServiceRobots[21,4]: - 999999 v[21,4] - service_time[21]
   + start_service[4] - start_service[21] >= -999965.4410353305
 StartServiceRobots[21,5]: - 999999 v[21,5] - service_time[21]
   + start_service[5] - start_service[21] >= -999993.937777768
 StartServiceRobots[21,6]: - 999999 v[21,6] - service_time[21]
   + start_service[6] - start_service[21] >= -999924.6895146155
 StartServiceRobots[21,7]: - 999999 v[21,7] - service_time[21]
   + start_service[7] - start_service[21] >= -999918.9888256516
 StartServiceRobots[21,8]: - 999999 v[21,8] - service_time[21]
   + start_service[8] - start_service[21] >= -999957.0143215929
 StartServiceRobots[21,9]: - 999999 v[21,9] - service_time[21]
   + start_service[9] - start_service[21] >= -999907.8119434982
 StartServiceRobots[21,10]: - 999999 v[21,10] - service_time[21]
   + start_service[10] - start_service[21] >= -999907.1394924825
 StartServiceRobots[21,11]: - 999999 v[21,11] - service_time[21]
   + start_service[11] - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 999999 v[21,12] - service_time[21]
   + start_service[12] - start_service[21] >= -999930.4704496202
 StartServiceRobots[21,13]: - 999999 v[21,13] - service_time[21]
   + start_service[13] - start_service[21] >= -999947.0066322301
 StartServiceRobots[21,14]: - 999999 v[21,14] - service_time[21]
   + start_service[14] - start_service[21] >= -999965.4410353305
 StartServiceRobots[21,15]: - 999999 v[21,15] - service_time[21]
   + start_service[15] - start_service[21] >= -999993.937777768
 StartServiceRobots[21,16]: - 999999 v[21,16] - service_time[21]
   + start_service[16] - start_service[21] >= -999924.6895146155
 StartServiceRobots[21,17]: - 999999 v[21,17] - service_time[21]
   + start_service[17] - start_service[21] >= -999918.9888256516
 StartServiceRobots[21,18]: - 999999 v[21,18] - service_time[21]
   + start_service[18] - start_service[21] >= -999957.0143215929
 StartServiceRobots[21,19]: - 999999 v[21,19] - service_time[21]
   + start_service[19] - start_service[21] >= -999907.8119434982
 StartServiceRobots[21,20]: - 999999 v[21,20] - service_time[21]
   + start_service[20] - start_service[21] >= -999907.1394924825
 StartServiceRobots[21,21]: - 999999 v[21,21] - service_time[21]
   >= -999999
 StartServiceRobots[21,22]: - 999999 v[21,22] - service_time[21]
   - start_service[21] + start_service[22] >= -999911.607195191
 StartServiceRobots[21,23]: - 999999 v[21,23] - service_time[21]
   - start_service[21] + start_service[23] >= -999911.607195191
 StartServiceRobots[22,1]: - 999999 v[22,1] - service_time[22]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 999999 v[22,2] - service_time[22]
   + start_service[2] - start_service[22] >= -999968.5896813161
 StartServiceRobots[22,3]: - 999999 v[22,3] - service_time[22]
   + start_service[3] - start_service[22] >= -999952.9831366809
 StartServiceRobots[22,4]: - 999999 v[22,4] - service_time[22]
   + start_service[4] - start_service[22] >= -999918.8440504119
 StartServiceRobots[22,5]: - 999999 v[22,5] - service_time[22]
   + start_service[5] - start_service[22] >= -999906.8636006397
 StartServiceRobots[22,6]: - 999999 v[22,6] - service_time[22]
   + start_service[6] - start_service[22] >= -999931.7152103848
 StartServiceRobots[22,7]: - 999999 v[22,7] - service_time[22]
   + start_service[7] - start_service[22] >= -999935.9092929828
 StartServiceRobots[22,8]: - 999999 v[22,8] - service_time[22]
   + start_service[8] - start_service[22] >= -999948.4803299305
 StartServiceRobots[22,9]: - 999999 v[22,9] - service_time[22]
   + start_service[9] - start_service[22] >= -999979.8163579979
 StartServiceRobots[22,10]: - 999999 v[22,10] - service_time[22]
   + start_service[10] - start_service[22] >= -999968.7963188644
 StartServiceRobots[22,11]: - 999999 v[22,11] - service_time[22]
   + start_service[11] - start_service[22] >= -999911.607195191
 StartServiceRobots[22,12]: - 999999 v[22,12] - service_time[22]
   + start_service[12] - start_service[22] >= -999968.5896813161
 StartServiceRobots[22,13]: - 999999 v[22,13] - service_time[22]
   + start_service[13] - start_service[22] >= -999952.9831366809
 StartServiceRobots[22,14]: - 999999 v[22,14] - service_time[22]
   + start_service[14] - start_service[22] >= -999918.8440504119
 StartServiceRobots[22,15]: - 999999 v[22,15] - service_time[22]
   + start_service[15] - start_service[22] >= -999906.8636006397
 StartServiceRobots[22,16]: - 999999 v[22,16] - service_time[22]
   + start_service[16] - start_service[22] >= -999931.7152103848
 StartServiceRobots[22,17]: - 999999 v[22,17] - service_time[22]
   + start_service[17] - start_service[22] >= -999935.9092929828
 StartServiceRobots[22,18]: - 999999 v[22,18] - service_time[22]
   + start_service[18] - start_service[22] >= -999948.4803299305
 StartServiceRobots[22,19]: - 999999 v[22,19] - service_time[22]
   + start_service[19] - start_service[22] >= -999979.8163579979
 StartServiceRobots[22,20]: - 999999 v[22,20] - service_time[22]
   + start_service[20] - start_service[22] >= -999968.7963188644
 StartServiceRobots[22,21]: - 999999 v[22,21] - service_time[22]
   + start_service[21] - start_service[22] >= -999911.607195191
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
 SettingServiceTime[2]: - 280 y[2,1] - 140 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 410 y[3,1] - 205 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 440 y[4,1] - 220 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 390 y[5,1] - 195 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 210 y[6,1] - 105 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 230 y[7,1] - 115 y[7,2] + service_time[7] = 0
 SettingServiceTime[8]: - 380 y[8,1] - 190 y[8,2] + service_time[8] = 0
 SettingServiceTime[9]: - 390 y[9,1] - 195 y[9,2] + service_time[9] = 0
 SettingServiceTime[10]: - 420 y[10,1] - 210 y[10,2] + service_time[10]
   = 0
 SettingServiceTime[11]: - 410 y[11,1] - 205 y[11,2] + service_time[11]
   = 0
 AverageTimeRobots: - 4.344331240562464 w[0,2] - 6.573837617007667 w[0,3]
   - 11.4508499411422 w[0,4] - 13.16234276575076 w[0,5]
   - 9.612112802177897 w[0,6] - 9.012958145309993 w[0,7]
   - 7.217095724205025 w[0,8] - 2.740520286016073 w[0,9]
   - 4.314811590792756 w[0,10] - 12.48468640129708 w[0,11]
   - 4.344331240562464 w[0,12] - 6.573837617007667 w[0,13]
   - 11.4508499411422 w[0,14] - 13.16234276575076 w[0,15]
   - 9.612112802177897 w[0,16] - 9.012958145309993 w[0,17]
   - 7.217095724205025 w[0,18] - 2.740520286016073 w[0,19]
   - 4.314811590792756 w[0,20] - 12.48468640129708 w[0,21]
   - 4.344331240562464 w[1,2] - 6.573837617007667 w[1,3]
   - 11.4508499411422 w[1,4] - 13.16234276575076 w[1,5]
   - 9.612112802177897 w[1,6] - 9.012958145309993 w[1,7]
   - 7.217095724205025 w[1,8] - 2.740520286016073 w[1,9]
   - 4.314811590792756 w[1,10] - 12.48468640129708 w[1,11]
   - 4.344331240562464 w[1,12] - 6.573837617007667 w[1,13]
   - 11.4508499411422 w[1,14] - 13.16234276575076 w[1,15]
   - 9.612112802177897 w[1,16] - 9.012958145309993 w[1,17]
   - 7.217095724205025 w[1,18] - 2.740520286016073 w[1,19]
   - 4.314811590792756 w[1,20] - 12.48468640129708 w[1,21]
   - 4.344331240562464 w[2,1] - 2.61003755895037 w[2,3]
   - 10.45913911847039 w[2,4] - 10.51200846194448 w[2,5]
   - 5.445922708582885 w[2,6] - 5.080731659730024 w[2,7]
   - 6.347317788032758 w[2,8] - 3.422284583926837 w[2,9]
   - 3.345499433306067 w[2,10] - 9.789935768555161 w[2,11]
   - 2.61003755895037 w[2,13] - 10.45913911847039 w[2,14]
   - 10.51200846194448 w[2,15] - 5.445922708582885 w[2,16]
   - 5.080731659730024 w[2,17] - 6.347317788032758 w[2,18]
   - 3.422284583926837 w[2,19] - 3.345499433306067 w[2,20]
   - 9.789935768555161 w[2,21] - 4.344331240562464 w[2,22]
   - 4.344331240562464 w[2,23] - 6.573837617007667 w[3,1]
   - 2.61003755895037 w[3,2] - 8.95878742215544 w[3,4]
   - 8.149041695863 w[3,5] - 4.781941277589094 w[3,6]
   - 5.009125786524788 w[3,7] - 5.435075522044343 w[3,8]
   - 6.032305321062844 w[3,9] - 5.788348893748577 w[3,10]
   - 7.427623967124651 w[3,11] - 2.61003755895037 w[3,12]
   - 8.95878742215544 w[3,14] - 8.149041695863 w[3,15]
   - 4.781941277589094 w[3,16] - 5.009125786524788 w[3,17]
   - 5.435075522044343 w[3,18] - 6.032305321062844 w[3,19]
   - 5.788348893748577 w[3,20] - 7.427623967124651 w[3,21]
   - 6.573837617007667 w[3,22] - 6.573837617007667 w[3,23]
   - 11.4508499411422 w[4,1] - 10.45913911847039 w[4,2]
   - 8.95878742215544 w[4,3] - 4.879336040253222 w[4,5]
   - 13.35860992801736 w[4,6] - 13.85739579475303 w[4,7]
   - 4.313284215254392 w[4,8] - 12.96073712322885 w[4,9]
   - 13.62887851321657 w[4,10] - 4.794137809929045 w[4,11]
   - 10.45913911847039 w[4,12] - 8.95878742215544 w[4,13]
   - 4.879336040253222 w[4,15] - 13.35860992801736 w[4,16]
   - 13.85739579475303 w[4,17] - 4.313284215254392 w[4,18]
   - 12.96073712322885 w[4,19] - 13.62887851321657 w[4,20]
   - 4.794137809929045 w[4,21] - 11.4508499411422 w[4,22]
   - 11.4508499411422 w[4,23] - 13.16234276575076 w[5,1]
   - 10.51200846194448 w[5,2] - 8.149041695863 w[5,3]
   - 4.879336040253222 w[5,4] - 11.26308377322209 w[5,6]
   - 12.09916693733823 w[5,7] - 6.552694434412026 w[5,8]
   - 13.74175662106976 w[5,9] - 13.8457595723512 w[5,10]
   - 0.7231746045625118 w[5,11] - 10.51200846194448 w[5,12]
   - 8.149041695863 w[5,13] - 4.879336040253222 w[5,14]
   - 11.26308377322209 w[5,16] - 12.09916693733823 w[5,17]
   - 6.552694434412026 w[5,18] - 13.74175662106976 w[5,19]
   - 13.8457595723512 w[5,20] - 0.7231746045625118 w[5,21]
   - 13.16234276575076 w[5,22] - 13.16234276575076 w[5,23]
   - 9.612112802177897 w[6,1] - 5.445922708582885 w[6,2]
   - 4.781941277589094 w[6,3] - 13.35860992801736 w[6,4]
   - 11.26308377322209 w[6,5] - 1.165989466062996 w[6,7]
   - 10.19268680045306 w[6,8] - 7.704720764512761 w[6,9]
   - 6.448801192803242 w[6,10] - 10.61578362637059 w[6,11]
   - 5.445922708582885 w[6,12] - 4.781941277589094 w[6,13]
   - 13.35860992801736 w[6,14] - 11.26308377322209 w[6,15]
   - 1.165989466062996 w[6,17] - 10.19268680045306 w[6,18]
   - 7.704720764512761 w[6,19] - 6.448801192803242 w[6,20]
   - 10.61578362637059 w[6,21] - 9.612112802177897 w[6,22]
   - 9.612112802177897 w[6,23] - 9.012958145309993 w[7,1]
   - 5.080731659730024 w[7,2] - 5.009125786524788 w[7,3]
   - 13.85739579475303 w[7,4] - 12.09916693733823 w[7,5]
   - 1.165989466062996 w[7,6] - 10.43222375502204 w[7,8]
   - 6.896688334011931 w[7,9] - 5.528381888071401 w[7,10]
   - 11.43016776405479 w[7,11] - 5.080731659730024 w[7,12]
   - 5.009125786524788 w[7,13] - 13.85739579475303 w[7,14]
   - 12.09916693733823 w[7,15] - 1.165989466062996 w[7,16]
   - 10.43222375502204 w[7,18] - 6.896688334011931 w[7,19]
   - 5.528381888071401 w[7,20] - 11.43016776405479 w[7,21]
   - 9.012958145309993 w[7,22] - 9.012958145309993 w[7,23]
   - 7.217095724205025 w[8,1] - 6.347317788032758 w[8,2]
   - 5.435075522044343 w[8,3] - 4.313284215254392 w[8,4]
   - 6.552694434412026 w[8,5] - 10.19268680045306 w[8,6]
   - 10.43222375502204 w[8,7] - 8.651601560727855 w[8,9]
   - 9.38349272973112 w[8,10] - 5.997954058161801 w[8,11]
   - 6.347317788032758 w[8,12] - 5.435075522044343 w[8,13]
   - 4.313284215254392 w[8,14] - 6.552694434412026 w[8,15]
   - 10.19268680045306 w[8,16] - 10.43222375502204 w[8,17]
   - 8.651601560727855 w[8,19] - 9.38349272973112 w[8,20]
   - 5.997954058161801 w[8,21] - 7.217095724205025 w[8,22]
   - 7.217095724205025 w[8,23] - 2.740520286016073 w[9,1]
   - 3.422284583926837 w[9,2] - 6.032305321062844 w[9,3]
   - 12.96073712322885 w[9,4] - 13.74175662106976 w[9,5]
   - 7.704720764512761 w[9,6] - 6.896688334011931 w[9,7]
   - 8.651601560727855 w[9,8] - 1.62199593217746 w[9,10]
   - 13.02686521454084 w[9,11] - 3.422284583926837 w[9,12]
   - 6.032305321062844 w[9,13] - 12.96073712322885 w[9,14]
   - 13.74175662106976 w[9,15] - 7.704720764512761 w[9,16]
   - 6.896688334011931 w[9,17] - 8.651601560727855 w[9,18]
   - 1.62199593217746 w[9,20] - 13.02686521454084 w[9,21]
   - 2.740520286016073 w[9,22] - 2.740520286016073 w[9,23]
   - 4.314811590792756 w[10,1] - 3.345499433306067 w[10,2]
   - 5.788348893748577 w[10,3] - 13.62887851321657 w[10,4]
   - 13.8457595723512 w[10,5] - 6.448801192803242 w[10,6]
   - 5.528381888071401 w[10,7] - 9.38349272973112 w[10,8]
   - 1.62199593217746 w[10,9] - 13.12292964535524 w[10,11]
   - 3.345499433306067 w[10,12] - 5.788348893748577 w[10,13]
   - 13.62887851321657 w[10,14] - 13.8457595723512 w[10,15]
   - 6.448801192803242 w[10,16] - 5.528381888071401 w[10,17]
   - 9.38349272973112 w[10,18] - 1.62199593217746 w[10,19]
   - 13.12292964535524 w[10,21] - 4.314811590792756 w[10,22]
   - 4.314811590792756 w[10,23] - 12.48468640129708 w[11,1]
   - 9.789935768555161 w[11,2] - 7.427623967124651 w[11,3]
   - 4.794137809929045 w[11,4] - 0.7231746045625118 w[11,5]
   - 10.61578362637059 w[11,6] - 11.43016776405479 w[11,7]
   - 5.997954058161801 w[11,8] - 13.02686521454084 w[11,9]
   - 13.12292964535524 w[11,10] - 9.789935768555161 w[11,12]
   - 7.427623967124651 w[11,13] - 4.794137809929045 w[11,14]
   - 0.7231746045625118 w[11,15] - 10.61578362637059 w[11,16]
   - 11.43016776405479 w[11,17] - 5.997954058161801 w[11,18]
   - 13.02686521454084 w[11,19] - 13.12292964535524 w[11,20]
   - 12.48468640129708 w[11,22] - 12.48468640129708 w[11,23]
   - 4.344331240562464 w[12,1] - 2.61003755895037 w[12,3]
   - 10.45913911847039 w[12,4] - 10.51200846194448 w[12,5]
   - 5.445922708582885 w[12,6] - 5.080731659730024 w[12,7]
   - 6.347317788032758 w[12,8] - 3.422284583926837 w[12,9]
   - 3.345499433306067 w[12,10] - 9.789935768555161 w[12,11]
   - 2.61003755895037 w[12,13] - 10.45913911847039 w[12,14]
   - 10.51200846194448 w[12,15] - 5.445922708582885 w[12,16]
   - 5.080731659730024 w[12,17] - 6.347317788032758 w[12,18]
   - 3.422284583926837 w[12,19] - 3.345499433306067 w[12,20]
   - 9.789935768555161 w[12,21] - 4.344331240562464 w[12,22]
   - 4.344331240562464 w[12,23] - 6.573837617007667 w[13,1]
   - 2.61003755895037 w[13,2] - 8.95878742215544 w[13,4]
   - 8.149041695863 w[13,5] - 4.781941277589094 w[13,6]
   - 5.009125786524788 w[13,7] - 5.435075522044343 w[13,8]
   - 6.032305321062844 w[13,9] - 5.788348893748577 w[13,10]
   - 7.427623967124651 w[13,11] - 2.61003755895037 w[13,12]
   - 8.95878742215544 w[13,14] - 8.149041695863 w[13,15]
   - 4.781941277589094 w[13,16] - 5.009125786524788 w[13,17]
   - 5.435075522044343 w[13,18] - 6.032305321062844 w[13,19]
   - 5.788348893748577 w[13,20] - 7.427623967124651 w[13,21]
   - 6.573837617007667 w[13,22] - 6.573837617007667 w[13,23]
   - 11.4508499411422 w[14,1] - 10.45913911847039 w[14,2]
   - 8.95878742215544 w[14,3] - 4.879336040253222 w[14,5]
   - 13.35860992801736 w[14,6] - 13.85739579475303 w[14,7]
   - 4.313284215254392 w[14,8] - 12.96073712322885 w[14,9]
   - 13.62887851321657 w[14,10] - 4.794137809929045 w[14,11]
   - 10.45913911847039 w[14,12] - 8.95878742215544 w[14,13]
   - 4.879336040253222 w[14,15] - 13.35860992801736 w[14,16]
   - 13.85739579475303 w[14,17] - 4.313284215254392 w[14,18]
   - 12.96073712322885 w[14,19] - 13.62887851321657 w[14,20]
   - 4.794137809929045 w[14,21] - 11.4508499411422 w[14,22]
   - 11.4508499411422 w[14,23] - 13.16234276575076 w[15,1]
   - 10.51200846194448 w[15,2] - 8.149041695863 w[15,3]
   - 4.879336040253222 w[15,4] - 11.26308377322209 w[15,6]
   - 12.09916693733823 w[15,7] - 6.552694434412026 w[15,8]
   - 13.74175662106976 w[15,9] - 13.8457595723512 w[15,10]
   - 0.7231746045625118 w[15,11] - 10.51200846194448 w[15,12]
   - 8.149041695863 w[15,13] - 4.879336040253222 w[15,14]
   - 11.26308377322209 w[15,16] - 12.09916693733823 w[15,17]
   - 6.552694434412026 w[15,18] - 13.74175662106976 w[15,19]
   - 13.8457595723512 w[15,20] - 0.7231746045625118 w[15,21]
   - 13.16234276575076 w[15,22] - 13.16234276575076 w[15,23]
   - 9.612112802177897 w[16,1] - 5.445922708582885 w[16,2]
   - 4.781941277589094 w[16,3] - 13.35860992801736 w[16,4]
   - 11.26308377322209 w[16,5] - 1.165989466062996 w[16,7]
   - 10.19268680045306 w[16,8] - 7.704720764512761 w[16,9]
   - 6.448801192803242 w[16,10] - 10.61578362637059 w[16,11]
   - 5.445922708582885 w[16,12] - 4.781941277589094 w[16,13]
   - 13.35860992801736 w[16,14] - 11.26308377322209 w[16,15]
   - 1.165989466062996 w[16,17] - 10.19268680045306 w[16,18]
   - 7.704720764512761 w[16,19] - 6.448801192803242 w[16,20]
   - 10.61578362637059 w[16,21] - 9.612112802177897 w[16,22]
   - 9.612112802177897 w[16,23] - 9.012958145309993 w[17,1]
   - 5.080731659730024 w[17,2] - 5.009125786524788 w[17,3]
   - 13.85739579475303 w[17,4] - 12.09916693733823 w[17,5]
   - 1.165989466062996 w[17,6] - 10.43222375502204 w[17,8]
   - 6.896688334011931 w[17,9] - 5.528381888071401 w[17,10]
   - 11.43016776405479 w[17,11] - 5.080731659730024 w[17,12]
   - 5.009125786524788 w[17,13] - 13.85739579475303 w[17,14]
   - 12.09916693733823 w[17,15] - 1.165989466062996 w[17,16]
   - 10.43222375502204 w[17,18] - 6.896688334011931 w[17,19]
   - 5.528381888071401 w[17,20] - 11.43016776405479 w[17,21]
   - 9.012958145309993 w[17,22] - 9.012958145309993 w[17,23]
   - 7.217095724205025 w[18,1] - 6.347317788032758 w[18,2]
   - 5.435075522044343 w[18,3] - 4.313284215254392 w[18,4]
   - 6.552694434412026 w[18,5] - 10.19268680045306 w[18,6]
   - 10.43222375502204 w[18,7] - 8.651601560727855 w[18,9]
   - 9.38349272973112 w[18,10] - 5.997954058161801 w[18,11]
   - 6.347317788032758 w[18,12] - 5.435075522044343 w[18,13]
   - 4.313284215254392 w[18,14] - 6.552694434412026 w[18,15]
   - 10.19268680045306 w[18,16] - 10.43222375502204 w[18,17]
   - 8.651601560727855 w[18,19] - 9.38349272973112 w[18,20]
   - 5.997954058161801 w[18,21] - 7.217095724205025 w[18,22]
   - 7.217095724205025 w[18,23] - 2.740520286016073 w[19,1]
   - 3.422284583926837 w[19,2] - 6.032305321062844 w[19,3]
   - 12.96073712322885 w[19,4] - 13.74175662106976 w[19,5]
   - 7.704720764512761 w[19,6] - 6.896688334011931 w[19,7]
   - 8.651601560727855 w[19,8] - 1.62199593217746 w[19,10]
   - 13.02686521454084 w[19,11] - 3.422284583926837 w[19,12]
   - 6.032305321062844 w[19,13] - 12.96073712322885 w[19,14]
   - 13.74175662106976 w[19,15] - 7.704720764512761 w[19,16]
   - 6.896688334011931 w[19,17] - 8.651601560727855 w[19,18]
   - 1.62199593217746 w[19,20] - 13.02686521454084 w[19,21]
   - 2.740520286016073 w[19,22] - 2.740520286016073 w[19,23]
   - 4.314811590792756 w[20,1] - 3.345499433306067 w[20,2]
   - 5.788348893748577 w[20,3] - 13.62887851321657 w[20,4]
   - 13.8457595723512 w[20,5] - 6.448801192803242 w[20,6]
   - 5.528381888071401 w[20,7] - 9.38349272973112 w[20,8]
   - 1.62199593217746 w[20,9] - 13.12292964535524 w[20,11]
   - 3.345499433306067 w[20,12] - 5.788348893748577 w[20,13]
   - 13.62887851321657 w[20,14] - 13.8457595723512 w[20,15]
   - 6.448801192803242 w[20,16] - 5.528381888071401 w[20,17]
   - 9.38349272973112 w[20,18] - 1.62199593217746 w[20,19]
   - 13.12292964535524 w[20,21] - 4.314811590792756 w[20,22]
   - 4.314811590792756 w[20,23] - 12.48468640129708 w[21,1]
   - 9.789935768555161 w[21,2] - 7.427623967124651 w[21,3]
   - 4.794137809929045 w[21,4] - 0.7231746045625118 w[21,5]
   - 10.61578362637059 w[21,6] - 11.43016776405479 w[21,7]
   - 5.997954058161801 w[21,8] - 13.02686521454084 w[21,9]
   - 13.12292964535524 w[21,10] - 9.789935768555161 w[21,12]
   - 7.427623967124651 w[21,13] - 4.794137809929045 w[21,14]
   - 0.7231746045625118 w[21,15] - 10.61578362637059 w[21,16]
   - 11.43016776405479 w[21,17] - 5.997954058161801 w[21,18]
   - 13.02686521454084 w[21,19] - 13.12292964535524 w[21,20]
   - 12.48468640129708 w[21,22] - 12.48468640129708 w[21,23]
   - 4.344331240562464 w[22,2] - 6.573837617007667 w[22,3]
   - 11.4508499411422 w[22,4] - 13.16234276575076 w[22,5]
   - 9.612112802177897 w[22,6] - 9.012958145309993 w[22,7]
   - 7.217095724205025 w[22,8] - 2.740520286016073 w[22,9]
   - 4.314811590792756 w[22,10] - 12.48468640129708 w[22,11]
   - 4.344331240562464 w[22,12] - 6.573837617007667 w[22,13]
   - 11.4508499411422 w[22,14] - 13.16234276575076 w[22,15]
   - 9.612112802177897 w[22,16] - 9.012958145309993 w[22,17]
   - 7.217095724205025 w[22,18] - 2.740520286016073 w[22,19]
   - 4.314811590792756 w[22,20] - 12.48468640129708 w[22,21] - 40 y[2,1]
   - 80 y[2,2] - 58.57142857142857 y[3,1] - 117.1428571428571 y[3,2]
   - 62.85714285714285 y[4,1] - 125.7142857142857 y[4,2]
   - 55.71428571428571 y[5,1] - 111.4285714285714 y[5,2] - 30 y[6,1]
   - 60 y[6,2] - 32.85714285714285 y[7,1] - 65.71428571428571 y[7,2]
   - 54.28571428571428 y[8,1] - 108.5714285714286 y[8,2]
   - 55.71428571428571 y[9,1] - 111.4285714285714 y[9,2] - 60 y[10,1]
   - 120 y[10,2] - 58.57142857142857 y[11,1] - 117.1428571428571 y[11,2]
   + start_service[23] >= 0
 AverageTimeVehices: - 10.13677289464575 x[0,2] - 15.33895443968456 x[0,3]
   - 26.71864986266515 x[0,4] - 30.7121331200851 x[0,5]
   - 22.42826320508176 x[0,6] - 21.03023567238998 x[0,7]
   - 16.83989002314506 x[0,8] - 6.394547334037503 x[0,9]
   - 10.06789371184976 x[0,10] - 29.13093493635985 x[0,11]
   - 10.13677289464575 x[0,12] - 15.33895443968456 x[0,13]
   - 26.71864986266515 x[0,14] - 30.7121331200851 x[0,15]
   - 22.42826320508176 x[0,16] - 21.03023567238998 x[0,17]
   - 16.83989002314506 x[0,18] - 6.394547334037503 x[0,19]
   - 10.06789371184976 x[0,20] - 29.13093493635985 x[0,21]
   - 10.13677289464575 x[1,2] - 15.33895443968456 x[1,3]
   - 26.71864986266515 x[1,4] - 30.7121331200851 x[1,5]
   - 22.42826320508176 x[1,6] - 21.03023567238998 x[1,7]
   - 16.83989002314506 x[1,8] - 6.394547334037503 x[1,9]
   - 10.06789371184976 x[1,10] - 29.13093493635985 x[1,11]
   - 10.13677289464575 x[1,12] - 15.33895443968456 x[1,13]
   - 26.71864986266515 x[1,14] - 30.7121331200851 x[1,15]
   - 22.42826320508176 x[1,16] - 21.03023567238998 x[1,17]
   - 16.83989002314506 x[1,18] - 6.394547334037503 x[1,19]
   - 10.06789371184976 x[1,20] - 29.13093493635985 x[1,21]
   - 10.13677289464575 x[2,1] - 6.090087637550863 x[2,3]
   - 24.40465794309757 x[2,4] - 24.52801974453712 x[2,5]
   - 12.7071529866934 x[2,6] - 11.85504053937006 x[2,7]
   - 14.81040817207644 x[2,8] - 7.985330695829287 x[2,9]
   - 7.806165344380823 x[2,10] - 22.84318345996204 x[2,11]
   - 6.090087637550863 x[2,13] - 24.40465794309757 x[2,14]
   - 24.52801974453712 x[2,15] - 12.7071529866934 x[2,16]
   - 11.85504053937006 x[2,17] - 14.81040817207644 x[2,18]
   - 7.985330695829287 x[2,19] - 7.806165344380823 x[2,20]
   - 22.84318345996204 x[2,21] - 10.13677289464575 x[2,22]
   - 10.13677289464575 x[2,23] - 15.33895443968456 x[3,1]
   - 6.090087637550863 x[3,2] - 20.90383731836269 x[3,4]
   - 19.01443062368033 x[3,5] - 11.15786298104122 x[3,6]
   - 11.68796016855784 x[3,7] - 12.68184288477013 x[3,8]
   - 14.07537908247997 x[3,9] - 13.50614741874668 x[3,10]
   - 17.33112258995752 x[3,11] - 6.090087637550863 x[3,12]
   - 20.90383731836269 x[3,14] - 19.01443062368033 x[3,15]
   - 11.15786298104122 x[3,16] - 11.68796016855784 x[3,17]
   - 12.68184288477013 x[3,18] - 14.07537908247997 x[3,19]
   - 13.50614741874668 x[3,20] - 17.33112258995752 x[3,21]
   - 15.33895443968456 x[3,22] - 15.33895443968456 x[3,23]
   - 26.71864986266515 x[4,1] - 24.40465794309757 x[4,2]
   - 20.90383731836269 x[4,3] - 11.38511742725752 x[4,5]
   - 31.17008983204052 x[4,6] - 32.3339235210904 x[4,7]
   - 10.06432983559358 x[4,8] - 30.24171995420065 x[4,9]
   - 31.80071653083866 x[4,10] - 11.18632155650111 x[4,11]
   - 24.40465794309757 x[4,12] - 20.90383731836269 x[4,13]
   - 11.38511742725752 x[4,15] - 31.17008983204052 x[4,16]
   - 32.3339235210904 x[4,17] - 10.06432983559358 x[4,18]
   - 30.24171995420065 x[4,19] - 31.80071653083866 x[4,20]
   - 11.18632155650111 x[4,21] - 26.71864986266515 x[4,22]
   - 26.71864986266515 x[4,23] - 30.7121331200851 x[5,1]
   - 24.52801974453712 x[5,2] - 19.01443062368033 x[5,3]
   - 11.38511742725752 x[5,4] - 26.28052880418488 x[5,6]
   - 28.23138952045586 x[5,7] - 15.28962034696139 x[5,8]
   - 32.06409878249611 x[5,9] - 32.30677233548614 x[5,10]
   - 1.687407410645861 x[5,11] - 24.52801974453712 x[5,12]
   - 19.01443062368033 x[5,13] - 11.38511742725752 x[5,14]
   - 26.28052880418488 x[5,16] - 28.23138952045586 x[5,17]
   - 15.28962034696139 x[5,18] - 32.06409878249611 x[5,19]
   - 32.30677233548614 x[5,20] - 1.687407410645861 x[5,21]
   - 30.7121331200851 x[5,22] - 30.7121331200851 x[5,23]
   - 22.42826320508176 x[6,1] - 12.7071529866934 x[6,2]
   - 11.15786298104122 x[6,3] - 31.17008983204052 x[6,4]
   - 26.28052880418488 x[6,5] - 2.720642087480323 x[6,7]
   - 23.78293586772381 x[6,8] - 17.97768178386311 x[6,9]
   - 15.04720278320757 x[6,10] - 24.77016179486472 x[6,11]
   - 12.7071529866934 x[6,12] - 11.15786298104122 x[6,13]
   - 31.17008983204052 x[6,14] - 26.28052880418488 x[6,15]
   - 2.720642087480323 x[6,17] - 23.78293586772381 x[6,18]
   - 17.97768178386311 x[6,19] - 15.04720278320757 x[6,20]
   - 24.77016179486472 x[6,21] - 22.42826320508176 x[6,22]
   - 22.42826320508176 x[6,23] - 21.03023567238998 x[7,1]
   - 11.85504053937006 x[7,2] - 11.68796016855784 x[7,3]
   - 32.3339235210904 x[7,4] - 28.23138952045586 x[7,5]
   - 2.720642087480323 x[7,6] - 24.34185542838475 x[7,8]
   - 16.09227277936117 x[7,9] - 12.89955773883327 x[7,10]
   - 26.67039144946118 x[7,11] - 11.85504053937006 x[7,12]
   - 11.68796016855784 x[7,13] - 32.3339235210904 x[7,14]
   - 28.23138952045586 x[7,15] - 2.720642087480323 x[7,16]
   - 24.34185542838475 x[7,18] - 16.09227277936117 x[7,19]
   - 12.89955773883327 x[7,20] - 26.67039144946118 x[7,21]
   - 21.03023567238998 x[7,22] - 21.03023567238998 x[7,23]
   - 16.83989002314506 x[8,1] - 14.81040817207644 x[8,2]
   - 12.68184288477013 x[8,3] - 10.06432983559358 x[8,4]
   - 15.28962034696139 x[8,5] - 23.78293586772381 x[8,6]
   - 24.34185542838475 x[8,7] - 20.18707030836499 x[8,9]
   - 21.89481636937261 x[8,10] - 13.99522613571087 x[8,11]
   - 14.81040817207644 x[8,12] - 12.68184288477013 x[8,13]
   - 10.06432983559358 x[8,14] - 15.28962034696139 x[8,15]
   - 23.78293586772381 x[8,16] - 24.34185542838475 x[8,17]
   - 20.18707030836499 x[8,19] - 21.89481636937261 x[8,20]
   - 13.99522613571087 x[8,21] - 16.83989002314506 x[8,22]
   - 16.83989002314506 x[8,23] - 6.394547334037503 x[9,1]
   - 7.985330695829287 x[9,2] - 14.07537908247997 x[9,3]
   - 30.24171995420065 x[9,4] - 32.06409878249611 x[9,5]
   - 17.97768178386311 x[9,6] - 16.09227277936117 x[9,7]
   - 20.18707030836499 x[9,8] - 3.78465717508074 x[9,10]
   - 30.39601883392863 x[9,11] - 7.985330695829287 x[9,12]
   - 14.07537908247997 x[9,13] - 30.24171995420065 x[9,14]
   - 32.06409878249611 x[9,15] - 17.97768178386311 x[9,16]
   - 16.09227277936117 x[9,17] - 20.18707030836499 x[9,18]
   - 3.78465717508074 x[9,20] - 30.39601883392863 x[9,21]
   - 6.394547334037503 x[9,22] - 6.394547334037503 x[9,23]
   - 10.06789371184976 x[10,1] - 7.806165344380823 x[10,2]
   - 13.50614741874668 x[10,3] - 31.80071653083866 x[10,4]
   - 32.30677233548614 x[10,5] - 15.04720278320757 x[10,6]
   - 12.89955773883327 x[10,7] - 21.89481636937261 x[10,8]
   - 3.78465717508074 x[10,9] - 30.62016917249555 x[10,11]
   - 7.806165344380823 x[10,12] - 13.50614741874668 x[10,13]
   - 31.80071653083866 x[10,14] - 32.30677233548614 x[10,15]
   - 15.04720278320757 x[10,16] - 12.89955773883327 x[10,17]
   - 21.89481636937261 x[10,18] - 3.78465717508074 x[10,19]
   - 30.62016917249555 x[10,21] - 10.06789371184976 x[10,22]
   - 10.06789371184976 x[10,23] - 29.13093493635985 x[11,1]
   - 22.84318345996204 x[11,2] - 17.33112258995752 x[11,3]
   - 11.18632155650111 x[11,4] - 1.687407410645861 x[11,5]
   - 24.77016179486472 x[11,6] - 26.67039144946118 x[11,7]
   - 13.99522613571087 x[11,8] - 30.39601883392863 x[11,9]
   - 30.62016917249555 x[11,10] - 22.84318345996204 x[11,12]
   - 17.33112258995752 x[11,13] - 11.18632155650111 x[11,14]
   - 1.687407410645861 x[11,15] - 24.77016179486472 x[11,16]
   - 26.67039144946118 x[11,17] - 13.99522613571087 x[11,18]
   - 30.39601883392863 x[11,19] - 30.62016917249555 x[11,20]
   - 29.13093493635985 x[11,22] - 29.13093493635985 x[11,23]
   - 10.13677289464575 x[12,1] - 6.090087637550863 x[12,3]
   - 24.40465794309757 x[12,4] - 24.52801974453712 x[12,5]
   - 12.7071529866934 x[12,6] - 11.85504053937006 x[12,7]
   - 14.81040817207644 x[12,8] - 7.985330695829287 x[12,9]
   - 7.806165344380823 x[12,10] - 22.84318345996204 x[12,11]
   - 6.090087637550863 x[12,13] - 24.40465794309757 x[12,14]
   - 24.52801974453712 x[12,15] - 12.7071529866934 x[12,16]
   - 11.85504053937006 x[12,17] - 14.81040817207644 x[12,18]
   - 7.985330695829287 x[12,19] - 7.806165344380823 x[12,20]
   - 22.84318345996204 x[12,21] - 10.13677289464575 x[12,22]
   - 10.13677289464575 x[12,23] - 15.33895443968456 x[13,1]
   - 6.090087637550863 x[13,2] - 20.90383731836269 x[13,4]
   - 19.01443062368033 x[13,5] - 11.15786298104122 x[13,6]
   - 11.68796016855784 x[13,7] - 12.68184288477013 x[13,8]
   - 14.07537908247997 x[13,9] - 13.50614741874668 x[13,10]
   - 17.33112258995752 x[13,11] - 6.090087637550863 x[13,12]
   - 20.90383731836269 x[13,14] - 19.01443062368033 x[13,15]
   - 11.15786298104122 x[13,16] - 11.68796016855784 x[13,17]
   - 12.68184288477013 x[13,18] - 14.07537908247997 x[13,19]
   - 13.50614741874668 x[13,20] - 17.33112258995752 x[13,21]
   - 15.33895443968456 x[13,22] - 15.33895443968456 x[13,23]
   - 26.71864986266515 x[14,1] - 24.40465794309757 x[14,2]
   - 20.90383731836269 x[14,3] - 11.38511742725752 x[14,5]
   - 31.17008983204052 x[14,6] - 32.3339235210904 x[14,7]
   - 10.06432983559358 x[14,8] - 30.24171995420065 x[14,9]
   - 31.80071653083866 x[14,10] - 11.18632155650111 x[14,11]
   - 24.40465794309757 x[14,12] - 20.90383731836269 x[14,13]
   - 11.38511742725752 x[14,15] - 31.17008983204052 x[14,16]
   - 32.3339235210904 x[14,17] - 10.06432983559358 x[14,18]
   - 30.24171995420065 x[14,19] - 31.80071653083866 x[14,20]
   - 11.18632155650111 x[14,21] - 26.71864986266515 x[14,22]
   - 26.71864986266515 x[14,23] - 30.7121331200851 x[15,1]
   - 24.52801974453712 x[15,2] - 19.01443062368033 x[15,3]
   - 11.38511742725752 x[15,4] - 26.28052880418488 x[15,6]
   - 28.23138952045586 x[15,7] - 15.28962034696139 x[15,8]
   - 32.06409878249611 x[15,9] - 32.30677233548614 x[15,10]
   - 1.687407410645861 x[15,11] - 24.52801974453712 x[15,12]
   - 19.01443062368033 x[15,13] - 11.38511742725752 x[15,14]
   - 26.28052880418488 x[15,16] - 28.23138952045586 x[15,17]
   - 15.28962034696139 x[15,18] - 32.06409878249611 x[15,19]
   - 32.30677233548614 x[15,20] - 1.687407410645861 x[15,21]
   - 30.7121331200851 x[15,22] - 30.7121331200851 x[15,23]
   - 22.42826320508176 x[16,1] - 12.7071529866934 x[16,2]
   - 11.15786298104122 x[16,3] - 31.17008983204052 x[16,4]
   - 26.28052880418488 x[16,5] - 2.720642087480323 x[16,7]
   - 23.78293586772381 x[16,8] - 17.97768178386311 x[16,9]
   - 15.04720278320757 x[16,10] - 24.77016179486472 x[16,11]
   - 12.7071529866934 x[16,12] - 11.15786298104122 x[16,13]
   - 31.17008983204052 x[16,14] - 26.28052880418488 x[16,15]
   - 2.720642087480323 x[16,17] - 23.78293586772381 x[16,18]
   - 17.97768178386311 x[16,19] - 15.04720278320757 x[16,20]
   - 24.77016179486472 x[16,21] - 22.42826320508176 x[16,22]
   - 22.42826320508176 x[16,23] - 21.03023567238998 x[17,1]
   - 11.85504053937006 x[17,2] - 11.68796016855784 x[17,3]
   - 32.3339235210904 x[17,4] - 28.23138952045586 x[17,5]
   - 2.720642087480323 x[17,6] - 24.34185542838475 x[17,8]
   - 16.09227277936117 x[17,9] - 12.89955773883327 x[17,10]
   - 26.67039144946118 x[17,11] - 11.85504053937006 x[17,12]
   - 11.68796016855784 x[17,13] - 32.3339235210904 x[17,14]
   - 28.23138952045586 x[17,15] - 2.720642087480323 x[17,16]
   - 24.34185542838475 x[17,18] - 16.09227277936117 x[17,19]
   - 12.89955773883327 x[17,20] - 26.67039144946118 x[17,21]
   - 21.03023567238998 x[17,22] - 21.03023567238998 x[17,23]
   - 16.83989002314506 x[18,1] - 14.81040817207644 x[18,2]
   - 12.68184288477013 x[18,3] - 10.06432983559358 x[18,4]
   - 15.28962034696139 x[18,5] - 23.78293586772381 x[18,6]
   - 24.34185542838475 x[18,7] - 20.18707030836499 x[18,9]
   - 21.89481636937261 x[18,10] - 13.99522613571087 x[18,11]
   - 14.81040817207644 x[18,12] - 12.68184288477013 x[18,13]
   - 10.06432983559358 x[18,14] - 15.28962034696139 x[18,15]
   - 23.78293586772381 x[18,16] - 24.34185542838475 x[18,17]
   - 20.18707030836499 x[18,19] - 21.89481636937261 x[18,20]
   - 13.99522613571087 x[18,21] - 16.83989002314506 x[18,22]
   - 16.83989002314506 x[18,23] - 6.394547334037503 x[19,1]
   - 7.985330695829287 x[19,2] - 14.07537908247997 x[19,3]
   - 30.24171995420065 x[19,4] - 32.06409878249611 x[19,5]
   - 17.97768178386311 x[19,6] - 16.09227277936117 x[19,7]
   - 20.18707030836499 x[19,8] - 3.78465717508074 x[19,10]
   - 30.39601883392863 x[19,11] - 7.985330695829287 x[19,12]
   - 14.07537908247997 x[19,13] - 30.24171995420065 x[19,14]
   - 32.06409878249611 x[19,15] - 17.97768178386311 x[19,16]
   - 16.09227277936117 x[19,17] - 20.18707030836499 x[19,18]
   - 3.78465717508074 x[19,20] - 30.39601883392863 x[19,21]
   - 6.394547334037503 x[19,22] - 6.394547334037503 x[19,23]
   - 10.06789371184976 x[20,1] - 7.806165344380823 x[20,2]
   - 13.50614741874668 x[20,3] - 31.80071653083866 x[20,4]
   - 32.30677233548614 x[20,5] - 15.04720278320757 x[20,6]
   - 12.89955773883327 x[20,7] - 21.89481636937261 x[20,8]
   - 3.78465717508074 x[20,9] - 30.62016917249555 x[20,11]
   - 7.806165344380823 x[20,12] - 13.50614741874668 x[20,13]
   - 31.80071653083866 x[20,14] - 32.30677233548614 x[20,15]
   - 15.04720278320757 x[20,16] - 12.89955773883327 x[20,17]
   - 21.89481636937261 x[20,18] - 3.78465717508074 x[20,19]
   - 30.62016917249555 x[20,21] - 10.06789371184976 x[20,22]
   - 10.06789371184976 x[20,23] - 29.13093493635985 x[21,1]
   - 22.84318345996204 x[21,2] - 17.33112258995752 x[21,3]
   - 11.18632155650111 x[21,4] - 1.687407410645861 x[21,5]
   - 24.77016179486472 x[21,6] - 26.67039144946118 x[21,7]
   - 13.99522613571087 x[21,8] - 30.39601883392863 x[21,9]
   - 30.62016917249555 x[21,10] - 22.84318345996204 x[21,12]
   - 17.33112258995752 x[21,13] - 11.18632155650111 x[21,14]
   - 1.687407410645861 x[21,15] - 24.77016179486472 x[21,16]
   - 26.67039144946118 x[21,17] - 13.99522613571087 x[21,18]
   - 30.39601883392863 x[21,19] - 30.62016917249555 x[21,20]
   - 29.13093493635985 x[21,22] - 29.13093493635985 x[21,23]
   - 10.13677289464575 x[22,2] - 15.33895443968456 x[22,3]
   - 26.71864986266515 x[22,4] - 30.7121331200851 x[22,5]
   - 22.42826320508176 x[22,6] - 21.03023567238998 x[22,7]
   - 16.83989002314506 x[22,8] - 6.394547334037503 x[22,9]
   - 10.06789371184976 x[22,10] - 29.13093493635985 x[22,11]
   - 10.13677289464575 x[22,12] - 15.33895443968456 x[22,13]
   - 26.71864986266515 x[22,14] - 30.7121331200851 x[22,15]
   - 22.42826320508176 x[22,16] - 21.03023567238998 x[22,17]
   - 16.83989002314506 x[22,18] - 6.394547334037503 x[22,19]
   - 10.06789371184976 x[22,20] - 29.13093493635985 x[22,21]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[0]: - 60.8206373678745 x[0,2]
   - 92.03372663810735 x[0,3] - 160.3118991759909 x[0,4]
   - 184.2727987205106 x[0,5] - 134.5695792304906 x[0,6]
   - 126.1814140343399 x[0,7] - 101.0393401388704 x[0,8]
   - 38.36728400422502 x[0,9] - 60.40736227109858 x[0,10]
   - 174.7856096181591 x[0,11] - 60.8206373678745 x[0,12]
   - 92.03372663810735 x[0,13] - 160.3118991759909 x[0,14]
   - 184.2727987205106 x[0,15] - 134.5695792304906 x[0,16]
   - 126.1814140343399 x[0,17] - 101.0393401388704 x[0,18]
   - 38.36728400422502 x[0,19] - 60.40736227109858 x[0,20]
   - 174.7856096181591 x[0,21] - service_time[0] - start_service[0]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[1]: - 60.8206373678745 x[1,2]
   - 92.03372663810735 x[1,3] - 160.3118991759909 x[1,4]
   - 184.2727987205106 x[1,5] - 134.5695792304906 x[1,6]
   - 126.1814140343399 x[1,7] - 101.0393401388704 x[1,8]
   - 38.36728400422502 x[1,9] - 60.40736227109858 x[1,10]
   - 174.7856096181591 x[1,11] - 60.8206373678745 x[1,12]
   - 92.03372663810735 x[1,13] - 160.3118991759909 x[1,14]
   - 184.2727987205106 x[1,15] - 134.5695792304906 x[1,16]
   - 126.1814140343399 x[1,17] - 101.0393401388704 x[1,18]
   - 38.36728400422502 x[1,19] - 60.40736227109858 x[1,20]
   - 174.7856096181591 x[1,21] - service_time[1] - start_service[1]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[2]: - 30.41031868393725 x[2,1]
   - 30.41031868393725 x[2,2] - 64.28712623170627 x[2,3]
   - 153.3699234172881 x[2,4] - 165.7204585938667 x[2,5]
   - 105.4062485753255 x[2,6] - 98.65582863528012 x[2,7]
   - 94.95089458566449 x[2,8] - 43.13963408960037 x[2,9]
   - 53.62217716869176 x[2,10] - 155.9223551889657 x[2,11]
   - 30.41031868393725 x[2,12] - 64.28712623170627 x[2,13]
   - 153.3699234172881 x[2,14] - 165.7204585938667 x[2,15]
   - 105.4062485753255 x[2,16] - 98.65582863528012 x[2,17]
   - 94.95089458566449 x[2,18] - 43.13963408960037 x[2,19]
   - 53.62217716869176 x[2,20] - 155.9223551889657 x[2,21]
   - 30.41031868393725 x[2,22] - 30.41031868393725 x[2,23]
   - service_time[2] - start_service[2] + start_service[23] >= 0
 LowerBoundRoutingVehicle[3]: - 46.01686331905368 x[3,1]
   - 48.68058159658984 x[3,2] - 46.01686331905368 x[3,3]
   - 142.8674615430835 x[3,4] - 149.1796912312963 x[3,5]
   - 100.7583785583689 x[3,6] - 98.15458752284347 x[3,7]
   - 88.56519872374558 x[3,8] - 61.40977924955241 x[3,9]
   - 70.72212339178934 x[3,10] - 139.3861725789521 x[3,11]
   - 48.68058159658984 x[3,12] - 46.01686331905368 x[3,13]
   - 142.8674615430835 x[3,14] - 149.1796912312963 x[3,15]
   - 100.7583785583689 x[3,16] - 98.15458752284347 x[3,17]
   - 88.56519872374558 x[3,18] - 61.40977924955241 x[3,19]
   - 70.72212339178934 x[3,20] - 139.3861725789521 x[3,21]
   - 46.01686331905368 x[3,22] - 46.01686331905368 x[3,23]
   - service_time[3] - start_service[3] + start_service[23] >= 0
 LowerBoundRoutingVehicle[4]: - 80.15594958799544 x[4,1]
   - 103.62429251323 x[4,2] - 108.7283752741418 x[4,3]
   - 80.15594958799544 x[4,4] - 126.2917516420279 x[4,5]
   - 160.7950591113668 x[4,6] - 160.0924775804411 x[4,7]
   - 80.71265957621593 x[4,8] - 109.9088018647144 x[4,9]
   - 125.6058307280653 x[4,10] - 120.9517694785829 x[4,11]
   - 103.62429251323 x[4,12] - 108.7283752741418 x[4,13]
   - 80.15594958799544 x[4,14] - 126.2917516420279 x[4,15]
   - 160.7950591113668 x[4,16] - 160.0924775804411 x[4,17]
   - 80.71265957621593 x[4,18] - 109.9088018647144 x[4,19]
   - 125.6058307280653 x[4,20] - 120.9517694785829 x[4,21]
   - 80.15594958799544 x[4,22] - 80.15594958799544 x[4,23]
   - service_time[4] - start_service[4] + start_service[23] >= 0
 LowerBoundRoutingVehicle[5]: - 92.13639936025531 x[5,1]
   - 103.9943779175486 x[5,2] - 103.0601551900947 x[5,3]
   - 114.311301869768 x[5,4] - 92.13639936025531 x[5,5]
   - 146.1263760277999 x[5,6] - 147.7848755785375 x[5,7]
   - 96.38853111031936 x[5,8] - 115.3759383496008 x[5,9]
   - 127.1239981420077 x[5,10] - 92.45502704101713 x[5,11]
   - 103.9943779175486 x[5,12] - 103.0601551900947 x[5,13]
   - 114.311301869768 x[5,14] - 92.13639936025531 x[5,15]
   - 146.1263760277999 x[5,16] - 147.7848755785375 x[5,17]
   - 96.38853111031936 x[5,18] - 115.3759383496008 x[5,19]
   - 127.1239981420077 x[5,20] - 92.45502704101713 x[5,21]
   - 92.13639936025531 x[5,22] - 92.13639936025531 x[5,23]
   - service_time[5] - start_service[5] + start_service[23] >= 0
 LowerBoundRoutingVehicle[6]: - 67.28478961524529 x[6,1]
   - 68.53177764401744 x[6,2] - 79.49045226217734 x[6,3]
   - 173.666219084117 x[6,4] - 170.9779857728099 x[6,5]
   - 67.28478961524529 x[6,6] - 71.25263327961092 x[6,7]
   - 121.8684776726066 x[6,8] - 73.11668735370183 x[6,9]
   - 75.34528948517199 x[6,10] - 161.7032901936737 x[6,11]
   - 68.53177764401744 x[6,12] - 79.49045226217734 x[6,13]
   - 173.666219084117 x[6,14] - 170.9779857728099 x[6,15]
   - 67.28478961524529 x[6,16] - 71.25263327961092 x[6,17]
   - 121.8684776726066 x[6,18] - 73.11668735370183 x[6,19]
   - 75.34528948517199 x[6,20] - 161.7032901936737 x[6,21]
   - 67.28478961524529 x[6,22] - 67.28478961524529 x[6,23]
   - service_time[6] - start_service[6] + start_service[23] >= 0
 LowerBoundRoutingVehicle[7]: - 63.09070701716995 x[7,1]
   - 65.97544030204742 x[7,2] - 81.08074382472719 x[7,3]
   - 177.1577201512666 x[7,4] - 176.8305679216229 x[7,5]
   - 75.44671587768626 x[7,6] - 63.09070701716995 x[7,7]
   - 123.5452363545894 x[7,8] - 67.46046034019604 x[7,9]
   - 68.9023543520491 x[7,10] - 167.4039791574631 x[7,11]
   - 65.97544030204742 x[7,12] - 81.08074382472719 x[7,13]
   - 177.1577201512666 x[7,14] - 176.8305679216229 x[7,15]
   - 75.44671587768626 x[7,16] - 63.09070701716995 x[7,17]
   - 123.5452363545894 x[7,18] - 67.46046034019604 x[7,19]
   - 68.9023543520491 x[7,20] - 167.4039791574631 x[7,21]
   - 63.09070701716995 x[7,22] - 63.09070701716995 x[7,23]
   - service_time[7] - start_service[7] + start_service[23] >= 0
 LowerBoundRoutingVehicle[8]: - 50.51967006943518 x[8,1]
   - 74.84154320016656 x[8,2] - 84.06239197336407 x[8,3]
   - 110.3489390947762 x[8,4] - 138.0052604011395 x[8,5]
   - 138.6335972184168 x[8,6] - 136.1162733023242 x[8,7]
   - 50.51967006943518 x[8,8] - 79.7448529272075 x[8,9]
   - 95.88813024366713 x[8,10] - 129.3784832162121 x[8,11]
   - 74.84154320016656 x[8,12] - 84.06239197336407 x[8,13]
   - 110.3489390947762 x[8,14] - 138.0052604011395 x[8,15]
   - 138.6335972184168 x[8,16] - 136.1162733023242 x[8,17]
   - 50.51967006943518 x[8,18] - 79.7448529272075 x[8,19]
   - 95.88813024366713 x[8,20] - 129.3784832162121 x[8,21]
   - 50.51967006943518 x[8,22] - 50.51967006943518 x[8,23]
   - service_time[8] - start_service[8] + start_service[23] >= 0
 LowerBoundRoutingVehicle[9]: - 19.18364200211251 x[9,1]
   - 54.36631077142511 x[9,2] - 88.24300056649358 x[9,3]
   - 170.8811094505974 x[9,4] - 188.3286957077436 x[9,5]
   - 121.2178349668346 x[9,6] - 111.3675253552535 x[9,7]
   - 111.0808809945302 x[9,8] - 19.18364200211251 x[9,9]
   - 41.55765266079151 x[9,10] - 178.5808613108654 x[9,11]
   - 54.36631077142511 x[9,12] - 88.24300056649358 x[9,13]
   - 170.8811094505974 x[9,14] - 188.3286957077436 x[9,15]
   - 121.2178349668346 x[9,16] - 111.3675253552535 x[9,17]
   - 111.0808809945302 x[9,18] - 19.18364200211251 x[9,19]
   - 41.55765266079151 x[9,20] - 178.5808613108654 x[9,21]
   - 19.18364200211251 x[9,22] - 19.18364200211251 x[9,23]
   - service_time[9] - start_service[9] + start_service[23] >= 0
 LowerBoundRoutingVehicle[10]: - 30.20368113554929 x[10,1]
   - 53.82881471707972 x[10,2] - 86.53530557529372 x[10,3]
   - 175.5580991805114 x[10,4] - 189.0567163667137 x[10,5]
   - 112.426397964868 x[10,6] - 101.7893802336698 x[10,7]
   - 116.204119177553 x[10,8] - 30.53761352735473 x[10,9]
   - 30.20368113554929 x[10,10] - 179.2533123265662 x[10,11]
   - 53.82881471707972 x[10,12] - 86.53530557529372 x[10,13]
   - 175.5580991805114 x[10,14] - 189.0567163667137 x[10,15]
   - 112.426397964868 x[10,16] - 101.7893802336698 x[10,17]
   - 116.204119177553 x[10,18] - 30.53761352735473 x[10,19]
   - 30.20368113554929 x[10,20] - 179.2533123265662 x[10,21]
   - 30.20368113554929 x[10,22] - 30.20368113554929 x[10,23]
   - service_time[10] - start_service[10] + start_service[23] >= 0
 LowerBoundRoutingVehicle[11]: - 87.39280480907955 x[11,1]
   - 98.93986906382338 x[11,2] - 98.01023108892625 x[11,3]
   - 113.7149142574987 x[11,4] - 97.19862159219289 x[11,5]
   - 141.5952749998395 x[11,6] - 143.1018813655535 x[11,7]
   - 92.50534847656779 x[11,8] - 110.3716985038984 x[11,9]
   - 122.0641886530359 x[11,10] - 87.39280480907955 x[11,11]
   - 98.93986906382338 x[11,12] - 98.01023108892625 x[11,13]
   - 113.7149142574987 x[11,14] - 97.19862159219289 x[11,15]
   - 141.5952749998395 x[11,16] - 143.1018813655535 x[11,17]
   - 92.50534847656779 x[11,18] - 110.3716985038984 x[11,19]
   - 122.0641886530359 x[11,20] - 87.39280480907955 x[11,21]
   - 87.39280480907955 x[11,22] - 87.39280480907955 x[11,23]
   - service_time[11] - start_service[11] + start_service[23] >= 0
 LowerBoundRoutingVehicle[12]: - 30.41031868393725 x[12,1]
   - 30.41031868393725 x[12,2] - 64.28712623170627 x[12,3]
   - 153.3699234172881 x[12,4] - 165.7204585938667 x[12,5]
   - 105.4062485753255 x[12,6] - 98.65582863528012 x[12,7]
   - 94.95089458566449 x[12,8] - 43.13963408960037 x[12,9]
   - 53.62217716869176 x[12,10] - 155.9223551889657 x[12,11]
   - 30.41031868393725 x[12,12] - 64.28712623170627 x[12,13]
   - 153.3699234172881 x[12,14] - 165.7204585938667 x[12,15]
   - 105.4062485753255 x[12,16] - 98.65582863528012 x[12,17]
   - 94.95089458566449 x[12,18] - 43.13963408960037 x[12,19]
   - 53.62217716869176 x[12,20] - 155.9223551889657 x[12,21]
   - 30.41031868393725 x[12,22] - 30.41031868393725 x[12,23]
   - service_time[12] - start_service[12] + start_service[23] >= 0
 LowerBoundRoutingVehicle[13]: - 46.01686331905368 x[13,1]
   - 48.68058159658984 x[13,2] - 46.01686331905368 x[13,3]
   - 142.8674615430835 x[13,4] - 149.1796912312963 x[13,5]
   - 100.7583785583689 x[13,6] - 98.15458752284347 x[13,7]
   - 88.56519872374558 x[13,8] - 61.40977924955241 x[13,9]
   - 70.72212339178934 x[13,10] - 139.3861725789521 x[13,11]
   - 48.68058159658984 x[13,12] - 46.01686331905368 x[13,13]
   - 142.8674615430835 x[13,14] - 149.1796912312963 x[13,15]
   - 100.7583785583689 x[13,16] - 98.15458752284347 x[13,17]
   - 88.56519872374558 x[13,18] - 61.40977924955241 x[13,19]
   - 70.72212339178934 x[13,20] - 139.3861725789521 x[13,21]
   - 46.01686331905368 x[13,22] - 46.01686331905368 x[13,23]
   - service_time[13] - start_service[13] + start_service[23] >= 0
 LowerBoundRoutingVehicle[14]: - 80.15594958799544 x[14,1]
   - 103.62429251323 x[14,2] - 108.7283752741418 x[14,3]
   - 80.15594958799544 x[14,4] - 126.2917516420279 x[14,5]
   - 160.7950591113668 x[14,6] - 160.0924775804411 x[14,7]
   - 80.71265957621593 x[14,8] - 109.9088018647144 x[14,9]
   - 125.6058307280653 x[14,10] - 120.9517694785829 x[14,11]
   - 103.62429251323 x[14,12] - 108.7283752741418 x[14,13]
   - 80.15594958799544 x[14,14] - 126.2917516420279 x[14,15]
   - 160.7950591113668 x[14,16] - 160.0924775804411 x[14,17]
   - 80.71265957621593 x[14,18] - 109.9088018647144 x[14,19]
   - 125.6058307280653 x[14,20] - 120.9517694785829 x[14,21]
   - 80.15594958799544 x[14,22] - 80.15594958799544 x[14,23]
   - service_time[14] - start_service[14] + start_service[23] >= 0
 LowerBoundRoutingVehicle[15]: - 92.13639936025531 x[15,1]
   - 103.9943779175486 x[15,2] - 103.0601551900947 x[15,3]
   - 114.311301869768 x[15,4] - 92.13639936025531 x[15,5]
   - 146.1263760277999 x[15,6] - 147.7848755785375 x[15,7]
   - 96.38853111031936 x[15,8] - 115.3759383496008 x[15,9]
   - 127.1239981420077 x[15,10] - 92.45502704101713 x[15,11]
   - 103.9943779175486 x[15,12] - 103.0601551900947 x[15,13]
   - 114.311301869768 x[15,14] - 92.13639936025531 x[15,15]
   - 146.1263760277999 x[15,16] - 147.7848755785375 x[15,17]
   - 96.38853111031936 x[15,18] - 115.3759383496008 x[15,19]
   - 127.1239981420077 x[15,20] - 92.45502704101713 x[15,21]
   - 92.13639936025531 x[15,22] - 92.13639936025531 x[15,23]
   - service_time[15] - start_service[15] + start_service[23] >= 0
 LowerBoundRoutingVehicle[16]: - 67.28478961524529 x[16,1]
   - 68.53177764401744 x[16,2] - 79.49045226217734 x[16,3]
   - 173.666219084117 x[16,4] - 170.9779857728099 x[16,5]
   - 67.28478961524529 x[16,6] - 71.25263327961092 x[16,7]
   - 121.8684776726066 x[16,8] - 73.11668735370183 x[16,9]
   - 75.34528948517199 x[16,10] - 161.7032901936737 x[16,11]
   - 68.53177764401744 x[16,12] - 79.49045226217734 x[16,13]
   - 173.666219084117 x[16,14] - 170.9779857728099 x[16,15]
   - 67.28478961524529 x[16,16] - 71.25263327961092 x[16,17]
   - 121.8684776726066 x[16,18] - 73.11668735370183 x[16,19]
   - 75.34528948517199 x[16,20] - 161.7032901936737 x[16,21]
   - 67.28478961524529 x[16,22] - 67.28478961524529 x[16,23]
   - service_time[16] - start_service[16] + start_service[23] >= 0
 LowerBoundRoutingVehicle[17]: - 63.09070701716995 x[17,1]
   - 65.97544030204742 x[17,2] - 81.08074382472719 x[17,3]
   - 177.1577201512666 x[17,4] - 176.8305679216229 x[17,5]
   - 75.44671587768626 x[17,6] - 63.09070701716995 x[17,7]
   - 123.5452363545894 x[17,8] - 67.46046034019604 x[17,9]
   - 68.9023543520491 x[17,10] - 167.4039791574631 x[17,11]
   - 65.97544030204742 x[17,12] - 81.08074382472719 x[17,13]
   - 177.1577201512666 x[17,14] - 176.8305679216229 x[17,15]
   - 75.44671587768626 x[17,16] - 63.09070701716995 x[17,17]
   - 123.5452363545894 x[17,18] - 67.46046034019604 x[17,19]
   - 68.9023543520491 x[17,20] - 167.4039791574631 x[17,21]
   - 63.09070701716995 x[17,22] - 63.09070701716995 x[17,23]
   - service_time[17] - start_service[17] + start_service[23] >= 0
 LowerBoundRoutingVehicle[18]: - 50.51967006943518 x[18,1]
   - 74.84154320016656 x[18,2] - 84.06239197336407 x[18,3]
   - 110.3489390947762 x[18,4] - 138.0052604011395 x[18,5]
   - 138.6335972184168 x[18,6] - 136.1162733023242 x[18,7]
   - 50.51967006943518 x[18,8] - 79.7448529272075 x[18,9]
   - 95.88813024366713 x[18,10] - 129.3784832162121 x[18,11]
   - 74.84154320016656 x[18,12] - 84.06239197336407 x[18,13]
   - 110.3489390947762 x[18,14] - 138.0052604011395 x[18,15]
   - 138.6335972184168 x[18,16] - 136.1162733023242 x[18,17]
   - 50.51967006943518 x[18,18] - 79.7448529272075 x[18,19]
   - 95.88813024366713 x[18,20] - 129.3784832162121 x[18,21]
   - 50.51967006943518 x[18,22] - 50.51967006943518 x[18,23]
   - service_time[18] - start_service[18] + start_service[23] >= 0
 LowerBoundRoutingVehicle[19]: - 19.18364200211251 x[19,1]
   - 54.36631077142511 x[19,2] - 88.24300056649358 x[19,3]
   - 170.8811094505974 x[19,4] - 188.3286957077436 x[19,5]
   - 121.2178349668346 x[19,6] - 111.3675253552535 x[19,7]
   - 111.0808809945302 x[19,8] - 19.18364200211251 x[19,9]
   - 41.55765266079151 x[19,10] - 178.5808613108654 x[19,11]
   - 54.36631077142511 x[19,12] - 88.24300056649358 x[19,13]
   - 170.8811094505974 x[19,14] - 188.3286957077436 x[19,15]
   - 121.2178349668346 x[19,16] - 111.3675253552535 x[19,17]
   - 111.0808809945302 x[19,18] - 19.18364200211251 x[19,19]
   - 41.55765266079151 x[19,20] - 178.5808613108654 x[19,21]
   - 19.18364200211251 x[19,22] - 19.18364200211251 x[19,23]
   - service_time[19] - start_service[19] + start_service[23] >= 0
 LowerBoundRoutingVehicle[20]: - 30.20368113554929 x[20,1]
   - 53.82881471707972 x[20,2] - 86.53530557529372 x[20,3]
   - 175.5580991805114 x[20,4] - 189.0567163667137 x[20,5]
   - 112.426397964868 x[20,6] - 101.7893802336698 x[20,7]
   - 116.204119177553 x[20,8] - 30.53761352735473 x[20,9]
   - 30.20368113554929 x[20,10] - 179.2533123265662 x[20,11]
   - 53.82881471707972 x[20,12] - 86.53530557529372 x[20,13]
   - 175.5580991805114 x[20,14] - 189.0567163667137 x[20,15]
   - 112.426397964868 x[20,16] - 101.7893802336698 x[20,17]
   - 116.204119177553 x[20,18] - 30.53761352735473 x[20,19]
   - 30.20368113554929 x[20,20] - 179.2533123265662 x[20,21]
   - 30.20368113554929 x[20,22] - 30.20368113554929 x[20,23]
   - service_time[20] - start_service[20] + start_service[23] >= 0
 LowerBoundRoutingVehicle[21]: - 87.39280480907955 x[21,1]
   - 98.93986906382338 x[21,2] - 98.01023108892625 x[21,3]
   - 113.7149142574987 x[21,4] - 97.19862159219289 x[21,5]
   - 141.5952749998395 x[21,6] - 143.1018813655535 x[21,7]
   - 92.50534847656779 x[21,8] - 110.3716985038984 x[21,9]
   - 122.0641886530359 x[21,10] - 87.39280480907955 x[21,11]
   - 98.93986906382338 x[21,12] - 98.01023108892625 x[21,13]
   - 113.7149142574987 x[21,14] - 97.19862159219289 x[21,15]
   - 141.5952749998395 x[21,16] - 143.1018813655535 x[21,17]
   - 92.50534847656779 x[21,18] - 110.3716985038984 x[21,19]
   - 122.0641886530359 x[21,20] - 87.39280480907955 x[21,21]
   - 87.39280480907955 x[21,22] - 87.39280480907955 x[21,23]
   - service_time[21] - start_service[21] + start_service[23] >= 0
 LowerBoundRoutingVehicle[22]: - 60.8206373678745 x[22,2]
   - 92.03372663810735 x[22,3] - 160.3118991759909 x[22,4]
   - 184.2727987205106 x[22,5] - 134.5695792304906 x[22,6]
   - 126.1814140343399 x[22,7] - 101.0393401388704 x[22,8]
   - 38.36728400422502 x[22,9] - 60.40736227109858 x[22,10]
   - 174.7856096181591 x[22,11] - 60.8206373678745 x[22,12]
   - 92.03372663810735 x[22,13] - 160.3118991759909 x[22,14]
   - 184.2727987205106 x[22,15] - 134.5695792304906 x[22,16]
   - 126.1814140343399 x[22,17] - 101.0393401388704 x[22,18]
   - 38.36728400422502 x[22,19] - 60.40736227109858 x[22,20]
   - 174.7856096181591 x[22,21] - service_time[22] - start_service[22]
   + start_service[23] >= 0
 LowerBoundArrivalVehicle[1]: - 60.8206373678745 x[2,1]
   - 92.03372663810735 x[3,1] - 160.3118991759909 x[4,1]
   - 184.2727987205106 x[5,1] - 134.5695792304906 x[6,1]
   - 126.1814140343399 x[7,1] - 101.0393401388704 x[8,1]
   - 38.36728400422502 x[9,1] - 60.40736227109858 x[10,1]
   - 174.7856096181591 x[11,1] - 60.8206373678745 x[12,1]
   - 92.03372663810735 x[13,1] - 160.3118991759909 x[14,1]
   - 184.2727987205106 x[15,1] - 134.5695792304906 x[16,1]
   - 126.1814140343399 x[17,1] - 101.0393401388704 x[18,1]
   - 38.36728400422502 x[19,1] - 60.40736227109858 x[20,1]
   - 174.7856096181591 x[21,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 30.41031868393725 x[0,2]
   - 30.41031868393725 x[1,2] - 30.41031868393725 x[2,2]
   - 64.28712623170627 x[3,2] - 153.3699234172881 x[4,2]
   - 165.7204585938667 x[5,2] - 105.4062485753255 x[6,2]
   - 98.65582863528012 x[7,2] - 94.95089458566449 x[8,2]
   - 43.13963408960037 x[9,2] - 53.62217716869176 x[10,2]
   - 155.9223551889657 x[11,2] - 30.41031868393725 x[12,2]
   - 64.28712623170627 x[13,2] - 153.3699234172881 x[14,2]
   - 165.7204585938667 x[15,2] - 105.4062485753255 x[16,2]
   - 98.65582863528012 x[17,2] - 94.95089458566449 x[18,2]
   - 43.13963408960037 x[19,2] - 53.62217716869176 x[20,2]
   - 155.9223551889657 x[21,2] - 30.41031868393725 x[22,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 46.01686331905368 x[0,3]
   - 46.01686331905368 x[1,3] - 48.68058159658984 x[2,3]
   - 46.01686331905368 x[3,3] - 142.8674615430835 x[4,3]
   - 149.1796912312963 x[5,3] - 100.7583785583689 x[6,3]
   - 98.15458752284347 x[7,3] - 88.56519872374558 x[8,3]
   - 61.40977924955241 x[9,3] - 70.72212339178934 x[10,3]
   - 139.3861725789521 x[11,3] - 48.68058159658984 x[12,3]
   - 46.01686331905368 x[13,3] - 142.8674615430835 x[14,3]
   - 149.1796912312963 x[15,3] - 100.7583785583689 x[16,3]
   - 98.15458752284347 x[17,3] - 88.56519872374558 x[18,3]
   - 61.40977924955241 x[19,3] - 70.72212339178934 x[20,3]
   - 139.3861725789521 x[21,3] - 46.01686331905368 x[22,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 80.15594958799544 x[0,4]
   - 80.15594958799544 x[1,4] - 103.62429251323 x[2,4]
   - 108.7283752741418 x[3,4] - 80.15594958799544 x[4,4]
   - 126.2917516420279 x[5,4] - 160.7950591113668 x[6,4]
   - 160.0924775804411 x[7,4] - 80.71265957621593 x[8,4]
   - 109.9088018647144 x[9,4] - 125.6058307280653 x[10,4]
   - 120.9517694785829 x[11,4] - 103.62429251323 x[12,4]
   - 108.7283752741418 x[13,4] - 80.15594958799544 x[14,4]
   - 126.2917516420279 x[15,4] - 160.7950591113668 x[16,4]
   - 160.0924775804411 x[17,4] - 80.71265957621593 x[18,4]
   - 109.9088018647144 x[19,4] - 125.6058307280653 x[20,4]
   - 120.9517694785829 x[21,4] - 80.15594958799544 x[22,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 92.13639936025531 x[0,5]
   - 92.13639936025531 x[1,5] - 103.9943779175486 x[2,5]
   - 103.0601551900947 x[3,5] - 114.311301869768 x[4,5]
   - 92.13639936025531 x[5,5] - 146.1263760277999 x[6,5]
   - 147.7848755785375 x[7,5] - 96.38853111031936 x[8,5]
   - 115.3759383496008 x[9,5] - 127.1239981420077 x[10,5]
   - 92.45502704101713 x[11,5] - 103.9943779175486 x[12,5]
   - 103.0601551900947 x[13,5] - 114.311301869768 x[14,5]
   - 92.13639936025531 x[15,5] - 146.1263760277999 x[16,5]
   - 147.7848755785375 x[17,5] - 96.38853111031936 x[18,5]
   - 115.3759383496008 x[19,5] - 127.1239981420077 x[20,5]
   - 92.45502704101713 x[21,5] - 92.13639936025531 x[22,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 67.28478961524529 x[0,6]
   - 67.28478961524529 x[1,6] - 68.53177764401744 x[2,6]
   - 79.49045226217734 x[3,6] - 173.666219084117 x[4,6]
   - 170.9779857728099 x[5,6] - 67.28478961524529 x[6,6]
   - 71.25263327961092 x[7,6] - 121.8684776726066 x[8,6]
   - 73.11668735370183 x[9,6] - 75.34528948517199 x[10,6]
   - 161.7032901936737 x[11,6] - 68.53177764401744 x[12,6]
   - 79.49045226217734 x[13,6] - 173.666219084117 x[14,6]
   - 170.9779857728099 x[15,6] - 67.28478961524529 x[16,6]
   - 71.25263327961092 x[17,6] - 121.8684776726066 x[18,6]
   - 73.11668735370183 x[19,6] - 75.34528948517199 x[20,6]
   - 161.7032901936737 x[21,6] - 67.28478961524529 x[22,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 63.09070701716995 x[0,7]
   - 63.09070701716995 x[1,7] - 65.97544030204742 x[2,7]
   - 81.08074382472719 x[3,7] - 177.1577201512666 x[4,7]
   - 176.8305679216229 x[5,7] - 75.44671587768626 x[6,7]
   - 63.09070701716995 x[7,7] - 123.5452363545894 x[8,7]
   - 67.46046034019604 x[9,7] - 68.9023543520491 x[10,7]
   - 167.4039791574631 x[11,7] - 65.97544030204742 x[12,7]
   - 81.08074382472719 x[13,7] - 177.1577201512666 x[14,7]
   - 176.8305679216229 x[15,7] - 75.44671587768626 x[16,7]
   - 63.09070701716995 x[17,7] - 123.5452363545894 x[18,7]
   - 67.46046034019604 x[19,7] - 68.9023543520491 x[20,7]
   - 167.4039791574631 x[21,7] - 63.09070701716995 x[22,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 50.51967006943518 x[0,8]
   - 50.51967006943518 x[1,8] - 74.84154320016656 x[2,8]
   - 84.06239197336407 x[3,8] - 110.3489390947762 x[4,8]
   - 138.0052604011395 x[5,8] - 138.6335972184168 x[6,8]
   - 136.1162733023242 x[7,8] - 50.51967006943518 x[8,8]
   - 79.7448529272075 x[9,8] - 95.88813024366713 x[10,8]
   - 129.3784832162121 x[11,8] - 74.84154320016656 x[12,8]
   - 84.06239197336407 x[13,8] - 110.3489390947762 x[14,8]
   - 138.0052604011395 x[15,8] - 138.6335972184168 x[16,8]
   - 136.1162733023242 x[17,8] - 50.51967006943518 x[18,8]
   - 79.7448529272075 x[19,8] - 95.88813024366713 x[20,8]
   - 129.3784832162121 x[21,8] - 50.51967006943518 x[22,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 19.18364200211251 x[0,9]
   - 19.18364200211251 x[1,9] - 54.36631077142511 x[2,9]
   - 88.24300056649358 x[3,9] - 170.8811094505974 x[4,9]
   - 188.3286957077436 x[5,9] - 121.2178349668346 x[6,9]
   - 111.3675253552535 x[7,9] - 111.0808809945302 x[8,9]
   - 19.18364200211251 x[9,9] - 41.55765266079151 x[10,9]
   - 178.5808613108654 x[11,9] - 54.36631077142511 x[12,9]
   - 88.24300056649358 x[13,9] - 170.8811094505974 x[14,9]
   - 188.3286957077436 x[15,9] - 121.2178349668346 x[16,9]
   - 111.3675253552535 x[17,9] - 111.0808809945302 x[18,9]
   - 19.18364200211251 x[19,9] - 41.55765266079151 x[20,9]
   - 178.5808613108654 x[21,9] - 19.18364200211251 x[22,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 30.20368113554929 x[0,10]
   - 30.20368113554929 x[1,10] - 53.82881471707972 x[2,10]
   - 86.53530557529372 x[3,10] - 175.5580991805114 x[4,10]
   - 189.0567163667137 x[5,10] - 112.426397964868 x[6,10]
   - 101.7893802336698 x[7,10] - 116.204119177553 x[8,10]
   - 30.53761352735473 x[9,10] - 30.20368113554929 x[10,10]
   - 179.2533123265662 x[11,10] - 53.82881471707972 x[12,10]
   - 86.53530557529372 x[13,10] - 175.5580991805114 x[14,10]
   - 189.0567163667137 x[15,10] - 112.426397964868 x[16,10]
   - 101.7893802336698 x[17,10] - 116.204119177553 x[18,10]
   - 30.53761352735473 x[19,10] - 30.20368113554929 x[20,10]
   - 179.2533123265662 x[21,10] - 30.20368113554929 x[22,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 87.39280480907955 x[0,11]
   - 87.39280480907955 x[1,11] - 98.93986906382338 x[2,11]
   - 98.01023108892625 x[3,11] - 113.7149142574987 x[4,11]
   - 97.19862159219289 x[5,11] - 141.5952749998395 x[6,11]
   - 143.1018813655535 x[7,11] - 92.50534847656779 x[8,11]
   - 110.3716985038984 x[9,11] - 122.0641886530359 x[10,11]
   - 87.39280480907955 x[11,11] - 98.93986906382338 x[12,11]
   - 98.01023108892625 x[13,11] - 113.7149142574987 x[14,11]
   - 97.19862159219289 x[15,11] - 141.5952749998395 x[16,11]
   - 143.1018813655535 x[17,11] - 92.50534847656779 x[18,11]
   - 110.3716985038984 x[19,11] - 122.0641886530359 x[20,11]
   - 87.39280480907955 x[21,11] - 87.39280480907955 x[22,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 30.41031868393725 x[0,12]
   - 30.41031868393725 x[1,12] - 30.41031868393725 x[2,12]
   - 64.28712623170627 x[3,12] - 153.3699234172881 x[4,12]
   - 165.7204585938667 x[5,12] - 105.4062485753255 x[6,12]
   - 98.65582863528012 x[7,12] - 94.95089458566449 x[8,12]
   - 43.13963408960037 x[9,12] - 53.62217716869176 x[10,12]
   - 155.9223551889657 x[11,12] - 30.41031868393725 x[12,12]
   - 64.28712623170627 x[13,12] - 153.3699234172881 x[14,12]
   - 165.7204585938667 x[15,12] - 105.4062485753255 x[16,12]
   - 98.65582863528012 x[17,12] - 94.95089458566449 x[18,12]
   - 43.13963408960037 x[19,12] - 53.62217716869176 x[20,12]
   - 155.9223551889657 x[21,12] - 30.41031868393725 x[22,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 46.01686331905368 x[0,13]
   - 46.01686331905368 x[1,13] - 48.68058159658984 x[2,13]
   - 46.01686331905368 x[3,13] - 142.8674615430835 x[4,13]
   - 149.1796912312963 x[5,13] - 100.7583785583689 x[6,13]
   - 98.15458752284347 x[7,13] - 88.56519872374558 x[8,13]
   - 61.40977924955241 x[9,13] - 70.72212339178934 x[10,13]
   - 139.3861725789521 x[11,13] - 48.68058159658984 x[12,13]
   - 46.01686331905368 x[13,13] - 142.8674615430835 x[14,13]
   - 149.1796912312963 x[15,13] - 100.7583785583689 x[16,13]
   - 98.15458752284347 x[17,13] - 88.56519872374558 x[18,13]
   - 61.40977924955241 x[19,13] - 70.72212339178934 x[20,13]
   - 139.3861725789521 x[21,13] - 46.01686331905368 x[22,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 80.15594958799544 x[0,14]
   - 80.15594958799544 x[1,14] - 103.62429251323 x[2,14]
   - 108.7283752741418 x[3,14] - 80.15594958799544 x[4,14]
   - 126.2917516420279 x[5,14] - 160.7950591113668 x[6,14]
   - 160.0924775804411 x[7,14] - 80.71265957621593 x[8,14]
   - 109.9088018647144 x[9,14] - 125.6058307280653 x[10,14]
   - 120.9517694785829 x[11,14] - 103.62429251323 x[12,14]
   - 108.7283752741418 x[13,14] - 80.15594958799544 x[14,14]
   - 126.2917516420279 x[15,14] - 160.7950591113668 x[16,14]
   - 160.0924775804411 x[17,14] - 80.71265957621593 x[18,14]
   - 109.9088018647144 x[19,14] - 125.6058307280653 x[20,14]
   - 120.9517694785829 x[21,14] - 80.15594958799544 x[22,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 92.13639936025531 x[0,15]
   - 92.13639936025531 x[1,15] - 103.9943779175486 x[2,15]
   - 103.0601551900947 x[3,15] - 114.311301869768 x[4,15]
   - 92.13639936025531 x[5,15] - 146.1263760277999 x[6,15]
   - 147.7848755785375 x[7,15] - 96.38853111031936 x[8,15]
   - 115.3759383496008 x[9,15] - 127.1239981420077 x[10,15]
   - 92.45502704101713 x[11,15] - 103.9943779175486 x[12,15]
   - 103.0601551900947 x[13,15] - 114.311301869768 x[14,15]
   - 92.13639936025531 x[15,15] - 146.1263760277999 x[16,15]
   - 147.7848755785375 x[17,15] - 96.38853111031936 x[18,15]
   - 115.3759383496008 x[19,15] - 127.1239981420077 x[20,15]
   - 92.45502704101713 x[21,15] - 92.13639936025531 x[22,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 67.28478961524529 x[0,16]
   - 67.28478961524529 x[1,16] - 68.53177764401744 x[2,16]
   - 79.49045226217734 x[3,16] - 173.666219084117 x[4,16]
   - 170.9779857728099 x[5,16] - 67.28478961524529 x[6,16]
   - 71.25263327961092 x[7,16] - 121.8684776726066 x[8,16]
   - 73.11668735370183 x[9,16] - 75.34528948517199 x[10,16]
   - 161.7032901936737 x[11,16] - 68.53177764401744 x[12,16]
   - 79.49045226217734 x[13,16] - 173.666219084117 x[14,16]
   - 170.9779857728099 x[15,16] - 67.28478961524529 x[16,16]
   - 71.25263327961092 x[17,16] - 121.8684776726066 x[18,16]
   - 73.11668735370183 x[19,16] - 75.34528948517199 x[20,16]
   - 161.7032901936737 x[21,16] - 67.28478961524529 x[22,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 63.09070701716995 x[0,17]
   - 63.09070701716995 x[1,17] - 65.97544030204742 x[2,17]
   - 81.08074382472719 x[3,17] - 177.1577201512666 x[4,17]
   - 176.8305679216229 x[5,17] - 75.44671587768626 x[6,17]
   - 63.09070701716995 x[7,17] - 123.5452363545894 x[8,17]
   - 67.46046034019604 x[9,17] - 68.9023543520491 x[10,17]
   - 167.4039791574631 x[11,17] - 65.97544030204742 x[12,17]
   - 81.08074382472719 x[13,17] - 177.1577201512666 x[14,17]
   - 176.8305679216229 x[15,17] - 75.44671587768626 x[16,17]
   - 63.09070701716995 x[17,17] - 123.5452363545894 x[18,17]
   - 67.46046034019604 x[19,17] - 68.9023543520491 x[20,17]
   - 167.4039791574631 x[21,17] - 63.09070701716995 x[22,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 50.51967006943518 x[0,18]
   - 50.51967006943518 x[1,18] - 74.84154320016656 x[2,18]
   - 84.06239197336407 x[3,18] - 110.3489390947762 x[4,18]
   - 138.0052604011395 x[5,18] - 138.6335972184168 x[6,18]
   - 136.1162733023242 x[7,18] - 50.51967006943518 x[8,18]
   - 79.7448529272075 x[9,18] - 95.88813024366713 x[10,18]
   - 129.3784832162121 x[11,18] - 74.84154320016656 x[12,18]
   - 84.06239197336407 x[13,18] - 110.3489390947762 x[14,18]
   - 138.0052604011395 x[15,18] - 138.6335972184168 x[16,18]
   - 136.1162733023242 x[17,18] - 50.51967006943518 x[18,18]
   - 79.7448529272075 x[19,18] - 95.88813024366713 x[20,18]
   - 129.3784832162121 x[21,18] - 50.51967006943518 x[22,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 19.18364200211251 x[0,19]
   - 19.18364200211251 x[1,19] - 54.36631077142511 x[2,19]
   - 88.24300056649358 x[3,19] - 170.8811094505974 x[4,19]
   - 188.3286957077436 x[5,19] - 121.2178349668346 x[6,19]
   - 111.3675253552535 x[7,19] - 111.0808809945302 x[8,19]
   - 19.18364200211251 x[9,19] - 41.55765266079151 x[10,19]
   - 178.5808613108654 x[11,19] - 54.36631077142511 x[12,19]
   - 88.24300056649358 x[13,19] - 170.8811094505974 x[14,19]
   - 188.3286957077436 x[15,19] - 121.2178349668346 x[16,19]
   - 111.3675253552535 x[17,19] - 111.0808809945302 x[18,19]
   - 19.18364200211251 x[19,19] - 41.55765266079151 x[20,19]
   - 178.5808613108654 x[21,19] - 19.18364200211251 x[22,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 30.20368113554929 x[0,20]
   - 30.20368113554929 x[1,20] - 53.82881471707972 x[2,20]
   - 86.53530557529372 x[3,20] - 175.5580991805114 x[4,20]
   - 189.0567163667137 x[5,20] - 112.426397964868 x[6,20]
   - 101.7893802336698 x[7,20] - 116.204119177553 x[8,20]
   - 30.53761352735473 x[9,20] - 30.20368113554929 x[10,20]
   - 179.2533123265662 x[11,20] - 53.82881471707972 x[12,20]
   - 86.53530557529372 x[13,20] - 175.5580991805114 x[14,20]
   - 189.0567163667137 x[15,20] - 112.426397964868 x[16,20]
   - 101.7893802336698 x[17,20] - 116.204119177553 x[18,20]
   - 30.53761352735473 x[19,20] - 30.20368113554929 x[20,20]
   - 179.2533123265662 x[21,20] - 30.20368113554929 x[22,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 87.39280480907955 x[0,21]
   - 87.39280480907955 x[1,21] - 98.93986906382338 x[2,21]
   - 98.01023108892625 x[3,21] - 113.7149142574987 x[4,21]
   - 97.19862159219289 x[5,21] - 141.5952749998395 x[6,21]
   - 143.1018813655535 x[7,21] - 92.50534847656779 x[8,21]
   - 110.3716985038984 x[9,21] - 122.0641886530359 x[10,21]
   - 87.39280480907955 x[11,21] - 98.93986906382338 x[12,21]
   - 98.01023108892625 x[13,21] - 113.7149142574987 x[14,21]
   - 97.19862159219289 x[15,21] - 141.5952749998395 x[16,21]
   - 143.1018813655535 x[17,21] - 92.50534847656779 x[18,21]
   - 110.3716985038984 x[19,21] - 122.0641886530359 x[20,21]
   - 87.39280480907955 x[21,21] - 87.39280480907955 x[22,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 60.8206373678745 x[2,22]
   - 92.03372663810735 x[3,22] - 160.3118991759909 x[4,22]
   - 184.2727987205106 x[5,22] - 134.5695792304906 x[6,22]
   - 126.1814140343399 x[7,22] - 101.0393401388704 x[8,22]
   - 38.36728400422502 x[9,22] - 60.40736227109858 x[10,22]
   - 174.7856096181591 x[11,22] - 60.8206373678745 x[12,22]
   - 92.03372663810735 x[13,22] - 160.3118991759909 x[14,22]
   - 184.2727987205106 x[15,22] - 134.5695792304906 x[16,22]
   - 126.1814140343399 x[17,22] - 101.0393401388704 x[18,22]
   - 38.36728400422502 x[19,22] - 60.40736227109858 x[20,22]
   - 174.7856096181591 x[21,22] - start_service[0] + start_service[22]
   >= 0
 LowerBoundArrivalVehicle[23]: - 60.8206373678745 x[2,23]
   - 92.03372663810735 x[3,23] - 160.3118991759909 x[4,23]
   - 184.2727987205106 x[5,23] - 134.5695792304906 x[6,23]
   - 126.1814140343399 x[7,23] - 101.0393401388704 x[8,23]
   - 38.36728400422502 x[9,23] - 60.40736227109858 x[10,23]
   - 174.7856096181591 x[11,23] - 60.8206373678745 x[12,23]
   - 92.03372663810735 x[13,23] - 160.3118991759909 x[14,23]
   - 184.2727987205106 x[15,23] - 134.5695792304906 x[16,23]
   - 126.1814140343399 x[17,23] - 101.0393401388704 x[18,23]
   - 38.36728400422502 x[19,23] - 60.40736227109858 x[20,23]
   - 174.7856096181591 x[21,23] - start_service[0] + start_service[23]
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
