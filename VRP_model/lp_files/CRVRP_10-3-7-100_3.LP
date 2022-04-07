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
   + start_service[2] >= -999915.2024796238
 NodeArrivalVehicles[0,3]: - 999999 x[0,3] - start_service[0]
   + start_service[3] >= -999928.4369415202
 NodeArrivalVehicles[0,4]: - 999999 x[0,4] - start_service[0]
   + start_service[4] >= -999890.901035983
 NodeArrivalVehicles[0,5]: - 999999 x[0,5] - start_service[0]
   + start_service[5] >= -999960.2532812204
 NodeArrivalVehicles[0,6]: - 999999 x[0,6] - start_service[0]
   + start_service[6] >= -999955.5912513951
 NodeArrivalVehicles[0,7]: - 999999 x[0,7] - start_service[0]
   + start_service[7] >= -999952.271273898
 NodeArrivalVehicles[0,8]: - 999999 x[0,8] - start_service[0]
   + start_service[8] >= -999915.0779296518
 NodeArrivalVehicles[0,9]: - 999999 x[0,9] - start_service[0]
   + start_service[9] >= -999960.7764686092
 NodeArrivalVehicles[0,10]: - 999999 x[0,10] - start_service[0]
   + start_service[10] >= -999970.4639211913
 NodeArrivalVehicles[0,11]: - 999999 x[0,11] - start_service[0]
   + start_service[11] >= -999909.1632951815
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999915.2024796238
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999928.4369415202
 NodeArrivalVehicles[0,14]: - 999999 x[0,14] - start_service[0]
   + start_service[14] >= -999890.901035983
 NodeArrivalVehicles[0,15]: - 999999 x[0,15] - start_service[0]
   + start_service[15] >= -999960.2532812204
 NodeArrivalVehicles[0,16]: - 999999 x[0,16] - start_service[0]
   + start_service[16] >= -999955.5912513951
 NodeArrivalVehicles[0,17]: - 999999 x[0,17] - start_service[0]
   + start_service[17] >= -999952.271273898
 NodeArrivalVehicles[0,18]: - 999999 x[0,18] - start_service[0]
   + start_service[18] >= -999915.0779296518
 NodeArrivalVehicles[0,19]: - 999999 x[0,19] - start_service[0]
   + start_service[19] >= -999960.7764686092
 NodeArrivalVehicles[0,20]: - 999999 x[0,20] - start_service[0]
   + start_service[20] >= -999970.4639211913
 NodeArrivalVehicles[0,21]: - 999999 x[0,21] - start_service[0]
   + start_service[21] >= -999909.1632951815
 NodeArrivalVehicles[0,22]: - 999999 x[0,22] - start_service[0]
   + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999 x[0,23] - start_service[0]
   + start_service[23] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
   + start_service[2] >= -999915.2024796238
 NodeArrivalVehicles[1,3]: - 999999 x[1,3] - start_service[1]
   + start_service[3] >= -999928.4369415202
 NodeArrivalVehicles[1,4]: - 999999 x[1,4] - start_service[1]
   + start_service[4] >= -999890.901035983
 NodeArrivalVehicles[1,5]: - 999999 x[1,5] - start_service[1]
   + start_service[5] >= -999960.2532812204
 NodeArrivalVehicles[1,6]: - 999999 x[1,6] - start_service[1]
   + start_service[6] >= -999955.5912513951
 NodeArrivalVehicles[1,7]: - 999999 x[1,7] - start_service[1]
   + start_service[7] >= -999952.271273898
 NodeArrivalVehicles[1,8]: - 999999 x[1,8] - start_service[1]
   + start_service[8] >= -999915.0779296518
 NodeArrivalVehicles[1,9]: - 999999 x[1,9] - start_service[1]
   + start_service[9] >= -999960.7764686092
 NodeArrivalVehicles[1,10]: - 999999 x[1,10] - start_service[1]
   + start_service[10] >= -999970.4639211913
 NodeArrivalVehicles[1,11]: - 999999 x[1,11] - start_service[1]
   + start_service[11] >= -999909.1632951815
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999915.2024796238
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999928.4369415202
 NodeArrivalVehicles[1,14]: - 999999 x[1,14] - start_service[1]
   + start_service[14] >= -999890.901035983
 NodeArrivalVehicles[1,15]: - 999999 x[1,15] - start_service[1]
   + start_service[15] >= -999960.2532812204
 NodeArrivalVehicles[1,16]: - 999999 x[1,16] - start_service[1]
   + start_service[16] >= -999955.5912513951
 NodeArrivalVehicles[1,17]: - 999999 x[1,17] - start_service[1]
   + start_service[17] >= -999952.271273898
 NodeArrivalVehicles[1,18]: - 999999 x[1,18] - start_service[1]
   + start_service[18] >= -999915.0779296518
 NodeArrivalVehicles[1,19]: - 999999 x[1,19] - start_service[1]
   + start_service[19] >= -999960.7764686092
 NodeArrivalVehicles[1,20]: - 999999 x[1,20] - start_service[1]
   + start_service[20] >= -999970.4639211913
 NodeArrivalVehicles[1,21]: - 999999 x[1,21] - start_service[1]
   + start_service[21] >= -999909.1632951815
 NodeArrivalVehicles[1,22]: - 999999 x[1,22] - start_service[1]
   + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999 x[1,23] - start_service[1]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999915.2024796238
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 999999 x[2,3] - start_service[2]
   + start_service[3] >= -999985.7650834119
 NodeArrivalVehicles[2,4]: - 999999 x[2,4] - start_service[2]
   + start_service[4] >= -999961.403612743
 NodeArrivalVehicles[2,5]: - 999999 x[2,5] - start_service[2]
   + start_service[5] >= -999953.4152526929
 NodeArrivalVehicles[2,6]: - 999999 x[2,6] - start_service[2]
   + start_service[6] >= -999926.3485351843
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999952.1869815146
 NodeArrivalVehicles[2,8]: - 999999 x[2,8] - start_service[2]
   + start_service[8] >= -999957.5030249554
 NodeArrivalVehicles[2,9]: - 999999 x[2,9] - start_service[2]
   + start_service[9] >= -999927.6383894689
 NodeArrivalVehicles[2,10]: - 999999 x[2,10] - start_service[2]
   + start_service[10] >= -999942.098220245
 NodeArrivalVehicles[2,11]: - 999999 x[2,11] - start_service[2]
   + start_service[11] >= -999968.3063737304
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - start_service[2]
   + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999985.7650834119
 NodeArrivalVehicles[2,14]: - 999999 x[2,14] - start_service[2]
   + start_service[14] >= -999961.403612743
 NodeArrivalVehicles[2,15]: - 999999 x[2,15] - start_service[2]
   + start_service[15] >= -999953.4152526929
 NodeArrivalVehicles[2,16]: - 999999 x[2,16] - start_service[2]
   + start_service[16] >= -999926.3485351843
 NodeArrivalVehicles[2,17]: - 999999 x[2,17] - start_service[2]
   + start_service[17] >= -999952.1869815146
 NodeArrivalVehicles[2,18]: - 999999 x[2,18] - start_service[2]
   + start_service[18] >= -999957.5030249554
 NodeArrivalVehicles[2,19]: - 999999 x[2,19] - start_service[2]
   + start_service[19] >= -999927.6383894689
 NodeArrivalVehicles[2,20]: - 999999 x[2,20] - start_service[2]
   + start_service[20] >= -999942.098220245
 NodeArrivalVehicles[2,21]: - 999999 x[2,21] - start_service[2]
   + start_service[21] >= -999968.3063737304
 NodeArrivalVehicles[2,22]: - 999999 x[2,22] - start_service[2]
   + start_service[22] >= -999915.2024796238
 NodeArrivalVehicles[2,23]: - 999999 x[2,23] - start_service[2]
   + start_service[23] >= -999915.2024796238
 NodeArrivalVehicles[3,1]: - 999999 x[3,1] + start_service[1]
   - start_service[3] >= -999928.4369415202
 NodeArrivalVehicles[3,2]: - 999999 x[3,2] + start_service[2]
   - start_service[3] >= -999985.7650834119
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 999999 x[3,4] - start_service[3]
   + start_service[4] >= -999953.0826375335
 NodeArrivalVehicles[3,5]: - 999999 x[3,5] - start_service[3]
   + start_service[5] >= -999966.5641297009
 NodeArrivalVehicles[3,6]: - 999999 x[3,6] - start_service[3]
   + start_service[6] >= -999937.2267384126
 NodeArrivalVehicles[3,7]: - 999999 x[3,7] - start_service[3]
   + start_service[7] >= -999963.4959134692
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999958.5450983534
 NodeArrivalVehicles[3,9]: - 999999 x[3,9] - start_service[3]
   + start_service[9] >= -999939.1789229446
 NodeArrivalVehicles[3,10]: - 999999 x[3,10] - start_service[3]
   + start_service[10] >= -999955.1505151036
 NodeArrivalVehicles[3,11]: - 999999 x[3,11] - start_service[3]
   + start_service[11] >= -999965.4157057379
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - start_service[3]
   + start_service[12] >= -999985.7650834119
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 999999 x[3,14] - start_service[3]
   + start_service[14] >= -999953.0826375335
 NodeArrivalVehicles[3,15]: - 999999 x[3,15] - start_service[3]
   + start_service[15] >= -999966.5641297009
 NodeArrivalVehicles[3,16]: - 999999 x[3,16] - start_service[3]
   + start_service[16] >= -999937.2267384126
 NodeArrivalVehicles[3,17]: - 999999 x[3,17] - start_service[3]
   + start_service[17] >= -999963.4959134692
 NodeArrivalVehicles[3,18]: - 999999 x[3,18] - start_service[3]
   + start_service[18] >= -999958.5450983534
 NodeArrivalVehicles[3,19]: - 999999 x[3,19] - start_service[3]
   + start_service[19] >= -999939.1789229446
 NodeArrivalVehicles[3,20]: - 999999 x[3,20] - start_service[3]
   + start_service[20] >= -999955.1505151036
 NodeArrivalVehicles[3,21]: - 999999 x[3,21] - start_service[3]
   + start_service[21] >= -999965.4157057379
 NodeArrivalVehicles[3,22]: - 999999 x[3,22] - start_service[3]
   + start_service[22] >= -999928.4369415202
 NodeArrivalVehicles[3,23]: - 999999 x[3,23] - start_service[3]
   + start_service[23] >= -999928.4369415202
 NodeArrivalVehicles[4,1]: - 999999 x[4,1] + start_service[1]
   - start_service[4] >= -999890.901035983
 NodeArrivalVehicles[4,2]: - 999999 x[4,2] + start_service[2]
   - start_service[4] >= -999961.403612743
 NodeArrivalVehicles[4,3]: - 999999 x[4,3] + start_service[3]
   - start_service[4] >= -999953.0826375335
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 999999 x[4,5] - start_service[4]
   + start_service[5] >= -999924.4748369423
 NodeArrivalVehicles[4,6]: - 999999 x[4,6] - start_service[4]
   + start_service[6] >= -999917.2796655721
 NodeArrivalVehicles[4,7]: - 999999 x[4,7] - start_service[4]
   + start_service[7] >= -999936.6080027135
 NodeArrivalVehicles[4,8]: - 999999 x[4,8] - start_service[4]
   + start_service[8] >= -999968.3875979197
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999893.8315006682
 NodeArrivalVehicles[4,10]: - 999999 x[4,10] - start_service[4]
   + start_service[10] >= -999919.4256871791
 NodeArrivalVehicles[4,11]: - 999999 x[4,11] - start_service[4]
   + start_service[11] >= -999933.3662834174
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - start_service[4]
   + start_service[12] >= -999961.403612743
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - start_service[4]
   + start_service[13] >= -999953.0826375335
 NodeArrivalVehicles[4,14]: - 999999 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 999999 x[4,15] - start_service[4]
   + start_service[15] >= -999924.4748369423
 NodeArrivalVehicles[4,16]: - 999999 x[4,16] - start_service[4]
   + start_service[16] >= -999917.2796655721
 NodeArrivalVehicles[4,17]: - 999999 x[4,17] - start_service[4]
   + start_service[17] >= -999936.6080027135
 NodeArrivalVehicles[4,18]: - 999999 x[4,18] - start_service[4]
   + start_service[18] >= -999968.3875979197
 NodeArrivalVehicles[4,19]: - 999999 x[4,19] - start_service[4]
   + start_service[19] >= -999893.8315006682
 NodeArrivalVehicles[4,20]: - 999999 x[4,20] - start_service[4]
   + start_service[20] >= -999919.4256871791
 NodeArrivalVehicles[4,21]: - 999999 x[4,21] - start_service[4]
   + start_service[21] >= -999933.3662834174
 NodeArrivalVehicles[4,22]: - 999999 x[4,22] - start_service[4]
   + start_service[22] >= -999890.901035983
 NodeArrivalVehicles[4,23]: - 999999 x[4,23] - start_service[4]
   + start_service[23] >= -999890.901035983
 NodeArrivalVehicles[5,1]: - 999999 x[5,1] + start_service[1]
   - start_service[5] >= -999960.2532812204
 NodeArrivalVehicles[5,2]: - 999999 x[5,2] + start_service[2]
   - start_service[5] >= -999953.4152526929
 NodeArrivalVehicles[5,3]: - 999999 x[5,3] + start_service[3]
   - start_service[5] >= -999966.5641297009
 NodeArrivalVehicles[5,4]: - 999999 x[5,4] + start_service[4]
   - start_service[5] >= -999924.4748369423
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 999999 x[5,6] - start_service[5]
   + start_service[6] >= -999953.4384806296
 NodeArrivalVehicles[5,7]: - 999999 x[5,7] - start_service[5]
   + start_service[7] >= -999973.2688131087
 NodeArrivalVehicles[5,8]: - 999999 x[5,8] - start_service[5]
   + start_service[8] >= -999941.9077472225
 NodeArrivalVehicles[5,9]: - 999999 x[5,9] - start_service[5]
   + start_service[9] >= -999965.8333693258
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999982.2907259347
 NodeArrivalVehicles[5,11]: - 999999 x[5,11] - start_service[5]
   + start_service[11] >= -999946.1993118351
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - start_service[5]
   + start_service[12] >= -999953.4152526929
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - start_service[5]
   + start_service[13] >= -999966.5641297009
 NodeArrivalVehicles[5,14]: - 999999 x[5,14] - start_service[5]
   + start_service[14] >= -999924.4748369423
 NodeArrivalVehicles[5,15]: - 999999 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 999999 x[5,16] - start_service[5]
   + start_service[16] >= -999953.4384806296
 NodeArrivalVehicles[5,17]: - 999999 x[5,17] - start_service[5]
   + start_service[17] >= -999973.2688131087
 NodeArrivalVehicles[5,18]: - 999999 x[5,18] - start_service[5]
   + start_service[18] >= -999941.9077472225
 NodeArrivalVehicles[5,19]: - 999999 x[5,19] - start_service[5]
   + start_service[19] >= -999965.8333693258
 NodeArrivalVehicles[5,20]: - 999999 x[5,20] - start_service[5]
   + start_service[20] >= -999982.2907259347
 NodeArrivalVehicles[5,21]: - 999999 x[5,21] - start_service[5]
   + start_service[21] >= -999946.1993118351
 NodeArrivalVehicles[5,22]: - 999999 x[5,22] - start_service[5]
   + start_service[22] >= -999960.2532812204
 NodeArrivalVehicles[5,23]: - 999999 x[5,23] - start_service[5]
   + start_service[23] >= -999960.2532812204
 NodeArrivalVehicles[6,1]: - 999999 x[6,1] + start_service[1]
   - start_service[6] >= -999955.5912513951
 NodeArrivalVehicles[6,2]: - 999999 x[6,2] + start_service[2]
   - start_service[6] >= -999926.3485351843
 NodeArrivalVehicles[6,3]: - 999999 x[6,3] + start_service[3]
   - start_service[6] >= -999937.2267384126
 NodeArrivalVehicles[6,4]: - 999999 x[6,4] + start_service[4]
   - start_service[6] >= -999917.2796655721
 NodeArrivalVehicles[6,5]: - 999999 x[6,5] + start_service[5]
   - start_service[6] >= -999953.4384806296
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 999999 x[6,7] - start_service[6]
   + start_service[7] >= -999972.6979548086
 NodeArrivalVehicles[6,8]: - 999999 x[6,8] - start_service[6]
   + start_service[8] >= -999946.9351560561
 NodeArrivalVehicles[6,9]: - 999999 x[6,9] - start_service[6]
   + start_service[9] >= -999928.9293963189
 NodeArrivalVehicles[6,10]: - 999999 x[6,10] - start_service[6]
   + start_service[10] >= -999968.6558211173
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999906.2176650148
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - start_service[6]
   + start_service[12] >= -999926.3485351843
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - start_service[6]
   + start_service[13] >= -999937.2267384126
 NodeArrivalVehicles[6,14]: - 999999 x[6,14] - start_service[6]
   + start_service[14] >= -999917.2796655721
 NodeArrivalVehicles[6,15]: - 999999 x[6,15] - start_service[6]
   + start_service[15] >= -999953.4384806296
 NodeArrivalVehicles[6,16]: - 999999 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 999999 x[6,17] - start_service[6]
   + start_service[17] >= -999972.6979548086
 NodeArrivalVehicles[6,18]: - 999999 x[6,18] - start_service[6]
   + start_service[18] >= -999946.9351560561
 NodeArrivalVehicles[6,19]: - 999999 x[6,19] - start_service[6]
   + start_service[19] >= -999928.9293963189
 NodeArrivalVehicles[6,20]: - 999999 x[6,20] - start_service[6]
   + start_service[20] >= -999968.6558211173
 NodeArrivalVehicles[6,21]: - 999999 x[6,21] - start_service[6]
   + start_service[21] >= -999906.2176650148
 NodeArrivalVehicles[6,22]: - 999999 x[6,22] - start_service[6]
   + start_service[22] >= -999955.5912513951
 NodeArrivalVehicles[6,23]: - 999999 x[6,23] - start_service[6]
   + start_service[23] >= -999955.5912513951
 NodeArrivalVehicles[7,1]: - 999999 x[7,1] + start_service[1]
   - start_service[7] >= -999952.271273898
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999952.1869815146
 NodeArrivalVehicles[7,3]: - 999999 x[7,3] + start_service[3]
   - start_service[7] >= -999963.4959134692
 NodeArrivalVehicles[7,4]: - 999999 x[7,4] + start_service[4]
   - start_service[7] >= -999936.6080027135
 NodeArrivalVehicles[7,5]: - 999999 x[7,5] + start_service[5]
   - start_service[7] >= -999973.2688131087
 NodeArrivalVehicles[7,6]: - 999999 x[7,6] + start_service[6]
   - start_service[7] >= -999972.6979548086
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 999999 x[7,8] - start_service[7]
   + start_service[8] >= -999961.7262354469
 NodeArrivalVehicles[7,9]: - 999999 x[7,9] - start_service[7]
   + start_service[9] >= -999941.5599706656
 NodeArrivalVehicles[7,10]: - 999999 x[7,10] - start_service[7]
   + start_service[10] >= -999979.6262282272
 NodeArrivalVehicles[7,11]: - 999999 x[7,11] - start_service[7]
   + start_service[11] >= -999932.1978989958
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999952.1869815146
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - start_service[7]
   + start_service[13] >= -999963.4959134692
 NodeArrivalVehicles[7,14]: - 999999 x[7,14] - start_service[7]
   + start_service[14] >= -999936.6080027135
 NodeArrivalVehicles[7,15]: - 999999 x[7,15] - start_service[7]
   + start_service[15] >= -999973.2688131087
 NodeArrivalVehicles[7,16]: - 999999 x[7,16] - start_service[7]
   + start_service[16] >= -999972.6979548086
 NodeArrivalVehicles[7,17]: - 999999 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 999999 x[7,18] - start_service[7]
   + start_service[18] >= -999961.7262354469
 NodeArrivalVehicles[7,19]: - 999999 x[7,19] - start_service[7]
   + start_service[19] >= -999941.5599706656
 NodeArrivalVehicles[7,20]: - 999999 x[7,20] - start_service[7]
   + start_service[20] >= -999979.6262282272
 NodeArrivalVehicles[7,21]: - 999999 x[7,21] - start_service[7]
   + start_service[21] >= -999932.1978989958
 NodeArrivalVehicles[7,22]: - 999999 x[7,22] - start_service[7]
   + start_service[22] >= -999952.271273898
 NodeArrivalVehicles[7,23]: - 999999 x[7,23] - start_service[7]
   + start_service[23] >= -999952.271273898
 NodeArrivalVehicles[8,1]: - 999999 x[8,1] + start_service[1]
   - start_service[8] >= -999915.0779296518
 NodeArrivalVehicles[8,2]: - 999999 x[8,2] + start_service[2]
   - start_service[8] >= -999957.5030249554
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999958.5450983534
 NodeArrivalVehicles[8,4]: - 999999 x[8,4] + start_service[4]
   - start_service[8] >= -999968.3875979197
 NodeArrivalVehicles[8,5]: - 999999 x[8,5] + start_service[5]
   - start_service[8] >= -999941.9077472225
 NodeArrivalVehicles[8,6]: - 999999 x[8,6] + start_service[6]
   - start_service[8] >= -999946.9351560561
 NodeArrivalVehicles[8,7]: - 999999 x[8,7] + start_service[7]
   - start_service[8] >= -999961.7262354469
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 999999 x[8,9] - start_service[8]
   + start_service[9] >= -999908.812105495
 NodeArrivalVehicles[8,10]: - 999999 x[8,10] - start_service[8]
   + start_service[10] >= -999942.5652146442
 NodeArrivalVehicles[8,11]: - 999999 x[8,11] - start_service[8]
   + start_service[11] >= -999927.2610558096
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - start_service[8]
   + start_service[12] >= -999957.5030249554
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - start_service[8]
   + start_service[13] >= -999958.5450983534
 NodeArrivalVehicles[8,14]: - 999999 x[8,14] - start_service[8]
   + start_service[14] >= -999968.3875979197
 NodeArrivalVehicles[8,15]: - 999999 x[8,15] - start_service[8]
   + start_service[15] >= -999941.9077472225
 NodeArrivalVehicles[8,16]: - 999999 x[8,16] - start_service[8]
   + start_service[16] >= -999946.9351560561
 NodeArrivalVehicles[8,17]: - 999999 x[8,17] - start_service[8]
   + start_service[17] >= -999961.7262354469
 NodeArrivalVehicles[8,18]: - 999999 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 999999 x[8,19] - start_service[8]
   + start_service[19] >= -999908.812105495
 NodeArrivalVehicles[8,20]: - 999999 x[8,20] - start_service[8]
   + start_service[20] >= -999942.5652146442
 NodeArrivalVehicles[8,21]: - 999999 x[8,21] - start_service[8]
   + start_service[21] >= -999927.2610558096
 NodeArrivalVehicles[8,22]: - 999999 x[8,22] - start_service[8]
   + start_service[22] >= -999915.0779296518
 NodeArrivalVehicles[8,23]: - 999999 x[8,23] - start_service[8]
   + start_service[23] >= -999915.0779296518
 NodeArrivalVehicles[9,1]: - 999999 x[9,1] + start_service[1]
   - start_service[9] >= -999960.7764686092
 NodeArrivalVehicles[9,2]: - 999999 x[9,2] + start_service[2]
   - start_service[9] >= -999927.6383894689
 NodeArrivalVehicles[9,3]: - 999999 x[9,3] + start_service[3]
   - start_service[9] >= -999939.1789229446
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999893.8315006682
 NodeArrivalVehicles[9,5]: - 999999 x[9,5] + start_service[5]
   - start_service[9] >= -999965.8333693258
 NodeArrivalVehicles[9,6]: - 999999 x[9,6] + start_service[6]
   - start_service[9] >= -999928.9293963189
 NodeArrivalVehicles[9,7]: - 999999 x[9,7] + start_service[7]
   - start_service[9] >= -999941.5599706656
 NodeArrivalVehicles[9,8]: - 999999 x[9,8] + start_service[8]
   - start_service[9] >= -999908.812105495
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 999999 x[9,10] - start_service[9]
   + start_service[10] >= -999958.0904356453
 NodeArrivalVehicles[9,11]: - 999999 x[9,11] - start_service[9]
   + start_service[11] >= -999934.7740920822
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - start_service[9]
   + start_service[12] >= -999927.6383894689
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - start_service[9]
   + start_service[13] >= -999939.1789229446
 NodeArrivalVehicles[9,14]: - 999999 x[9,14] - start_service[9]
   + start_service[14] >= -999893.8315006682
 NodeArrivalVehicles[9,15]: - 999999 x[9,15] - start_service[9]
   + start_service[15] >= -999965.8333693258
 NodeArrivalVehicles[9,16]: - 999999 x[9,16] - start_service[9]
   + start_service[16] >= -999928.9293963189
 NodeArrivalVehicles[9,17]: - 999999 x[9,17] - start_service[9]
   + start_service[17] >= -999941.5599706656
 NodeArrivalVehicles[9,18]: - 999999 x[9,18] - start_service[9]
   + start_service[18] >= -999908.812105495
 NodeArrivalVehicles[9,19]: - 999999 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 999999 x[9,20] - start_service[9]
   + start_service[20] >= -999958.0904356453
 NodeArrivalVehicles[9,21]: - 999999 x[9,21] - start_service[9]
   + start_service[21] >= -999934.7740920822
 NodeArrivalVehicles[9,22]: - 999999 x[9,22] - start_service[9]
   + start_service[22] >= -999960.7764686092
 NodeArrivalVehicles[9,23]: - 999999 x[9,23] - start_service[9]
   + start_service[23] >= -999960.7764686092
 NodeArrivalVehicles[10,1]: - 999999 x[10,1] + start_service[1]
   - start_service[10] >= -999970.4639211913
 NodeArrivalVehicles[10,2]: - 999999 x[10,2] + start_service[2]
   - start_service[10] >= -999942.098220245
 NodeArrivalVehicles[10,3]: - 999999 x[10,3] + start_service[3]
   - start_service[10] >= -999955.1505151036
 NodeArrivalVehicles[10,4]: - 999999 x[10,4] + start_service[4]
   - start_service[10] >= -999919.4256871791
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999982.2907259347
 NodeArrivalVehicles[10,6]: - 999999 x[10,6] + start_service[6]
   - start_service[10] >= -999968.6558211173
 NodeArrivalVehicles[10,7]: - 999999 x[10,7] + start_service[7]
   - start_service[10] >= -999979.6262282272
 NodeArrivalVehicles[10,8]: - 999999 x[10,8] + start_service[8]
   - start_service[10] >= -999942.5652146442
 NodeArrivalVehicles[10,9]: - 999999 x[10,9] + start_service[9]
   - start_service[10] >= -999958.0904356453
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 999999 x[10,11] - start_service[10]
   + start_service[11] >= -999930.2057225219
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - start_service[10]
   + start_service[12] >= -999942.098220245
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - start_service[10]
   + start_service[13] >= -999955.1505151036
 NodeArrivalVehicles[10,14]: - 999999 x[10,14] - start_service[10]
   + start_service[14] >= -999919.4256871791
 NodeArrivalVehicles[10,15]: - 999999 x[10,15] - start_service[10]
   + start_service[15] >= -999982.2907259347
 NodeArrivalVehicles[10,16]: - 999999 x[10,16] - start_service[10]
   + start_service[16] >= -999968.6558211173
 NodeArrivalVehicles[10,17]: - 999999 x[10,17] - start_service[10]
   + start_service[17] >= -999979.6262282272
 NodeArrivalVehicles[10,18]: - 999999 x[10,18] - start_service[10]
   + start_service[18] >= -999942.5652146442
 NodeArrivalVehicles[10,19]: - 999999 x[10,19] - start_service[10]
   + start_service[19] >= -999958.0904356453
 NodeArrivalVehicles[10,20]: - 999999 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 999999 x[10,21] - start_service[10]
   + start_service[21] >= -999930.2057225219
 NodeArrivalVehicles[10,22]: - 999999 x[10,22] - start_service[10]
   + start_service[22] >= -999970.4639211913
 NodeArrivalVehicles[10,23]: - 999999 x[10,23] - start_service[10]
   + start_service[23] >= -999970.4639211913
 NodeArrivalVehicles[11,1]: - 999999 x[11,1] + start_service[1]
   - start_service[11] >= -999909.1632951815
 NodeArrivalVehicles[11,2]: - 999999 x[11,2] + start_service[2]
   - start_service[11] >= -999968.3063737304
 NodeArrivalVehicles[11,3]: - 999999 x[11,3] + start_service[3]
   - start_service[11] >= -999965.4157057379
 NodeArrivalVehicles[11,4]: - 999999 x[11,4] + start_service[4]
   - start_service[11] >= -999933.3662834174
 NodeArrivalVehicles[11,5]: - 999999 x[11,5] + start_service[5]
   - start_service[11] >= -999946.1993118351
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999906.2176650148
 NodeArrivalVehicles[11,7]: - 999999 x[11,7] + start_service[7]
   - start_service[11] >= -999932.1978989958
 NodeArrivalVehicles[11,8]: - 999999 x[11,8] + start_service[8]
   - start_service[11] >= -999927.2610558096
 NodeArrivalVehicles[11,9]: - 999999 x[11,9] + start_service[9]
   - start_service[11] >= -999934.7740920822
 NodeArrivalVehicles[11,10]: - 999999 x[11,10] + start_service[10]
   - start_service[11] >= -999930.2057225219
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - start_service[11]
   + start_service[12] >= -999968.3063737304
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - start_service[11]
   + start_service[13] >= -999965.4157057379
 NodeArrivalVehicles[11,14]: - 999999 x[11,14] - start_service[11]
   + start_service[14] >= -999933.3662834174
 NodeArrivalVehicles[11,15]: - 999999 x[11,15] - start_service[11]
   + start_service[15] >= -999946.1993118351
 NodeArrivalVehicles[11,16]: - 999999 x[11,16] - start_service[11]
   + start_service[16] >= -999906.2176650148
 NodeArrivalVehicles[11,17]: - 999999 x[11,17] - start_service[11]
   + start_service[17] >= -999932.1978989958
 NodeArrivalVehicles[11,18]: - 999999 x[11,18] - start_service[11]
   + start_service[18] >= -999927.2610558096
 NodeArrivalVehicles[11,19]: - 999999 x[11,19] - start_service[11]
   + start_service[19] >= -999934.7740920822
 NodeArrivalVehicles[11,20]: - 999999 x[11,20] - start_service[11]
   + start_service[20] >= -999930.2057225219
 NodeArrivalVehicles[11,21]: - 999999 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 999999 x[11,22] - start_service[11]
   + start_service[22] >= -999909.1632951815
 NodeArrivalVehicles[11,23]: - 999999 x[11,23] - start_service[11]
   + start_service[23] >= -999909.1632951815
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999915.2024796238
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] + start_service[2]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 999999 x[12,3] + start_service[3]
   - start_service[12] >= -999985.7650834119
 NodeArrivalVehicles[12,4]: - 999999 x[12,4] + start_service[4]
   - start_service[12] >= -999961.403612743
 NodeArrivalVehicles[12,5]: - 999999 x[12,5] + start_service[5]
   - start_service[12] >= -999953.4152526929
 NodeArrivalVehicles[12,6]: - 999999 x[12,6] + start_service[6]
   - start_service[12] >= -999926.3485351843
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999952.1869815146
 NodeArrivalVehicles[12,8]: - 999999 x[12,8] + start_service[8]
   - start_service[12] >= -999957.5030249554
 NodeArrivalVehicles[12,9]: - 999999 x[12,9] + start_service[9]
   - start_service[12] >= -999927.6383894689
 NodeArrivalVehicles[12,10]: - 999999 x[12,10] + start_service[10]
   - start_service[12] >= -999942.098220245
 NodeArrivalVehicles[12,11]: - 999999 x[12,11] + start_service[11]
   - start_service[12] >= -999968.3063737304
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999985.7650834119
 NodeArrivalVehicles[12,14]: - 999999 x[12,14] - start_service[12]
   + start_service[14] >= -999961.403612743
 NodeArrivalVehicles[12,15]: - 999999 x[12,15] - start_service[12]
   + start_service[15] >= -999953.4152526929
 NodeArrivalVehicles[12,16]: - 999999 x[12,16] - start_service[12]
   + start_service[16] >= -999926.3485351843
 NodeArrivalVehicles[12,17]: - 999999 x[12,17] - start_service[12]
   + start_service[17] >= -999952.1869815146
 NodeArrivalVehicles[12,18]: - 999999 x[12,18] - start_service[12]
   + start_service[18] >= -999957.5030249554
 NodeArrivalVehicles[12,19]: - 999999 x[12,19] - start_service[12]
   + start_service[19] >= -999927.6383894689
 NodeArrivalVehicles[12,20]: - 999999 x[12,20] - start_service[12]
   + start_service[20] >= -999942.098220245
 NodeArrivalVehicles[12,21]: - 999999 x[12,21] - start_service[12]
   + start_service[21] >= -999968.3063737304
 NodeArrivalVehicles[12,22]: - 999999 x[12,22] - start_service[12]
   + start_service[22] >= -999915.2024796238
 NodeArrivalVehicles[12,23]: - 999999 x[12,23] - start_service[12]
   + start_service[23] >= -999915.2024796238
 NodeArrivalVehicles[13,1]: - 999999 x[13,1] + start_service[1]
   - start_service[13] >= -999928.4369415202
 NodeArrivalVehicles[13,2]: - 999999 x[13,2] + start_service[2]
   - start_service[13] >= -999985.7650834119
 NodeArrivalVehicles[13,3]: - 999999 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 999999 x[13,4] + start_service[4]
   - start_service[13] >= -999953.0826375335
 NodeArrivalVehicles[13,5]: - 999999 x[13,5] + start_service[5]
   - start_service[13] >= -999966.5641297009
 NodeArrivalVehicles[13,6]: - 999999 x[13,6] + start_service[6]
   - start_service[13] >= -999937.2267384126
 NodeArrivalVehicles[13,7]: - 999999 x[13,7] + start_service[7]
   - start_service[13] >= -999963.4959134692
 NodeArrivalVehicles[13,8]: - 999999 x[13,8] + start_service[8]
   - start_service[13] >= -999958.5450983534
 NodeArrivalVehicles[13,9]: - 999999 x[13,9] + start_service[9]
   - start_service[13] >= -999939.1789229446
 NodeArrivalVehicles[13,10]: - 999999 x[13,10] + start_service[10]
   - start_service[13] >= -999955.1505151036
 NodeArrivalVehicles[13,11]: - 999999 x[13,11] + start_service[11]
   - start_service[13] >= -999965.4157057379
 NodeArrivalVehicles[13,12]: - 999999 x[13,12] + start_service[12]
   - start_service[13] >= -999985.7650834119
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 999999 x[13,14] - start_service[13]
   + start_service[14] >= -999953.0826375335
 NodeArrivalVehicles[13,15]: - 999999 x[13,15] - start_service[13]
   + start_service[15] >= -999966.5641297009
 NodeArrivalVehicles[13,16]: - 999999 x[13,16] - start_service[13]
   + start_service[16] >= -999937.2267384126
 NodeArrivalVehicles[13,17]: - 999999 x[13,17] - start_service[13]
   + start_service[17] >= -999963.4959134692
 NodeArrivalVehicles[13,18]: - 999999 x[13,18] - start_service[13]
   + start_service[18] >= -999958.5450983534
 NodeArrivalVehicles[13,19]: - 999999 x[13,19] - start_service[13]
   + start_service[19] >= -999939.1789229446
 NodeArrivalVehicles[13,20]: - 999999 x[13,20] - start_service[13]
   + start_service[20] >= -999955.1505151036
 NodeArrivalVehicles[13,21]: - 999999 x[13,21] - start_service[13]
   + start_service[21] >= -999965.4157057379
 NodeArrivalVehicles[13,22]: - 999999 x[13,22] - start_service[13]
   + start_service[22] >= -999928.4369415202
 NodeArrivalVehicles[13,23]: - 999999 x[13,23] - start_service[13]
   + start_service[23] >= -999928.4369415202
 NodeArrivalVehicles[14,1]: - 999999 x[14,1] + start_service[1]
   - start_service[14] >= -999890.901035983
 NodeArrivalVehicles[14,2]: - 999999 x[14,2] + start_service[2]
   - start_service[14] >= -999961.403612743
 NodeArrivalVehicles[14,3]: - 999999 x[14,3] + start_service[3]
   - start_service[14] >= -999953.0826375335
 NodeArrivalVehicles[14,4]: - 999999 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 999999 x[14,5] + start_service[5]
   - start_service[14] >= -999924.4748369423
 NodeArrivalVehicles[14,6]: - 999999 x[14,6] + start_service[6]
   - start_service[14] >= -999917.2796655721
 NodeArrivalVehicles[14,7]: - 999999 x[14,7] + start_service[7]
   - start_service[14] >= -999936.6080027135
 NodeArrivalVehicles[14,8]: - 999999 x[14,8] + start_service[8]
   - start_service[14] >= -999968.3875979197
 NodeArrivalVehicles[14,9]: - 999999 x[14,9] + start_service[9]
   - start_service[14] >= -999893.8315006682
 NodeArrivalVehicles[14,10]: - 999999 x[14,10] + start_service[10]
   - start_service[14] >= -999919.4256871791
 NodeArrivalVehicles[14,11]: - 999999 x[14,11] + start_service[11]
   - start_service[14] >= -999933.3662834174
 NodeArrivalVehicles[14,12]: - 999999 x[14,12] + start_service[12]
   - start_service[14] >= -999961.403612743
 NodeArrivalVehicles[14,13]: - 999999 x[14,13] + start_service[13]
   - start_service[14] >= -999953.0826375335
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 999999 x[14,15] - start_service[14]
   + start_service[15] >= -999924.4748369423
 NodeArrivalVehicles[14,16]: - 999999 x[14,16] - start_service[14]
   + start_service[16] >= -999917.2796655721
 NodeArrivalVehicles[14,17]: - 999999 x[14,17] - start_service[14]
   + start_service[17] >= -999936.6080027135
 NodeArrivalVehicles[14,18]: - 999999 x[14,18] - start_service[14]
   + start_service[18] >= -999968.3875979197
 NodeArrivalVehicles[14,19]: - 999999 x[14,19] - start_service[14]
   + start_service[19] >= -999893.8315006682
 NodeArrivalVehicles[14,20]: - 999999 x[14,20] - start_service[14]
   + start_service[20] >= -999919.4256871791
 NodeArrivalVehicles[14,21]: - 999999 x[14,21] - start_service[14]
   + start_service[21] >= -999933.3662834174
 NodeArrivalVehicles[14,22]: - 999999 x[14,22] - start_service[14]
   + start_service[22] >= -999890.901035983
 NodeArrivalVehicles[14,23]: - 999999 x[14,23] - start_service[14]
   + start_service[23] >= -999890.901035983
 NodeArrivalVehicles[15,1]: - 999999 x[15,1] + start_service[1]
   - start_service[15] >= -999960.2532812204
 NodeArrivalVehicles[15,2]: - 999999 x[15,2] + start_service[2]
   - start_service[15] >= -999953.4152526929
 NodeArrivalVehicles[15,3]: - 999999 x[15,3] + start_service[3]
   - start_service[15] >= -999966.5641297009
 NodeArrivalVehicles[15,4]: - 999999 x[15,4] + start_service[4]
   - start_service[15] >= -999924.4748369423
 NodeArrivalVehicles[15,5]: - 999999 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 999999 x[15,6] + start_service[6]
   - start_service[15] >= -999953.4384806296
 NodeArrivalVehicles[15,7]: - 999999 x[15,7] + start_service[7]
   - start_service[15] >= -999973.2688131087
 NodeArrivalVehicles[15,8]: - 999999 x[15,8] + start_service[8]
   - start_service[15] >= -999941.9077472225
 NodeArrivalVehicles[15,9]: - 999999 x[15,9] + start_service[9]
   - start_service[15] >= -999965.8333693258
 NodeArrivalVehicles[15,10]: - 999999 x[15,10] + start_service[10]
   - start_service[15] >= -999982.2907259347
 NodeArrivalVehicles[15,11]: - 999999 x[15,11] + start_service[11]
   - start_service[15] >= -999946.1993118351
 NodeArrivalVehicles[15,12]: - 999999 x[15,12] + start_service[12]
   - start_service[15] >= -999953.4152526929
 NodeArrivalVehicles[15,13]: - 999999 x[15,13] + start_service[13]
   - start_service[15] >= -999966.5641297009
 NodeArrivalVehicles[15,14]: - 999999 x[15,14] + start_service[14]
   - start_service[15] >= -999924.4748369423
 NodeArrivalVehicles[15,15]: - 999999 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 999999 x[15,16] - start_service[15]
   + start_service[16] >= -999953.4384806296
 NodeArrivalVehicles[15,17]: - 999999 x[15,17] - start_service[15]
   + start_service[17] >= -999973.2688131087
 NodeArrivalVehicles[15,18]: - 999999 x[15,18] - start_service[15]
   + start_service[18] >= -999941.9077472225
 NodeArrivalVehicles[15,19]: - 999999 x[15,19] - start_service[15]
   + start_service[19] >= -999965.8333693258
 NodeArrivalVehicles[15,20]: - 999999 x[15,20] - start_service[15]
   + start_service[20] >= -999982.2907259347
 NodeArrivalVehicles[15,21]: - 999999 x[15,21] - start_service[15]
   + start_service[21] >= -999946.1993118351
 NodeArrivalVehicles[15,22]: - 999999 x[15,22] - start_service[15]
   + start_service[22] >= -999960.2532812204
 NodeArrivalVehicles[15,23]: - 999999 x[15,23] - start_service[15]
   + start_service[23] >= -999960.2532812204
 NodeArrivalVehicles[16,1]: - 999999 x[16,1] + start_service[1]
   - start_service[16] >= -999955.5912513951
 NodeArrivalVehicles[16,2]: - 999999 x[16,2] + start_service[2]
   - start_service[16] >= -999926.3485351843
 NodeArrivalVehicles[16,3]: - 999999 x[16,3] + start_service[3]
   - start_service[16] >= -999937.2267384126
 NodeArrivalVehicles[16,4]: - 999999 x[16,4] + start_service[4]
   - start_service[16] >= -999917.2796655721
 NodeArrivalVehicles[16,5]: - 999999 x[16,5] + start_service[5]
   - start_service[16] >= -999953.4384806296
 NodeArrivalVehicles[16,6]: - 999999 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 999999 x[16,7] + start_service[7]
   - start_service[16] >= -999972.6979548086
 NodeArrivalVehicles[16,8]: - 999999 x[16,8] + start_service[8]
   - start_service[16] >= -999946.9351560561
 NodeArrivalVehicles[16,9]: - 999999 x[16,9] + start_service[9]
   - start_service[16] >= -999928.9293963189
 NodeArrivalVehicles[16,10]: - 999999 x[16,10] + start_service[10]
   - start_service[16] >= -999968.6558211173
 NodeArrivalVehicles[16,11]: - 999999 x[16,11] + start_service[11]
   - start_service[16] >= -999906.2176650148
 NodeArrivalVehicles[16,12]: - 999999 x[16,12] + start_service[12]
   - start_service[16] >= -999926.3485351843
 NodeArrivalVehicles[16,13]: - 999999 x[16,13] + start_service[13]
   - start_service[16] >= -999937.2267384126
 NodeArrivalVehicles[16,14]: - 999999 x[16,14] + start_service[14]
   - start_service[16] >= -999917.2796655721
 NodeArrivalVehicles[16,15]: - 999999 x[16,15] + start_service[15]
   - start_service[16] >= -999953.4384806296
 NodeArrivalVehicles[16,16]: - 999999 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 999999 x[16,17] - start_service[16]
   + start_service[17] >= -999972.6979548086
 NodeArrivalVehicles[16,18]: - 999999 x[16,18] - start_service[16]
   + start_service[18] >= -999946.9351560561
 NodeArrivalVehicles[16,19]: - 999999 x[16,19] - start_service[16]
   + start_service[19] >= -999928.9293963189
 NodeArrivalVehicles[16,20]: - 999999 x[16,20] - start_service[16]
   + start_service[20] >= -999968.6558211173
 NodeArrivalVehicles[16,21]: - 999999 x[16,21] - start_service[16]
   + start_service[21] >= -999906.2176650148
 NodeArrivalVehicles[16,22]: - 999999 x[16,22] - start_service[16]
   + start_service[22] >= -999955.5912513951
 NodeArrivalVehicles[16,23]: - 999999 x[16,23] - start_service[16]
   + start_service[23] >= -999955.5912513951
 NodeArrivalVehicles[17,1]: - 999999 x[17,1] + start_service[1]
   - start_service[17] >= -999952.271273898
 NodeArrivalVehicles[17,2]: - 999999 x[17,2] + start_service[2]
   - start_service[17] >= -999952.1869815146
 NodeArrivalVehicles[17,3]: - 999999 x[17,3] + start_service[3]
   - start_service[17] >= -999963.4959134692
 NodeArrivalVehicles[17,4]: - 999999 x[17,4] + start_service[4]
   - start_service[17] >= -999936.6080027135
 NodeArrivalVehicles[17,5]: - 999999 x[17,5] + start_service[5]
   - start_service[17] >= -999973.2688131087
 NodeArrivalVehicles[17,6]: - 999999 x[17,6] + start_service[6]
   - start_service[17] >= -999972.6979548086
 NodeArrivalVehicles[17,7]: - 999999 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 999999 x[17,8] + start_service[8]
   - start_service[17] >= -999961.7262354469
 NodeArrivalVehicles[17,9]: - 999999 x[17,9] + start_service[9]
   - start_service[17] >= -999941.5599706656
 NodeArrivalVehicles[17,10]: - 999999 x[17,10] + start_service[10]
   - start_service[17] >= -999979.6262282272
 NodeArrivalVehicles[17,11]: - 999999 x[17,11] + start_service[11]
   - start_service[17] >= -999932.1978989958
 NodeArrivalVehicles[17,12]: - 999999 x[17,12] + start_service[12]
   - start_service[17] >= -999952.1869815146
 NodeArrivalVehicles[17,13]: - 999999 x[17,13] + start_service[13]
   - start_service[17] >= -999963.4959134692
 NodeArrivalVehicles[17,14]: - 999999 x[17,14] + start_service[14]
   - start_service[17] >= -999936.6080027135
 NodeArrivalVehicles[17,15]: - 999999 x[17,15] + start_service[15]
   - start_service[17] >= -999973.2688131087
 NodeArrivalVehicles[17,16]: - 999999 x[17,16] + start_service[16]
   - start_service[17] >= -999972.6979548086
 NodeArrivalVehicles[17,17]: - 999999 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 999999 x[17,18] - start_service[17]
   + start_service[18] >= -999961.7262354469
 NodeArrivalVehicles[17,19]: - 999999 x[17,19] - start_service[17]
   + start_service[19] >= -999941.5599706656
 NodeArrivalVehicles[17,20]: - 999999 x[17,20] - start_service[17]
   + start_service[20] >= -999979.6262282272
 NodeArrivalVehicles[17,21]: - 999999 x[17,21] - start_service[17]
   + start_service[21] >= -999932.1978989958
 NodeArrivalVehicles[17,22]: - 999999 x[17,22] - start_service[17]
   + start_service[22] >= -999952.271273898
 NodeArrivalVehicles[17,23]: - 999999 x[17,23] - start_service[17]
   + start_service[23] >= -999952.271273898
 NodeArrivalVehicles[18,1]: - 999999 x[18,1] + start_service[1]
   - start_service[18] >= -999915.0779296518
 NodeArrivalVehicles[18,2]: - 999999 x[18,2] + start_service[2]
   - start_service[18] >= -999957.5030249554
 NodeArrivalVehicles[18,3]: - 999999 x[18,3] + start_service[3]
   - start_service[18] >= -999958.5450983534
 NodeArrivalVehicles[18,4]: - 999999 x[18,4] + start_service[4]
   - start_service[18] >= -999968.3875979197
 NodeArrivalVehicles[18,5]: - 999999 x[18,5] + start_service[5]
   - start_service[18] >= -999941.9077472225
 NodeArrivalVehicles[18,6]: - 999999 x[18,6] + start_service[6]
   - start_service[18] >= -999946.9351560561
 NodeArrivalVehicles[18,7]: - 999999 x[18,7] + start_service[7]
   - start_service[18] >= -999961.7262354469
 NodeArrivalVehicles[18,8]: - 999999 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 999999 x[18,9] + start_service[9]
   - start_service[18] >= -999908.812105495
 NodeArrivalVehicles[18,10]: - 999999 x[18,10] + start_service[10]
   - start_service[18] >= -999942.5652146442
 NodeArrivalVehicles[18,11]: - 999999 x[18,11] + start_service[11]
   - start_service[18] >= -999927.2610558096
 NodeArrivalVehicles[18,12]: - 999999 x[18,12] + start_service[12]
   - start_service[18] >= -999957.5030249554
 NodeArrivalVehicles[18,13]: - 999999 x[18,13] + start_service[13]
   - start_service[18] >= -999958.5450983534
 NodeArrivalVehicles[18,14]: - 999999 x[18,14] + start_service[14]
   - start_service[18] >= -999968.3875979197
 NodeArrivalVehicles[18,15]: - 999999 x[18,15] + start_service[15]
   - start_service[18] >= -999941.9077472225
 NodeArrivalVehicles[18,16]: - 999999 x[18,16] + start_service[16]
   - start_service[18] >= -999946.9351560561
 NodeArrivalVehicles[18,17]: - 999999 x[18,17] + start_service[17]
   - start_service[18] >= -999961.7262354469
 NodeArrivalVehicles[18,18]: - 999999 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 999999 x[18,19] - start_service[18]
   + start_service[19] >= -999908.812105495
 NodeArrivalVehicles[18,20]: - 999999 x[18,20] - start_service[18]
   + start_service[20] >= -999942.5652146442
 NodeArrivalVehicles[18,21]: - 999999 x[18,21] - start_service[18]
   + start_service[21] >= -999927.2610558096
 NodeArrivalVehicles[18,22]: - 999999 x[18,22] - start_service[18]
   + start_service[22] >= -999915.0779296518
 NodeArrivalVehicles[18,23]: - 999999 x[18,23] - start_service[18]
   + start_service[23] >= -999915.0779296518
 NodeArrivalVehicles[19,1]: - 999999 x[19,1] + start_service[1]
   - start_service[19] >= -999960.7764686092
 NodeArrivalVehicles[19,2]: - 999999 x[19,2] + start_service[2]
   - start_service[19] >= -999927.6383894689
 NodeArrivalVehicles[19,3]: - 999999 x[19,3] + start_service[3]
   - start_service[19] >= -999939.1789229446
 NodeArrivalVehicles[19,4]: - 999999 x[19,4] + start_service[4]
   - start_service[19] >= -999893.8315006682
 NodeArrivalVehicles[19,5]: - 999999 x[19,5] + start_service[5]
   - start_service[19] >= -999965.8333693258
 NodeArrivalVehicles[19,6]: - 999999 x[19,6] + start_service[6]
   - start_service[19] >= -999928.9293963189
 NodeArrivalVehicles[19,7]: - 999999 x[19,7] + start_service[7]
   - start_service[19] >= -999941.5599706656
 NodeArrivalVehicles[19,8]: - 999999 x[19,8] + start_service[8]
   - start_service[19] >= -999908.812105495
 NodeArrivalVehicles[19,9]: - 999999 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 999999 x[19,10] + start_service[10]
   - start_service[19] >= -999958.0904356453
 NodeArrivalVehicles[19,11]: - 999999 x[19,11] + start_service[11]
   - start_service[19] >= -999934.7740920822
 NodeArrivalVehicles[19,12]: - 999999 x[19,12] + start_service[12]
   - start_service[19] >= -999927.6383894689
 NodeArrivalVehicles[19,13]: - 999999 x[19,13] + start_service[13]
   - start_service[19] >= -999939.1789229446
 NodeArrivalVehicles[19,14]: - 999999 x[19,14] + start_service[14]
   - start_service[19] >= -999893.8315006682
 NodeArrivalVehicles[19,15]: - 999999 x[19,15] + start_service[15]
   - start_service[19] >= -999965.8333693258
 NodeArrivalVehicles[19,16]: - 999999 x[19,16] + start_service[16]
   - start_service[19] >= -999928.9293963189
 NodeArrivalVehicles[19,17]: - 999999 x[19,17] + start_service[17]
   - start_service[19] >= -999941.5599706656
 NodeArrivalVehicles[19,18]: - 999999 x[19,18] + start_service[18]
   - start_service[19] >= -999908.812105495
 NodeArrivalVehicles[19,19]: - 999999 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 999999 x[19,20] - start_service[19]
   + start_service[20] >= -999958.0904356453
 NodeArrivalVehicles[19,21]: - 999999 x[19,21] - start_service[19]
   + start_service[21] >= -999934.7740920822
 NodeArrivalVehicles[19,22]: - 999999 x[19,22] - start_service[19]
   + start_service[22] >= -999960.7764686092
 NodeArrivalVehicles[19,23]: - 999999 x[19,23] - start_service[19]
   + start_service[23] >= -999960.7764686092
 NodeArrivalVehicles[20,1]: - 999999 x[20,1] + start_service[1]
   - start_service[20] >= -999970.4639211913
 NodeArrivalVehicles[20,2]: - 999999 x[20,2] + start_service[2]
   - start_service[20] >= -999942.098220245
 NodeArrivalVehicles[20,3]: - 999999 x[20,3] + start_service[3]
   - start_service[20] >= -999955.1505151036
 NodeArrivalVehicles[20,4]: - 999999 x[20,4] + start_service[4]
   - start_service[20] >= -999919.4256871791
 NodeArrivalVehicles[20,5]: - 999999 x[20,5] + start_service[5]
   - start_service[20] >= -999982.2907259347
 NodeArrivalVehicles[20,6]: - 999999 x[20,6] + start_service[6]
   - start_service[20] >= -999968.6558211173
 NodeArrivalVehicles[20,7]: - 999999 x[20,7] + start_service[7]
   - start_service[20] >= -999979.6262282272
 NodeArrivalVehicles[20,8]: - 999999 x[20,8] + start_service[8]
   - start_service[20] >= -999942.5652146442
 NodeArrivalVehicles[20,9]: - 999999 x[20,9] + start_service[9]
   - start_service[20] >= -999958.0904356453
 NodeArrivalVehicles[20,10]: - 999999 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 999999 x[20,11] + start_service[11]
   - start_service[20] >= -999930.2057225219
 NodeArrivalVehicles[20,12]: - 999999 x[20,12] + start_service[12]
   - start_service[20] >= -999942.098220245
 NodeArrivalVehicles[20,13]: - 999999 x[20,13] + start_service[13]
   - start_service[20] >= -999955.1505151036
 NodeArrivalVehicles[20,14]: - 999999 x[20,14] + start_service[14]
   - start_service[20] >= -999919.4256871791
 NodeArrivalVehicles[20,15]: - 999999 x[20,15] + start_service[15]
   - start_service[20] >= -999982.2907259347
 NodeArrivalVehicles[20,16]: - 999999 x[20,16] + start_service[16]
   - start_service[20] >= -999968.6558211173
 NodeArrivalVehicles[20,17]: - 999999 x[20,17] + start_service[17]
   - start_service[20] >= -999979.6262282272
 NodeArrivalVehicles[20,18]: - 999999 x[20,18] + start_service[18]
   - start_service[20] >= -999942.5652146442
 NodeArrivalVehicles[20,19]: - 999999 x[20,19] + start_service[19]
   - start_service[20] >= -999958.0904356453
 NodeArrivalVehicles[20,20]: - 999999 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 999999 x[20,21] - start_service[20]
   + start_service[21] >= -999930.2057225219
 NodeArrivalVehicles[20,22]: - 999999 x[20,22] - start_service[20]
   + start_service[22] >= -999970.4639211913
 NodeArrivalVehicles[20,23]: - 999999 x[20,23] - start_service[20]
   + start_service[23] >= -999970.4639211913
 NodeArrivalVehicles[21,1]: - 999999 x[21,1] + start_service[1]
   - start_service[21] >= -999909.1632951815
 NodeArrivalVehicles[21,2]: - 999999 x[21,2] + start_service[2]
   - start_service[21] >= -999968.3063737304
 NodeArrivalVehicles[21,3]: - 999999 x[21,3] + start_service[3]
   - start_service[21] >= -999965.4157057379
 NodeArrivalVehicles[21,4]: - 999999 x[21,4] + start_service[4]
   - start_service[21] >= -999933.3662834174
 NodeArrivalVehicles[21,5]: - 999999 x[21,5] + start_service[5]
   - start_service[21] >= -999946.1993118351
 NodeArrivalVehicles[21,6]: - 999999 x[21,6] + start_service[6]
   - start_service[21] >= -999906.2176650148
 NodeArrivalVehicles[21,7]: - 999999 x[21,7] + start_service[7]
   - start_service[21] >= -999932.1978989958
 NodeArrivalVehicles[21,8]: - 999999 x[21,8] + start_service[8]
   - start_service[21] >= -999927.2610558096
 NodeArrivalVehicles[21,9]: - 999999 x[21,9] + start_service[9]
   - start_service[21] >= -999934.7740920822
 NodeArrivalVehicles[21,10]: - 999999 x[21,10] + start_service[10]
   - start_service[21] >= -999930.2057225219
 NodeArrivalVehicles[21,11]: - 999999 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 999999 x[21,12] + start_service[12]
   - start_service[21] >= -999968.3063737304
 NodeArrivalVehicles[21,13]: - 999999 x[21,13] + start_service[13]
   - start_service[21] >= -999965.4157057379
 NodeArrivalVehicles[21,14]: - 999999 x[21,14] + start_service[14]
   - start_service[21] >= -999933.3662834174
 NodeArrivalVehicles[21,15]: - 999999 x[21,15] + start_service[15]
   - start_service[21] >= -999946.1993118351
 NodeArrivalVehicles[21,16]: - 999999 x[21,16] + start_service[16]
   - start_service[21] >= -999906.2176650148
 NodeArrivalVehicles[21,17]: - 999999 x[21,17] + start_service[17]
   - start_service[21] >= -999932.1978989958
 NodeArrivalVehicles[21,18]: - 999999 x[21,18] + start_service[18]
   - start_service[21] >= -999927.2610558096
 NodeArrivalVehicles[21,19]: - 999999 x[21,19] + start_service[19]
   - start_service[21] >= -999934.7740920822
 NodeArrivalVehicles[21,20]: - 999999 x[21,20] + start_service[20]
   - start_service[21] >= -999930.2057225219
 NodeArrivalVehicles[21,21]: - 999999 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 999999 x[21,22] - start_service[21]
   + start_service[22] >= -999909.1632951815
 NodeArrivalVehicles[21,23]: - 999999 x[21,23] - start_service[21]
   + start_service[23] >= -999909.1632951815
 NodeArrivalVehicles[22,1]: - 999999 x[22,1] + start_service[1]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 999999 x[22,2] + start_service[2]
   - start_service[22] >= -999915.2024796238
 NodeArrivalVehicles[22,3]: - 999999 x[22,3] + start_service[3]
   - start_service[22] >= -999928.4369415202
 NodeArrivalVehicles[22,4]: - 999999 x[22,4] + start_service[4]
   - start_service[22] >= -999890.901035983
 NodeArrivalVehicles[22,5]: - 999999 x[22,5] + start_service[5]
   - start_service[22] >= -999960.2532812204
 NodeArrivalVehicles[22,6]: - 999999 x[22,6] + start_service[6]
   - start_service[22] >= -999955.5912513951
 NodeArrivalVehicles[22,7]: - 999999 x[22,7] + start_service[7]
   - start_service[22] >= -999952.271273898
 NodeArrivalVehicles[22,8]: - 999999 x[22,8] + start_service[8]
   - start_service[22] >= -999915.0779296518
 NodeArrivalVehicles[22,9]: - 999999 x[22,9] + start_service[9]
   - start_service[22] >= -999960.7764686092
 NodeArrivalVehicles[22,10]: - 999999 x[22,10] + start_service[10]
   - start_service[22] >= -999970.4639211913
 NodeArrivalVehicles[22,11]: - 999999 x[22,11] + start_service[11]
   - start_service[22] >= -999909.1632951815
 NodeArrivalVehicles[22,12]: - 999999 x[22,12] + start_service[12]
   - start_service[22] >= -999915.2024796238
 NodeArrivalVehicles[22,13]: - 999999 x[22,13] + start_service[13]
   - start_service[22] >= -999928.4369415202
 NodeArrivalVehicles[22,14]: - 999999 x[22,14] + start_service[14]
   - start_service[22] >= -999890.901035983
 NodeArrivalVehicles[22,15]: - 999999 x[22,15] + start_service[15]
   - start_service[22] >= -999960.2532812204
 NodeArrivalVehicles[22,16]: - 999999 x[22,16] + start_service[16]
   - start_service[22] >= -999955.5912513951
 NodeArrivalVehicles[22,17]: - 999999 x[22,17] + start_service[17]
   - start_service[22] >= -999952.271273898
 NodeArrivalVehicles[22,18]: - 999999 x[22,18] + start_service[18]
   - start_service[22] >= -999915.0779296518
 NodeArrivalVehicles[22,19]: - 999999 x[22,19] + start_service[19]
   - start_service[22] >= -999960.7764686092
 NodeArrivalVehicles[22,20]: - 999999 x[22,20] + start_service[20]
   - start_service[22] >= -999970.4639211913
 NodeArrivalVehicles[22,21]: - 999999 x[22,21] + start_service[21]
   - start_service[22] >= -999909.1632951815
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
   - start_service[0] + start_service[2] >= -999915.2024796238
 StartServiceRobots[0,3]: - 999999 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999928.4369415202
 StartServiceRobots[0,4]: - 999999 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999890.901035983
 StartServiceRobots[0,5]: - 999999 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999960.2532812204
 StartServiceRobots[0,6]: - 999999 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999955.5912513951
 StartServiceRobots[0,7]: - 999999 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999952.271273898
 StartServiceRobots[0,8]: - 999999 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999915.0779296518
 StartServiceRobots[0,9]: - 999999 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999960.7764686092
 StartServiceRobots[0,10]: - 999999 v[0,10] - service_time[0]
   - start_service[0] + start_service[10] >= -999970.4639211913
 StartServiceRobots[0,11]: - 999999 v[0,11] - service_time[0]
   - start_service[0] + start_service[11] >= -999909.1632951815
 StartServiceRobots[0,12]: - 999999 v[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999915.2024796238
 StartServiceRobots[0,13]: - 999999 v[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999928.4369415202
 StartServiceRobots[0,14]: - 999999 v[0,14] - service_time[0]
   - start_service[0] + start_service[14] >= -999890.901035983
 StartServiceRobots[0,15]: - 999999 v[0,15] - service_time[0]
   - start_service[0] + start_service[15] >= -999960.2532812204
 StartServiceRobots[0,16]: - 999999 v[0,16] - service_time[0]
   - start_service[0] + start_service[16] >= -999955.5912513951
 StartServiceRobots[0,17]: - 999999 v[0,17] - service_time[0]
   - start_service[0] + start_service[17] >= -999952.271273898
 StartServiceRobots[0,18]: - 999999 v[0,18] - service_time[0]
   - start_service[0] + start_service[18] >= -999915.0779296518
 StartServiceRobots[0,19]: - 999999 v[0,19] - service_time[0]
   - start_service[0] + start_service[19] >= -999960.7764686092
 StartServiceRobots[0,20]: - 999999 v[0,20] - service_time[0]
   - start_service[0] + start_service[20] >= -999970.4639211913
 StartServiceRobots[0,21]: - 999999 v[0,21] - service_time[0]
   - start_service[0] + start_service[21] >= -999909.1632951815
 StartServiceRobots[0,22]: - 999999 v[0,22] - service_time[0]
   - start_service[0] + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999 v[0,23] - service_time[0]
   - start_service[0] + start_service[23] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 999999 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999915.2024796238
 StartServiceRobots[1,3]: - 999999 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999928.4369415202
 StartServiceRobots[1,4]: - 999999 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999890.901035983
 StartServiceRobots[1,5]: - 999999 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999960.2532812204
 StartServiceRobots[1,6]: - 999999 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999955.5912513951
 StartServiceRobots[1,7]: - 999999 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999952.271273898
 StartServiceRobots[1,8]: - 999999 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999915.0779296518
 StartServiceRobots[1,9]: - 999999 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999960.7764686092
 StartServiceRobots[1,10]: - 999999 v[1,10] - service_time[1]
   - start_service[1] + start_service[10] >= -999970.4639211913
 StartServiceRobots[1,11]: - 999999 v[1,11] - service_time[1]
   - start_service[1] + start_service[11] >= -999909.1632951815
 StartServiceRobots[1,12]: - 999999 v[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999915.2024796238
 StartServiceRobots[1,13]: - 999999 v[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999928.4369415202
 StartServiceRobots[1,14]: - 999999 v[1,14] - service_time[1]
   - start_service[1] + start_service[14] >= -999890.901035983
 StartServiceRobots[1,15]: - 999999 v[1,15] - service_time[1]
   - start_service[1] + start_service[15] >= -999960.2532812204
 StartServiceRobots[1,16]: - 999999 v[1,16] - service_time[1]
   - start_service[1] + start_service[16] >= -999955.5912513951
 StartServiceRobots[1,17]: - 999999 v[1,17] - service_time[1]
   - start_service[1] + start_service[17] >= -999952.271273898
 StartServiceRobots[1,18]: - 999999 v[1,18] - service_time[1]
   - start_service[1] + start_service[18] >= -999915.0779296518
 StartServiceRobots[1,19]: - 999999 v[1,19] - service_time[1]
   - start_service[1] + start_service[19] >= -999960.7764686092
 StartServiceRobots[1,20]: - 999999 v[1,20] - service_time[1]
   - start_service[1] + start_service[20] >= -999970.4639211913
 StartServiceRobots[1,21]: - 999999 v[1,21] - service_time[1]
   - start_service[1] + start_service[21] >= -999909.1632951815
 StartServiceRobots[1,22]: - 999999 v[1,22] - service_time[1]
   - start_service[1] + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999 v[1,23] - service_time[1]
   - start_service[1] + start_service[23] >= -999999
 StartServiceRobots[2,1]: - 999999 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999915.2024796238
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 999999 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999985.7650834119
 StartServiceRobots[2,4]: - 999999 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999961.403612743
 StartServiceRobots[2,5]: - 999999 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999953.4152526929
 StartServiceRobots[2,6]: - 999999 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999926.3485351843
 StartServiceRobots[2,7]: - 999999 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999952.1869815146
 StartServiceRobots[2,8]: - 999999 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999957.5030249554
 StartServiceRobots[2,9]: - 999999 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999927.6383894689
 StartServiceRobots[2,10]: - 999999 v[2,10] - service_time[2]
   - start_service[2] + start_service[10] >= -999942.098220245
 StartServiceRobots[2,11]: - 999999 v[2,11] - service_time[2]
   - start_service[2] + start_service[11] >= -999968.3063737304
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 999999 v[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999985.7650834119
 StartServiceRobots[2,14]: - 999999 v[2,14] - service_time[2]
   - start_service[2] + start_service[14] >= -999961.403612743
 StartServiceRobots[2,15]: - 999999 v[2,15] - service_time[2]
   - start_service[2] + start_service[15] >= -999953.4152526929
 StartServiceRobots[2,16]: - 999999 v[2,16] - service_time[2]
   - start_service[2] + start_service[16] >= -999926.3485351843
 StartServiceRobots[2,17]: - 999999 v[2,17] - service_time[2]
   - start_service[2] + start_service[17] >= -999952.1869815146
 StartServiceRobots[2,18]: - 999999 v[2,18] - service_time[2]
   - start_service[2] + start_service[18] >= -999957.5030249554
 StartServiceRobots[2,19]: - 999999 v[2,19] - service_time[2]
   - start_service[2] + start_service[19] >= -999927.6383894689
 StartServiceRobots[2,20]: - 999999 v[2,20] - service_time[2]
   - start_service[2] + start_service[20] >= -999942.098220245
 StartServiceRobots[2,21]: - 999999 v[2,21] - service_time[2]
   - start_service[2] + start_service[21] >= -999968.3063737304
 StartServiceRobots[2,22]: - 999999 v[2,22] - service_time[2]
   - start_service[2] + start_service[22] >= -999915.2024796238
 StartServiceRobots[2,23]: - 999999 v[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= -999915.2024796238
 StartServiceRobots[3,1]: - 999999 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999928.4369415202
 StartServiceRobots[3,2]: - 999999 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999985.7650834119
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 999999 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999953.0826375335
 StartServiceRobots[3,5]: - 999999 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999966.5641297009
 StartServiceRobots[3,6]: - 999999 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999937.2267384126
 StartServiceRobots[3,7]: - 999999 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999963.4959134692
 StartServiceRobots[3,8]: - 999999 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999958.5450983534
 StartServiceRobots[3,9]: - 999999 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999939.1789229446
 StartServiceRobots[3,10]: - 999999 v[3,10] - service_time[3]
   - start_service[3] + start_service[10] >= -999955.1505151036
 StartServiceRobots[3,11]: - 999999 v[3,11] - service_time[3]
   - start_service[3] + start_service[11] >= -999965.4157057379
 StartServiceRobots[3,12]: - 999999 v[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999985.7650834119
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 999999 v[3,14] - service_time[3]
   - start_service[3] + start_service[14] >= -999953.0826375335
 StartServiceRobots[3,15]: - 999999 v[3,15] - service_time[3]
   - start_service[3] + start_service[15] >= -999966.5641297009
 StartServiceRobots[3,16]: - 999999 v[3,16] - service_time[3]
   - start_service[3] + start_service[16] >= -999937.2267384126
 StartServiceRobots[3,17]: - 999999 v[3,17] - service_time[3]
   - start_service[3] + start_service[17] >= -999963.4959134692
 StartServiceRobots[3,18]: - 999999 v[3,18] - service_time[3]
   - start_service[3] + start_service[18] >= -999958.5450983534
 StartServiceRobots[3,19]: - 999999 v[3,19] - service_time[3]
   - start_service[3] + start_service[19] >= -999939.1789229446
 StartServiceRobots[3,20]: - 999999 v[3,20] - service_time[3]
   - start_service[3] + start_service[20] >= -999955.1505151036
 StartServiceRobots[3,21]: - 999999 v[3,21] - service_time[3]
   - start_service[3] + start_service[21] >= -999965.4157057379
 StartServiceRobots[3,22]: - 999999 v[3,22] - service_time[3]
   - start_service[3] + start_service[22] >= -999928.4369415202
 StartServiceRobots[3,23]: - 999999 v[3,23] - service_time[3]
   - start_service[3] + start_service[23] >= -999928.4369415202
 StartServiceRobots[4,1]: - 999999 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999890.901035983
 StartServiceRobots[4,2]: - 999999 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999961.403612743
 StartServiceRobots[4,3]: - 999999 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999953.0826375335
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 999999 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999924.4748369423
 StartServiceRobots[4,6]: - 999999 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999917.2796655721
 StartServiceRobots[4,7]: - 999999 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999936.6080027135
 StartServiceRobots[4,8]: - 999999 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999968.3875979197
 StartServiceRobots[4,9]: - 999999 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999893.8315006682
 StartServiceRobots[4,10]: - 999999 v[4,10] - service_time[4]
   - start_service[4] + start_service[10] >= -999919.4256871791
 StartServiceRobots[4,11]: - 999999 v[4,11] - service_time[4]
   - start_service[4] + start_service[11] >= -999933.3662834174
 StartServiceRobots[4,12]: - 999999 v[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999961.403612743
 StartServiceRobots[4,13]: - 999999 v[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999953.0826375335
 StartServiceRobots[4,14]: - 999999 v[4,14] - service_time[4]
   - start_service[4] + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 999999 v[4,15] - service_time[4]
   - start_service[4] + start_service[15] >= -999924.4748369423
 StartServiceRobots[4,16]: - 999999 v[4,16] - service_time[4]
   - start_service[4] + start_service[16] >= -999917.2796655721
 StartServiceRobots[4,17]: - 999999 v[4,17] - service_time[4]
   - start_service[4] + start_service[17] >= -999936.6080027135
 StartServiceRobots[4,18]: - 999999 v[4,18] - service_time[4]
   - start_service[4] + start_service[18] >= -999968.3875979197
 StartServiceRobots[4,19]: - 999999 v[4,19] - service_time[4]
   - start_service[4] + start_service[19] >= -999893.8315006682
 StartServiceRobots[4,20]: - 999999 v[4,20] - service_time[4]
   - start_service[4] + start_service[20] >= -999919.4256871791
 StartServiceRobots[4,21]: - 999999 v[4,21] - service_time[4]
   - start_service[4] + start_service[21] >= -999933.3662834174
 StartServiceRobots[4,22]: - 999999 v[4,22] - service_time[4]
   - start_service[4] + start_service[22] >= -999890.901035983
 StartServiceRobots[4,23]: - 999999 v[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= -999890.901035983
 StartServiceRobots[5,1]: - 999999 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999960.2532812204
 StartServiceRobots[5,2]: - 999999 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999953.4152526929
 StartServiceRobots[5,3]: - 999999 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999966.5641297009
 StartServiceRobots[5,4]: - 999999 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999924.4748369423
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 999999 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999953.4384806296
 StartServiceRobots[5,7]: - 999999 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999973.2688131087
 StartServiceRobots[5,8]: - 999999 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999941.9077472225
 StartServiceRobots[5,9]: - 999999 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999965.8333693258
 StartServiceRobots[5,10]: - 999999 v[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999982.2907259347
 StartServiceRobots[5,11]: - 999999 v[5,11] - service_time[5]
   - start_service[5] + start_service[11] >= -999946.1993118351
 StartServiceRobots[5,12]: - 999999 v[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999953.4152526929
 StartServiceRobots[5,13]: - 999999 v[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999966.5641297009
 StartServiceRobots[5,14]: - 999999 v[5,14] - service_time[5]
   - start_service[5] + start_service[14] >= -999924.4748369423
 StartServiceRobots[5,15]: - 999999 v[5,15] - service_time[5]
   - start_service[5] + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 999999 v[5,16] - service_time[5]
   - start_service[5] + start_service[16] >= -999953.4384806296
 StartServiceRobots[5,17]: - 999999 v[5,17] - service_time[5]
   - start_service[5] + start_service[17] >= -999973.2688131087
 StartServiceRobots[5,18]: - 999999 v[5,18] - service_time[5]
   - start_service[5] + start_service[18] >= -999941.9077472225
 StartServiceRobots[5,19]: - 999999 v[5,19] - service_time[5]
   - start_service[5] + start_service[19] >= -999965.8333693258
 StartServiceRobots[5,20]: - 999999 v[5,20] - service_time[5]
   - start_service[5] + start_service[20] >= -999982.2907259347
 StartServiceRobots[5,21]: - 999999 v[5,21] - service_time[5]
   - start_service[5] + start_service[21] >= -999946.1993118351
 StartServiceRobots[5,22]: - 999999 v[5,22] - service_time[5]
   - start_service[5] + start_service[22] >= -999960.2532812204
 StartServiceRobots[5,23]: - 999999 v[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= -999960.2532812204
 StartServiceRobots[6,1]: - 999999 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999955.5912513951
 StartServiceRobots[6,2]: - 999999 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999926.3485351843
 StartServiceRobots[6,3]: - 999999 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999937.2267384126
 StartServiceRobots[6,4]: - 999999 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999917.2796655721
 StartServiceRobots[6,5]: - 999999 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999953.4384806296
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 999999 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999972.6979548086
 StartServiceRobots[6,8]: - 999999 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999946.9351560561
 StartServiceRobots[6,9]: - 999999 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999928.9293963189
 StartServiceRobots[6,10]: - 999999 v[6,10] - service_time[6]
   - start_service[6] + start_service[10] >= -999968.6558211173
 StartServiceRobots[6,11]: - 999999 v[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999906.2176650148
 StartServiceRobots[6,12]: - 999999 v[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999926.3485351843
 StartServiceRobots[6,13]: - 999999 v[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999937.2267384126
 StartServiceRobots[6,14]: - 999999 v[6,14] - service_time[6]
   - start_service[6] + start_service[14] >= -999917.2796655721
 StartServiceRobots[6,15]: - 999999 v[6,15] - service_time[6]
   - start_service[6] + start_service[15] >= -999953.4384806296
 StartServiceRobots[6,16]: - 999999 v[6,16] - service_time[6]
   - start_service[6] + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 999999 v[6,17] - service_time[6]
   - start_service[6] + start_service[17] >= -999972.6979548086
 StartServiceRobots[6,18]: - 999999 v[6,18] - service_time[6]
   - start_service[6] + start_service[18] >= -999946.9351560561
 StartServiceRobots[6,19]: - 999999 v[6,19] - service_time[6]
   - start_service[6] + start_service[19] >= -999928.9293963189
 StartServiceRobots[6,20]: - 999999 v[6,20] - service_time[6]
   - start_service[6] + start_service[20] >= -999968.6558211173
 StartServiceRobots[6,21]: - 999999 v[6,21] - service_time[6]
   - start_service[6] + start_service[21] >= -999906.2176650148
 StartServiceRobots[6,22]: - 999999 v[6,22] - service_time[6]
   - start_service[6] + start_service[22] >= -999955.5912513951
 StartServiceRobots[6,23]: - 999999 v[6,23] - service_time[6]
   - start_service[6] + start_service[23] >= -999955.5912513951
 StartServiceRobots[7,1]: - 999999 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999952.271273898
 StartServiceRobots[7,2]: - 999999 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999952.1869815146
 StartServiceRobots[7,3]: - 999999 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999963.4959134692
 StartServiceRobots[7,4]: - 999999 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999936.6080027135
 StartServiceRobots[7,5]: - 999999 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999973.2688131087
 StartServiceRobots[7,6]: - 999999 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999972.6979548086
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 999999 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999961.7262354469
 StartServiceRobots[7,9]: - 999999 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999941.5599706656
 StartServiceRobots[7,10]: - 999999 v[7,10] - service_time[7]
   - start_service[7] + start_service[10] >= -999979.6262282272
 StartServiceRobots[7,11]: - 999999 v[7,11] - service_time[7]
   - start_service[7] + start_service[11] >= -999932.1978989958
 StartServiceRobots[7,12]: - 999999 v[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999952.1869815146
 StartServiceRobots[7,13]: - 999999 v[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999963.4959134692
 StartServiceRobots[7,14]: - 999999 v[7,14] - service_time[7]
   - start_service[7] + start_service[14] >= -999936.6080027135
 StartServiceRobots[7,15]: - 999999 v[7,15] - service_time[7]
   - start_service[7] + start_service[15] >= -999973.2688131087
 StartServiceRobots[7,16]: - 999999 v[7,16] - service_time[7]
   - start_service[7] + start_service[16] >= -999972.6979548086
 StartServiceRobots[7,17]: - 999999 v[7,17] - service_time[7]
   - start_service[7] + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 999999 v[7,18] - service_time[7]
   - start_service[7] + start_service[18] >= -999961.7262354469
 StartServiceRobots[7,19]: - 999999 v[7,19] - service_time[7]
   - start_service[7] + start_service[19] >= -999941.5599706656
 StartServiceRobots[7,20]: - 999999 v[7,20] - service_time[7]
   - start_service[7] + start_service[20] >= -999979.6262282272
 StartServiceRobots[7,21]: - 999999 v[7,21] - service_time[7]
   - start_service[7] + start_service[21] >= -999932.1978989958
 StartServiceRobots[7,22]: - 999999 v[7,22] - service_time[7]
   - start_service[7] + start_service[22] >= -999952.271273898
 StartServiceRobots[7,23]: - 999999 v[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= -999952.271273898
 StartServiceRobots[8,1]: - 999999 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999915.0779296518
 StartServiceRobots[8,2]: - 999999 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999957.5030249554
 StartServiceRobots[8,3]: - 999999 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999958.5450983534
 StartServiceRobots[8,4]: - 999999 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999968.3875979197
 StartServiceRobots[8,5]: - 999999 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999941.9077472225
 StartServiceRobots[8,6]: - 999999 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999946.9351560561
 StartServiceRobots[8,7]: - 999999 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999961.7262354469
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 999999 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999908.812105495
 StartServiceRobots[8,10]: - 999999 v[8,10] - service_time[8]
   - start_service[8] + start_service[10] >= -999942.5652146442
 StartServiceRobots[8,11]: - 999999 v[8,11] - service_time[8]
   - start_service[8] + start_service[11] >= -999927.2610558096
 StartServiceRobots[8,12]: - 999999 v[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999957.5030249554
 StartServiceRobots[8,13]: - 999999 v[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999958.5450983534
 StartServiceRobots[8,14]: - 999999 v[8,14] - service_time[8]
   - start_service[8] + start_service[14] >= -999968.3875979197
 StartServiceRobots[8,15]: - 999999 v[8,15] - service_time[8]
   - start_service[8] + start_service[15] >= -999941.9077472225
 StartServiceRobots[8,16]: - 999999 v[8,16] - service_time[8]
   - start_service[8] + start_service[16] >= -999946.9351560561
 StartServiceRobots[8,17]: - 999999 v[8,17] - service_time[8]
   - start_service[8] + start_service[17] >= -999961.7262354469
 StartServiceRobots[8,18]: - 999999 v[8,18] - service_time[8]
   - start_service[8] + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 999999 v[8,19] - service_time[8]
   - start_service[8] + start_service[19] >= -999908.812105495
 StartServiceRobots[8,20]: - 999999 v[8,20] - service_time[8]
   - start_service[8] + start_service[20] >= -999942.5652146442
 StartServiceRobots[8,21]: - 999999 v[8,21] - service_time[8]
   - start_service[8] + start_service[21] >= -999927.2610558096
 StartServiceRobots[8,22]: - 999999 v[8,22] - service_time[8]
   - start_service[8] + start_service[22] >= -999915.0779296518
 StartServiceRobots[8,23]: - 999999 v[8,23] - service_time[8]
   - start_service[8] + start_service[23] >= -999915.0779296518
 StartServiceRobots[9,1]: - 999999 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999960.7764686092
 StartServiceRobots[9,2]: - 999999 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999927.6383894689
 StartServiceRobots[9,3]: - 999999 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999939.1789229446
 StartServiceRobots[9,4]: - 999999 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999893.8315006682
 StartServiceRobots[9,5]: - 999999 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999965.8333693258
 StartServiceRobots[9,6]: - 999999 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999928.9293963189
 StartServiceRobots[9,7]: - 999999 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999941.5599706656
 StartServiceRobots[9,8]: - 999999 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999908.812105495
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 999999 v[9,10] - service_time[9]
   - start_service[9] + start_service[10] >= -999958.0904356453
 StartServiceRobots[9,11]: - 999999 v[9,11] - service_time[9]
   - start_service[9] + start_service[11] >= -999934.7740920822
 StartServiceRobots[9,12]: - 999999 v[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999927.6383894689
 StartServiceRobots[9,13]: - 999999 v[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999939.1789229446
 StartServiceRobots[9,14]: - 999999 v[9,14] - service_time[9]
   - start_service[9] + start_service[14] >= -999893.8315006682
 StartServiceRobots[9,15]: - 999999 v[9,15] - service_time[9]
   - start_service[9] + start_service[15] >= -999965.8333693258
 StartServiceRobots[9,16]: - 999999 v[9,16] - service_time[9]
   - start_service[9] + start_service[16] >= -999928.9293963189
 StartServiceRobots[9,17]: - 999999 v[9,17] - service_time[9]
   - start_service[9] + start_service[17] >= -999941.5599706656
 StartServiceRobots[9,18]: - 999999 v[9,18] - service_time[9]
   - start_service[9] + start_service[18] >= -999908.812105495
 StartServiceRobots[9,19]: - 999999 v[9,19] - service_time[9]
   - start_service[9] + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 999999 v[9,20] - service_time[9]
   - start_service[9] + start_service[20] >= -999958.0904356453
 StartServiceRobots[9,21]: - 999999 v[9,21] - service_time[9]
   - start_service[9] + start_service[21] >= -999934.7740920822
 StartServiceRobots[9,22]: - 999999 v[9,22] - service_time[9]
   - start_service[9] + start_service[22] >= -999960.7764686092
 StartServiceRobots[9,23]: - 999999 v[9,23] - service_time[9]
   - start_service[9] + start_service[23] >= -999960.7764686092
 StartServiceRobots[10,1]: - 999999 v[10,1] - service_time[10]
   + start_service[1] - start_service[10] >= -999970.4639211913
 StartServiceRobots[10,2]: - 999999 v[10,2] - service_time[10]
   + start_service[2] - start_service[10] >= -999942.098220245
 StartServiceRobots[10,3]: - 999999 v[10,3] - service_time[10]
   + start_service[3] - start_service[10] >= -999955.1505151036
 StartServiceRobots[10,4]: - 999999 v[10,4] - service_time[10]
   + start_service[4] - start_service[10] >= -999919.4256871791
 StartServiceRobots[10,5]: - 999999 v[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999982.2907259347
 StartServiceRobots[10,6]: - 999999 v[10,6] - service_time[10]
   + start_service[6] - start_service[10] >= -999968.6558211173
 StartServiceRobots[10,7]: - 999999 v[10,7] - service_time[10]
   + start_service[7] - start_service[10] >= -999979.6262282272
 StartServiceRobots[10,8]: - 999999 v[10,8] - service_time[10]
   + start_service[8] - start_service[10] >= -999942.5652146442
 StartServiceRobots[10,9]: - 999999 v[10,9] - service_time[10]
   + start_service[9] - start_service[10] >= -999958.0904356453
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 999999 v[10,11] - service_time[10]
   - start_service[10] + start_service[11] >= -999930.2057225219
 StartServiceRobots[10,12]: - 999999 v[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999942.098220245
 StartServiceRobots[10,13]: - 999999 v[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999955.1505151036
 StartServiceRobots[10,14]: - 999999 v[10,14] - service_time[10]
   - start_service[10] + start_service[14] >= -999919.4256871791
 StartServiceRobots[10,15]: - 999999 v[10,15] - service_time[10]
   - start_service[10] + start_service[15] >= -999982.2907259347
 StartServiceRobots[10,16]: - 999999 v[10,16] - service_time[10]
   - start_service[10] + start_service[16] >= -999968.6558211173
 StartServiceRobots[10,17]: - 999999 v[10,17] - service_time[10]
   - start_service[10] + start_service[17] >= -999979.6262282272
 StartServiceRobots[10,18]: - 999999 v[10,18] - service_time[10]
   - start_service[10] + start_service[18] >= -999942.5652146442
 StartServiceRobots[10,19]: - 999999 v[10,19] - service_time[10]
   - start_service[10] + start_service[19] >= -999958.0904356453
 StartServiceRobots[10,20]: - 999999 v[10,20] - service_time[10]
   - start_service[10] + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 999999 v[10,21] - service_time[10]
   - start_service[10] + start_service[21] >= -999930.2057225219
 StartServiceRobots[10,22]: - 999999 v[10,22] - service_time[10]
   - start_service[10] + start_service[22] >= -999970.4639211913
 StartServiceRobots[10,23]: - 999999 v[10,23] - service_time[10]
   - start_service[10] + start_service[23] >= -999970.4639211913
 StartServiceRobots[11,1]: - 999999 v[11,1] - service_time[11]
   + start_service[1] - start_service[11] >= -999909.1632951815
 StartServiceRobots[11,2]: - 999999 v[11,2] - service_time[11]
   + start_service[2] - start_service[11] >= -999968.3063737304
 StartServiceRobots[11,3]: - 999999 v[11,3] - service_time[11]
   + start_service[3] - start_service[11] >= -999965.4157057379
 StartServiceRobots[11,4]: - 999999 v[11,4] - service_time[11]
   + start_service[4] - start_service[11] >= -999933.3662834174
 StartServiceRobots[11,5]: - 999999 v[11,5] - service_time[11]
   + start_service[5] - start_service[11] >= -999946.1993118351
 StartServiceRobots[11,6]: - 999999 v[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999906.2176650148
 StartServiceRobots[11,7]: - 999999 v[11,7] - service_time[11]
   + start_service[7] - start_service[11] >= -999932.1978989958
 StartServiceRobots[11,8]: - 999999 v[11,8] - service_time[11]
   + start_service[8] - start_service[11] >= -999927.2610558096
 StartServiceRobots[11,9]: - 999999 v[11,9] - service_time[11]
   + start_service[9] - start_service[11] >= -999934.7740920822
 StartServiceRobots[11,10]: - 999999 v[11,10] - service_time[11]
   + start_service[10] - start_service[11] >= -999930.2057225219
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 999999 v[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999968.3063737304
 StartServiceRobots[11,13]: - 999999 v[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999965.4157057379
 StartServiceRobots[11,14]: - 999999 v[11,14] - service_time[11]
   - start_service[11] + start_service[14] >= -999933.3662834174
 StartServiceRobots[11,15]: - 999999 v[11,15] - service_time[11]
   - start_service[11] + start_service[15] >= -999946.1993118351
 StartServiceRobots[11,16]: - 999999 v[11,16] - service_time[11]
   - start_service[11] + start_service[16] >= -999906.2176650148
 StartServiceRobots[11,17]: - 999999 v[11,17] - service_time[11]
   - start_service[11] + start_service[17] >= -999932.1978989958
 StartServiceRobots[11,18]: - 999999 v[11,18] - service_time[11]
   - start_service[11] + start_service[18] >= -999927.2610558096
 StartServiceRobots[11,19]: - 999999 v[11,19] - service_time[11]
   - start_service[11] + start_service[19] >= -999934.7740920822
 StartServiceRobots[11,20]: - 999999 v[11,20] - service_time[11]
   - start_service[11] + start_service[20] >= -999930.2057225219
 StartServiceRobots[11,21]: - 999999 v[11,21] - service_time[11]
   - start_service[11] + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 999999 v[11,22] - service_time[11]
   - start_service[11] + start_service[22] >= -999909.1632951815
 StartServiceRobots[11,23]: - 999999 v[11,23] - service_time[11]
   - start_service[11] + start_service[23] >= -999909.1632951815
 StartServiceRobots[12,1]: - 999999 v[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999915.2024796238
 StartServiceRobots[12,2]: - 999999 v[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 999999 v[12,3] - service_time[12]
   + start_service[3] - start_service[12] >= -999985.7650834119
 StartServiceRobots[12,4]: - 999999 v[12,4] - service_time[12]
   + start_service[4] - start_service[12] >= -999961.403612743
 StartServiceRobots[12,5]: - 999999 v[12,5] - service_time[12]
   + start_service[5] - start_service[12] >= -999953.4152526929
 StartServiceRobots[12,6]: - 999999 v[12,6] - service_time[12]
   + start_service[6] - start_service[12] >= -999926.3485351843
 StartServiceRobots[12,7]: - 999999 v[12,7] - service_time[12]
   + start_service[7] - start_service[12] >= -999952.1869815146
 StartServiceRobots[12,8]: - 999999 v[12,8] - service_time[12]
   + start_service[8] - start_service[12] >= -999957.5030249554
 StartServiceRobots[12,9]: - 999999 v[12,9] - service_time[12]
   + start_service[9] - start_service[12] >= -999927.6383894689
 StartServiceRobots[12,10]: - 999999 v[12,10] - service_time[12]
   + start_service[10] - start_service[12] >= -999942.098220245
 StartServiceRobots[12,11]: - 999999 v[12,11] - service_time[12]
   + start_service[11] - start_service[12] >= -999968.3063737304
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 999999 v[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999985.7650834119
 StartServiceRobots[12,14]: - 999999 v[12,14] - service_time[12]
   - start_service[12] + start_service[14] >= -999961.403612743
 StartServiceRobots[12,15]: - 999999 v[12,15] - service_time[12]
   - start_service[12] + start_service[15] >= -999953.4152526929
 StartServiceRobots[12,16]: - 999999 v[12,16] - service_time[12]
   - start_service[12] + start_service[16] >= -999926.3485351843
 StartServiceRobots[12,17]: - 999999 v[12,17] - service_time[12]
   - start_service[12] + start_service[17] >= -999952.1869815146
 StartServiceRobots[12,18]: - 999999 v[12,18] - service_time[12]
   - start_service[12] + start_service[18] >= -999957.5030249554
 StartServiceRobots[12,19]: - 999999 v[12,19] - service_time[12]
   - start_service[12] + start_service[19] >= -999927.6383894689
 StartServiceRobots[12,20]: - 999999 v[12,20] - service_time[12]
   - start_service[12] + start_service[20] >= -999942.098220245
 StartServiceRobots[12,21]: - 999999 v[12,21] - service_time[12]
   - start_service[12] + start_service[21] >= -999968.3063737304
 StartServiceRobots[12,22]: - 999999 v[12,22] - service_time[12]
   - start_service[12] + start_service[22] >= -999915.2024796238
 StartServiceRobots[12,23]: - 999999 v[12,23] - service_time[12]
   - start_service[12] + start_service[23] >= -999915.2024796238
 StartServiceRobots[13,1]: - 999999 v[13,1] - service_time[13]
   + start_service[1] - start_service[13] >= -999928.4369415202
 StartServiceRobots[13,2]: - 999999 v[13,2] - service_time[13]
   + start_service[2] - start_service[13] >= -999985.7650834119
 StartServiceRobots[13,3]: - 999999 v[13,3] - service_time[13]
   + start_service[3] - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 999999 v[13,4] - service_time[13]
   + start_service[4] - start_service[13] >= -999953.0826375335
 StartServiceRobots[13,5]: - 999999 v[13,5] - service_time[13]
   + start_service[5] - start_service[13] >= -999966.5641297009
 StartServiceRobots[13,6]: - 999999 v[13,6] - service_time[13]
   + start_service[6] - start_service[13] >= -999937.2267384126
 StartServiceRobots[13,7]: - 999999 v[13,7] - service_time[13]
   + start_service[7] - start_service[13] >= -999963.4959134692
 StartServiceRobots[13,8]: - 999999 v[13,8] - service_time[13]
   + start_service[8] - start_service[13] >= -999958.5450983534
 StartServiceRobots[13,9]: - 999999 v[13,9] - service_time[13]
   + start_service[9] - start_service[13] >= -999939.1789229446
 StartServiceRobots[13,10]: - 999999 v[13,10] - service_time[13]
   + start_service[10] - start_service[13] >= -999955.1505151036
 StartServiceRobots[13,11]: - 999999 v[13,11] - service_time[13]
   + start_service[11] - start_service[13] >= -999965.4157057379
 StartServiceRobots[13,12]: - 999999 v[13,12] - service_time[13]
   + start_service[12] - start_service[13] >= -999985.7650834119
 StartServiceRobots[13,13]: - 999999 v[13,13] - service_time[13]
   >= -999999
 StartServiceRobots[13,14]: - 999999 v[13,14] - service_time[13]
   - start_service[13] + start_service[14] >= -999953.0826375335
 StartServiceRobots[13,15]: - 999999 v[13,15] - service_time[13]
   - start_service[13] + start_service[15] >= -999966.5641297009
 StartServiceRobots[13,16]: - 999999 v[13,16] - service_time[13]
   - start_service[13] + start_service[16] >= -999937.2267384126
 StartServiceRobots[13,17]: - 999999 v[13,17] - service_time[13]
   - start_service[13] + start_service[17] >= -999963.4959134692
 StartServiceRobots[13,18]: - 999999 v[13,18] - service_time[13]
   - start_service[13] + start_service[18] >= -999958.5450983534
 StartServiceRobots[13,19]: - 999999 v[13,19] - service_time[13]
   - start_service[13] + start_service[19] >= -999939.1789229446
 StartServiceRobots[13,20]: - 999999 v[13,20] - service_time[13]
   - start_service[13] + start_service[20] >= -999955.1505151036
 StartServiceRobots[13,21]: - 999999 v[13,21] - service_time[13]
   - start_service[13] + start_service[21] >= -999965.4157057379
 StartServiceRobots[13,22]: - 999999 v[13,22] - service_time[13]
   - start_service[13] + start_service[22] >= -999928.4369415202
 StartServiceRobots[13,23]: - 999999 v[13,23] - service_time[13]
   - start_service[13] + start_service[23] >= -999928.4369415202
 StartServiceRobots[14,1]: - 999999 v[14,1] - service_time[14]
   + start_service[1] - start_service[14] >= -999890.901035983
 StartServiceRobots[14,2]: - 999999 v[14,2] - service_time[14]
   + start_service[2] - start_service[14] >= -999961.403612743
 StartServiceRobots[14,3]: - 999999 v[14,3] - service_time[14]
   + start_service[3] - start_service[14] >= -999953.0826375335
 StartServiceRobots[14,4]: - 999999 v[14,4] - service_time[14]
   + start_service[4] - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 999999 v[14,5] - service_time[14]
   + start_service[5] - start_service[14] >= -999924.4748369423
 StartServiceRobots[14,6]: - 999999 v[14,6] - service_time[14]
   + start_service[6] - start_service[14] >= -999917.2796655721
 StartServiceRobots[14,7]: - 999999 v[14,7] - service_time[14]
   + start_service[7] - start_service[14] >= -999936.6080027135
 StartServiceRobots[14,8]: - 999999 v[14,8] - service_time[14]
   + start_service[8] - start_service[14] >= -999968.3875979197
 StartServiceRobots[14,9]: - 999999 v[14,9] - service_time[14]
   + start_service[9] - start_service[14] >= -999893.8315006682
 StartServiceRobots[14,10]: - 999999 v[14,10] - service_time[14]
   + start_service[10] - start_service[14] >= -999919.4256871791
 StartServiceRobots[14,11]: - 999999 v[14,11] - service_time[14]
   + start_service[11] - start_service[14] >= -999933.3662834174
 StartServiceRobots[14,12]: - 999999 v[14,12] - service_time[14]
   + start_service[12] - start_service[14] >= -999961.403612743
 StartServiceRobots[14,13]: - 999999 v[14,13] - service_time[14]
   + start_service[13] - start_service[14] >= -999953.0826375335
 StartServiceRobots[14,14]: - 999999 v[14,14] - service_time[14]
   >= -999999
 StartServiceRobots[14,15]: - 999999 v[14,15] - service_time[14]
   - start_service[14] + start_service[15] >= -999924.4748369423
 StartServiceRobots[14,16]: - 999999 v[14,16] - service_time[14]
   - start_service[14] + start_service[16] >= -999917.2796655721
 StartServiceRobots[14,17]: - 999999 v[14,17] - service_time[14]
   - start_service[14] + start_service[17] >= -999936.6080027135
 StartServiceRobots[14,18]: - 999999 v[14,18] - service_time[14]
   - start_service[14] + start_service[18] >= -999968.3875979197
 StartServiceRobots[14,19]: - 999999 v[14,19] - service_time[14]
   - start_service[14] + start_service[19] >= -999893.8315006682
 StartServiceRobots[14,20]: - 999999 v[14,20] - service_time[14]
   - start_service[14] + start_service[20] >= -999919.4256871791
 StartServiceRobots[14,21]: - 999999 v[14,21] - service_time[14]
   - start_service[14] + start_service[21] >= -999933.3662834174
 StartServiceRobots[14,22]: - 999999 v[14,22] - service_time[14]
   - start_service[14] + start_service[22] >= -999890.901035983
 StartServiceRobots[14,23]: - 999999 v[14,23] - service_time[14]
   - start_service[14] + start_service[23] >= -999890.901035983
 StartServiceRobots[15,1]: - 999999 v[15,1] - service_time[15]
   + start_service[1] - start_service[15] >= -999960.2532812204
 StartServiceRobots[15,2]: - 999999 v[15,2] - service_time[15]
   + start_service[2] - start_service[15] >= -999953.4152526929
 StartServiceRobots[15,3]: - 999999 v[15,3] - service_time[15]
   + start_service[3] - start_service[15] >= -999966.5641297009
 StartServiceRobots[15,4]: - 999999 v[15,4] - service_time[15]
   + start_service[4] - start_service[15] >= -999924.4748369423
 StartServiceRobots[15,5]: - 999999 v[15,5] - service_time[15]
   + start_service[5] - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 999999 v[15,6] - service_time[15]
   + start_service[6] - start_service[15] >= -999953.4384806296
 StartServiceRobots[15,7]: - 999999 v[15,7] - service_time[15]
   + start_service[7] - start_service[15] >= -999973.2688131087
 StartServiceRobots[15,8]: - 999999 v[15,8] - service_time[15]
   + start_service[8] - start_service[15] >= -999941.9077472225
 StartServiceRobots[15,9]: - 999999 v[15,9] - service_time[15]
   + start_service[9] - start_service[15] >= -999965.8333693258
 StartServiceRobots[15,10]: - 999999 v[15,10] - service_time[15]
   + start_service[10] - start_service[15] >= -999982.2907259347
 StartServiceRobots[15,11]: - 999999 v[15,11] - service_time[15]
   + start_service[11] - start_service[15] >= -999946.1993118351
 StartServiceRobots[15,12]: - 999999 v[15,12] - service_time[15]
   + start_service[12] - start_service[15] >= -999953.4152526929
 StartServiceRobots[15,13]: - 999999 v[15,13] - service_time[15]
   + start_service[13] - start_service[15] >= -999966.5641297009
 StartServiceRobots[15,14]: - 999999 v[15,14] - service_time[15]
   + start_service[14] - start_service[15] >= -999924.4748369423
 StartServiceRobots[15,15]: - 999999 v[15,15] - service_time[15]
   >= -999999
 StartServiceRobots[15,16]: - 999999 v[15,16] - service_time[15]
   - start_service[15] + start_service[16] >= -999953.4384806296
 StartServiceRobots[15,17]: - 999999 v[15,17] - service_time[15]
   - start_service[15] + start_service[17] >= -999973.2688131087
 StartServiceRobots[15,18]: - 999999 v[15,18] - service_time[15]
   - start_service[15] + start_service[18] >= -999941.9077472225
 StartServiceRobots[15,19]: - 999999 v[15,19] - service_time[15]
   - start_service[15] + start_service[19] >= -999965.8333693258
 StartServiceRobots[15,20]: - 999999 v[15,20] - service_time[15]
   - start_service[15] + start_service[20] >= -999982.2907259347
 StartServiceRobots[15,21]: - 999999 v[15,21] - service_time[15]
   - start_service[15] + start_service[21] >= -999946.1993118351
 StartServiceRobots[15,22]: - 999999 v[15,22] - service_time[15]
   - start_service[15] + start_service[22] >= -999960.2532812204
 StartServiceRobots[15,23]: - 999999 v[15,23] - service_time[15]
   - start_service[15] + start_service[23] >= -999960.2532812204
 StartServiceRobots[16,1]: - 999999 v[16,1] - service_time[16]
   + start_service[1] - start_service[16] >= -999955.5912513951
 StartServiceRobots[16,2]: - 999999 v[16,2] - service_time[16]
   + start_service[2] - start_service[16] >= -999926.3485351843
 StartServiceRobots[16,3]: - 999999 v[16,3] - service_time[16]
   + start_service[3] - start_service[16] >= -999937.2267384126
 StartServiceRobots[16,4]: - 999999 v[16,4] - service_time[16]
   + start_service[4] - start_service[16] >= -999917.2796655721
 StartServiceRobots[16,5]: - 999999 v[16,5] - service_time[16]
   + start_service[5] - start_service[16] >= -999953.4384806296
 StartServiceRobots[16,6]: - 999999 v[16,6] - service_time[16]
   + start_service[6] - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 999999 v[16,7] - service_time[16]
   + start_service[7] - start_service[16] >= -999972.6979548086
 StartServiceRobots[16,8]: - 999999 v[16,8] - service_time[16]
   + start_service[8] - start_service[16] >= -999946.9351560561
 StartServiceRobots[16,9]: - 999999 v[16,9] - service_time[16]
   + start_service[9] - start_service[16] >= -999928.9293963189
 StartServiceRobots[16,10]: - 999999 v[16,10] - service_time[16]
   + start_service[10] - start_service[16] >= -999968.6558211173
 StartServiceRobots[16,11]: - 999999 v[16,11] - service_time[16]
   + start_service[11] - start_service[16] >= -999906.2176650148
 StartServiceRobots[16,12]: - 999999 v[16,12] - service_time[16]
   + start_service[12] - start_service[16] >= -999926.3485351843
 StartServiceRobots[16,13]: - 999999 v[16,13] - service_time[16]
   + start_service[13] - start_service[16] >= -999937.2267384126
 StartServiceRobots[16,14]: - 999999 v[16,14] - service_time[16]
   + start_service[14] - start_service[16] >= -999917.2796655721
 StartServiceRobots[16,15]: - 999999 v[16,15] - service_time[16]
   + start_service[15] - start_service[16] >= -999953.4384806296
 StartServiceRobots[16,16]: - 999999 v[16,16] - service_time[16]
   >= -999999
 StartServiceRobots[16,17]: - 999999 v[16,17] - service_time[16]
   - start_service[16] + start_service[17] >= -999972.6979548086
 StartServiceRobots[16,18]: - 999999 v[16,18] - service_time[16]
   - start_service[16] + start_service[18] >= -999946.9351560561
 StartServiceRobots[16,19]: - 999999 v[16,19] - service_time[16]
   - start_service[16] + start_service[19] >= -999928.9293963189
 StartServiceRobots[16,20]: - 999999 v[16,20] - service_time[16]
   - start_service[16] + start_service[20] >= -999968.6558211173
 StartServiceRobots[16,21]: - 999999 v[16,21] - service_time[16]
   - start_service[16] + start_service[21] >= -999906.2176650148
 StartServiceRobots[16,22]: - 999999 v[16,22] - service_time[16]
   - start_service[16] + start_service[22] >= -999955.5912513951
 StartServiceRobots[16,23]: - 999999 v[16,23] - service_time[16]
   - start_service[16] + start_service[23] >= -999955.5912513951
 StartServiceRobots[17,1]: - 999999 v[17,1] - service_time[17]
   + start_service[1] - start_service[17] >= -999952.271273898
 StartServiceRobots[17,2]: - 999999 v[17,2] - service_time[17]
   + start_service[2] - start_service[17] >= -999952.1869815146
 StartServiceRobots[17,3]: - 999999 v[17,3] - service_time[17]
   + start_service[3] - start_service[17] >= -999963.4959134692
 StartServiceRobots[17,4]: - 999999 v[17,4] - service_time[17]
   + start_service[4] - start_service[17] >= -999936.6080027135
 StartServiceRobots[17,5]: - 999999 v[17,5] - service_time[17]
   + start_service[5] - start_service[17] >= -999973.2688131087
 StartServiceRobots[17,6]: - 999999 v[17,6] - service_time[17]
   + start_service[6] - start_service[17] >= -999972.6979548086
 StartServiceRobots[17,7]: - 999999 v[17,7] - service_time[17]
   + start_service[7] - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 999999 v[17,8] - service_time[17]
   + start_service[8] - start_service[17] >= -999961.7262354469
 StartServiceRobots[17,9]: - 999999 v[17,9] - service_time[17]
   + start_service[9] - start_service[17] >= -999941.5599706656
 StartServiceRobots[17,10]: - 999999 v[17,10] - service_time[17]
   + start_service[10] - start_service[17] >= -999979.6262282272
 StartServiceRobots[17,11]: - 999999 v[17,11] - service_time[17]
   + start_service[11] - start_service[17] >= -999932.1978989958
 StartServiceRobots[17,12]: - 999999 v[17,12] - service_time[17]
   + start_service[12] - start_service[17] >= -999952.1869815146
 StartServiceRobots[17,13]: - 999999 v[17,13] - service_time[17]
   + start_service[13] - start_service[17] >= -999963.4959134692
 StartServiceRobots[17,14]: - 999999 v[17,14] - service_time[17]
   + start_service[14] - start_service[17] >= -999936.6080027135
 StartServiceRobots[17,15]: - 999999 v[17,15] - service_time[17]
   + start_service[15] - start_service[17] >= -999973.2688131087
 StartServiceRobots[17,16]: - 999999 v[17,16] - service_time[17]
   + start_service[16] - start_service[17] >= -999972.6979548086
 StartServiceRobots[17,17]: - 999999 v[17,17] - service_time[17]
   >= -999999
 StartServiceRobots[17,18]: - 999999 v[17,18] - service_time[17]
   - start_service[17] + start_service[18] >= -999961.7262354469
 StartServiceRobots[17,19]: - 999999 v[17,19] - service_time[17]
   - start_service[17] + start_service[19] >= -999941.5599706656
 StartServiceRobots[17,20]: - 999999 v[17,20] - service_time[17]
   - start_service[17] + start_service[20] >= -999979.6262282272
 StartServiceRobots[17,21]: - 999999 v[17,21] - service_time[17]
   - start_service[17] + start_service[21] >= -999932.1978989958
 StartServiceRobots[17,22]: - 999999 v[17,22] - service_time[17]
   - start_service[17] + start_service[22] >= -999952.271273898
 StartServiceRobots[17,23]: - 999999 v[17,23] - service_time[17]
   - start_service[17] + start_service[23] >= -999952.271273898
 StartServiceRobots[18,1]: - 999999 v[18,1] - service_time[18]
   + start_service[1] - start_service[18] >= -999915.0779296518
 StartServiceRobots[18,2]: - 999999 v[18,2] - service_time[18]
   + start_service[2] - start_service[18] >= -999957.5030249554
 StartServiceRobots[18,3]: - 999999 v[18,3] - service_time[18]
   + start_service[3] - start_service[18] >= -999958.5450983534
 StartServiceRobots[18,4]: - 999999 v[18,4] - service_time[18]
   + start_service[4] - start_service[18] >= -999968.3875979197
 StartServiceRobots[18,5]: - 999999 v[18,5] - service_time[18]
   + start_service[5] - start_service[18] >= -999941.9077472225
 StartServiceRobots[18,6]: - 999999 v[18,6] - service_time[18]
   + start_service[6] - start_service[18] >= -999946.9351560561
 StartServiceRobots[18,7]: - 999999 v[18,7] - service_time[18]
   + start_service[7] - start_service[18] >= -999961.7262354469
 StartServiceRobots[18,8]: - 999999 v[18,8] - service_time[18]
   + start_service[8] - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 999999 v[18,9] - service_time[18]
   + start_service[9] - start_service[18] >= -999908.812105495
 StartServiceRobots[18,10]: - 999999 v[18,10] - service_time[18]
   + start_service[10] - start_service[18] >= -999942.5652146442
 StartServiceRobots[18,11]: - 999999 v[18,11] - service_time[18]
   + start_service[11] - start_service[18] >= -999927.2610558096
 StartServiceRobots[18,12]: - 999999 v[18,12] - service_time[18]
   + start_service[12] - start_service[18] >= -999957.5030249554
 StartServiceRobots[18,13]: - 999999 v[18,13] - service_time[18]
   + start_service[13] - start_service[18] >= -999958.5450983534
 StartServiceRobots[18,14]: - 999999 v[18,14] - service_time[18]
   + start_service[14] - start_service[18] >= -999968.3875979197
 StartServiceRobots[18,15]: - 999999 v[18,15] - service_time[18]
   + start_service[15] - start_service[18] >= -999941.9077472225
 StartServiceRobots[18,16]: - 999999 v[18,16] - service_time[18]
   + start_service[16] - start_service[18] >= -999946.9351560561
 StartServiceRobots[18,17]: - 999999 v[18,17] - service_time[18]
   + start_service[17] - start_service[18] >= -999961.7262354469
 StartServiceRobots[18,18]: - 999999 v[18,18] - service_time[18]
   >= -999999
 StartServiceRobots[18,19]: - 999999 v[18,19] - service_time[18]
   - start_service[18] + start_service[19] >= -999908.812105495
 StartServiceRobots[18,20]: - 999999 v[18,20] - service_time[18]
   - start_service[18] + start_service[20] >= -999942.5652146442
 StartServiceRobots[18,21]: - 999999 v[18,21] - service_time[18]
   - start_service[18] + start_service[21] >= -999927.2610558096
 StartServiceRobots[18,22]: - 999999 v[18,22] - service_time[18]
   - start_service[18] + start_service[22] >= -999915.0779296518
 StartServiceRobots[18,23]: - 999999 v[18,23] - service_time[18]
   - start_service[18] + start_service[23] >= -999915.0779296518
 StartServiceRobots[19,1]: - 999999 v[19,1] - service_time[19]
   + start_service[1] - start_service[19] >= -999960.7764686092
 StartServiceRobots[19,2]: - 999999 v[19,2] - service_time[19]
   + start_service[2] - start_service[19] >= -999927.6383894689
 StartServiceRobots[19,3]: - 999999 v[19,3] - service_time[19]
   + start_service[3] - start_service[19] >= -999939.1789229446
 StartServiceRobots[19,4]: - 999999 v[19,4] - service_time[19]
   + start_service[4] - start_service[19] >= -999893.8315006682
 StartServiceRobots[19,5]: - 999999 v[19,5] - service_time[19]
   + start_service[5] - start_service[19] >= -999965.8333693258
 StartServiceRobots[19,6]: - 999999 v[19,6] - service_time[19]
   + start_service[6] - start_service[19] >= -999928.9293963189
 StartServiceRobots[19,7]: - 999999 v[19,7] - service_time[19]
   + start_service[7] - start_service[19] >= -999941.5599706656
 StartServiceRobots[19,8]: - 999999 v[19,8] - service_time[19]
   + start_service[8] - start_service[19] >= -999908.812105495
 StartServiceRobots[19,9]: - 999999 v[19,9] - service_time[19]
   + start_service[9] - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 999999 v[19,10] - service_time[19]
   + start_service[10] - start_service[19] >= -999958.0904356453
 StartServiceRobots[19,11]: - 999999 v[19,11] - service_time[19]
   + start_service[11] - start_service[19] >= -999934.7740920822
 StartServiceRobots[19,12]: - 999999 v[19,12] - service_time[19]
   + start_service[12] - start_service[19] >= -999927.6383894689
 StartServiceRobots[19,13]: - 999999 v[19,13] - service_time[19]
   + start_service[13] - start_service[19] >= -999939.1789229446
 StartServiceRobots[19,14]: - 999999 v[19,14] - service_time[19]
   + start_service[14] - start_service[19] >= -999893.8315006682
 StartServiceRobots[19,15]: - 999999 v[19,15] - service_time[19]
   + start_service[15] - start_service[19] >= -999965.8333693258
 StartServiceRobots[19,16]: - 999999 v[19,16] - service_time[19]
   + start_service[16] - start_service[19] >= -999928.9293963189
 StartServiceRobots[19,17]: - 999999 v[19,17] - service_time[19]
   + start_service[17] - start_service[19] >= -999941.5599706656
 StartServiceRobots[19,18]: - 999999 v[19,18] - service_time[19]
   + start_service[18] - start_service[19] >= -999908.812105495
 StartServiceRobots[19,19]: - 999999 v[19,19] - service_time[19]
   >= -999999
 StartServiceRobots[19,20]: - 999999 v[19,20] - service_time[19]
   - start_service[19] + start_service[20] >= -999958.0904356453
 StartServiceRobots[19,21]: - 999999 v[19,21] - service_time[19]
   - start_service[19] + start_service[21] >= -999934.7740920822
 StartServiceRobots[19,22]: - 999999 v[19,22] - service_time[19]
   - start_service[19] + start_service[22] >= -999960.7764686092
 StartServiceRobots[19,23]: - 999999 v[19,23] - service_time[19]
   - start_service[19] + start_service[23] >= -999960.7764686092
 StartServiceRobots[20,1]: - 999999 v[20,1] - service_time[20]
   + start_service[1] - start_service[20] >= -999970.4639211913
 StartServiceRobots[20,2]: - 999999 v[20,2] - service_time[20]
   + start_service[2] - start_service[20] >= -999942.098220245
 StartServiceRobots[20,3]: - 999999 v[20,3] - service_time[20]
   + start_service[3] - start_service[20] >= -999955.1505151036
 StartServiceRobots[20,4]: - 999999 v[20,4] - service_time[20]
   + start_service[4] - start_service[20] >= -999919.4256871791
 StartServiceRobots[20,5]: - 999999 v[20,5] - service_time[20]
   + start_service[5] - start_service[20] >= -999982.2907259347
 StartServiceRobots[20,6]: - 999999 v[20,6] - service_time[20]
   + start_service[6] - start_service[20] >= -999968.6558211173
 StartServiceRobots[20,7]: - 999999 v[20,7] - service_time[20]
   + start_service[7] - start_service[20] >= -999979.6262282272
 StartServiceRobots[20,8]: - 999999 v[20,8] - service_time[20]
   + start_service[8] - start_service[20] >= -999942.5652146442
 StartServiceRobots[20,9]: - 999999 v[20,9] - service_time[20]
   + start_service[9] - start_service[20] >= -999958.0904356453
 StartServiceRobots[20,10]: - 999999 v[20,10] - service_time[20]
   + start_service[10] - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 999999 v[20,11] - service_time[20]
   + start_service[11] - start_service[20] >= -999930.2057225219
 StartServiceRobots[20,12]: - 999999 v[20,12] - service_time[20]
   + start_service[12] - start_service[20] >= -999942.098220245
 StartServiceRobots[20,13]: - 999999 v[20,13] - service_time[20]
   + start_service[13] - start_service[20] >= -999955.1505151036
 StartServiceRobots[20,14]: - 999999 v[20,14] - service_time[20]
   + start_service[14] - start_service[20] >= -999919.4256871791
 StartServiceRobots[20,15]: - 999999 v[20,15] - service_time[20]
   + start_service[15] - start_service[20] >= -999982.2907259347
 StartServiceRobots[20,16]: - 999999 v[20,16] - service_time[20]
   + start_service[16] - start_service[20] >= -999968.6558211173
 StartServiceRobots[20,17]: - 999999 v[20,17] - service_time[20]
   + start_service[17] - start_service[20] >= -999979.6262282272
 StartServiceRobots[20,18]: - 999999 v[20,18] - service_time[20]
   + start_service[18] - start_service[20] >= -999942.5652146442
 StartServiceRobots[20,19]: - 999999 v[20,19] - service_time[20]
   + start_service[19] - start_service[20] >= -999958.0904356453
 StartServiceRobots[20,20]: - 999999 v[20,20] - service_time[20]
   >= -999999
 StartServiceRobots[20,21]: - 999999 v[20,21] - service_time[20]
   - start_service[20] + start_service[21] >= -999930.2057225219
 StartServiceRobots[20,22]: - 999999 v[20,22] - service_time[20]
   - start_service[20] + start_service[22] >= -999970.4639211913
 StartServiceRobots[20,23]: - 999999 v[20,23] - service_time[20]
   - start_service[20] + start_service[23] >= -999970.4639211913
 StartServiceRobots[21,1]: - 999999 v[21,1] - service_time[21]
   + start_service[1] - start_service[21] >= -999909.1632951815
 StartServiceRobots[21,2]: - 999999 v[21,2] - service_time[21]
   + start_service[2] - start_service[21] >= -999968.3063737304
 StartServiceRobots[21,3]: - 999999 v[21,3] - service_time[21]
   + start_service[3] - start_service[21] >= -999965.4157057379
 StartServiceRobots[21,4]: - 999999 v[21,4] - service_time[21]
   + start_service[4] - start_service[21] >= -999933.3662834174
 StartServiceRobots[21,5]: - 999999 v[21,5] - service_time[21]
   + start_service[5] - start_service[21] >= -999946.1993118351
 StartServiceRobots[21,6]: - 999999 v[21,6] - service_time[21]
   + start_service[6] - start_service[21] >= -999906.2176650148
 StartServiceRobots[21,7]: - 999999 v[21,7] - service_time[21]
   + start_service[7] - start_service[21] >= -999932.1978989958
 StartServiceRobots[21,8]: - 999999 v[21,8] - service_time[21]
   + start_service[8] - start_service[21] >= -999927.2610558096
 StartServiceRobots[21,9]: - 999999 v[21,9] - service_time[21]
   + start_service[9] - start_service[21] >= -999934.7740920822
 StartServiceRobots[21,10]: - 999999 v[21,10] - service_time[21]
   + start_service[10] - start_service[21] >= -999930.2057225219
 StartServiceRobots[21,11]: - 999999 v[21,11] - service_time[21]
   + start_service[11] - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 999999 v[21,12] - service_time[21]
   + start_service[12] - start_service[21] >= -999968.3063737304
 StartServiceRobots[21,13]: - 999999 v[21,13] - service_time[21]
   + start_service[13] - start_service[21] >= -999965.4157057379
 StartServiceRobots[21,14]: - 999999 v[21,14] - service_time[21]
   + start_service[14] - start_service[21] >= -999933.3662834174
 StartServiceRobots[21,15]: - 999999 v[21,15] - service_time[21]
   + start_service[15] - start_service[21] >= -999946.1993118351
 StartServiceRobots[21,16]: - 999999 v[21,16] - service_time[21]
   + start_service[16] - start_service[21] >= -999906.2176650148
 StartServiceRobots[21,17]: - 999999 v[21,17] - service_time[21]
   + start_service[17] - start_service[21] >= -999932.1978989958
 StartServiceRobots[21,18]: - 999999 v[21,18] - service_time[21]
   + start_service[18] - start_service[21] >= -999927.2610558096
 StartServiceRobots[21,19]: - 999999 v[21,19] - service_time[21]
   + start_service[19] - start_service[21] >= -999934.7740920822
 StartServiceRobots[21,20]: - 999999 v[21,20] - service_time[21]
   + start_service[20] - start_service[21] >= -999930.2057225219
 StartServiceRobots[21,21]: - 999999 v[21,21] - service_time[21]
   >= -999999
 StartServiceRobots[21,22]: - 999999 v[21,22] - service_time[21]
   - start_service[21] + start_service[22] >= -999909.1632951815
 StartServiceRobots[21,23]: - 999999 v[21,23] - service_time[21]
   - start_service[21] + start_service[23] >= -999909.1632951815
 StartServiceRobots[22,1]: - 999999 v[22,1] - service_time[22]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 999999 v[22,2] - service_time[22]
   + start_service[2] - start_service[22] >= -999915.2024796238
 StartServiceRobots[22,3]: - 999999 v[22,3] - service_time[22]
   + start_service[3] - start_service[22] >= -999928.4369415202
 StartServiceRobots[22,4]: - 999999 v[22,4] - service_time[22]
   + start_service[4] - start_service[22] >= -999890.901035983
 StartServiceRobots[22,5]: - 999999 v[22,5] - service_time[22]
   + start_service[5] - start_service[22] >= -999960.2532812204
 StartServiceRobots[22,6]: - 999999 v[22,6] - service_time[22]
   + start_service[6] - start_service[22] >= -999955.5912513951
 StartServiceRobots[22,7]: - 999999 v[22,7] - service_time[22]
   + start_service[7] - start_service[22] >= -999952.271273898
 StartServiceRobots[22,8]: - 999999 v[22,8] - service_time[22]
   + start_service[8] - start_service[22] >= -999915.0779296518
 StartServiceRobots[22,9]: - 999999 v[22,9] - service_time[22]
   + start_service[9] - start_service[22] >= -999960.7764686092
 StartServiceRobots[22,10]: - 999999 v[22,10] - service_time[22]
   + start_service[10] - start_service[22] >= -999970.4639211913
 StartServiceRobots[22,11]: - 999999 v[22,11] - service_time[22]
   + start_service[11] - start_service[22] >= -999909.1632951815
 StartServiceRobots[22,12]: - 999999 v[22,12] - service_time[22]
   + start_service[12] - start_service[22] >= -999915.2024796238
 StartServiceRobots[22,13]: - 999999 v[22,13] - service_time[22]
   + start_service[13] - start_service[22] >= -999928.4369415202
 StartServiceRobots[22,14]: - 999999 v[22,14] - service_time[22]
   + start_service[14] - start_service[22] >= -999890.901035983
 StartServiceRobots[22,15]: - 999999 v[22,15] - service_time[22]
   + start_service[15] - start_service[22] >= -999960.2532812204
 StartServiceRobots[22,16]: - 999999 v[22,16] - service_time[22]
   + start_service[16] - start_service[22] >= -999955.5912513951
 StartServiceRobots[22,17]: - 999999 v[22,17] - service_time[22]
   + start_service[17] - start_service[22] >= -999952.271273898
 StartServiceRobots[22,18]: - 999999 v[22,18] - service_time[22]
   + start_service[18] - start_service[22] >= -999915.0779296518
 StartServiceRobots[22,19]: - 999999 v[22,19] - service_time[22]
   + start_service[19] - start_service[22] >= -999960.7764686092
 StartServiceRobots[22,20]: - 999999 v[22,20] - service_time[22]
   + start_service[20] - start_service[22] >= -999970.4639211913
 StartServiceRobots[22,21]: - 999999 v[22,21] - service_time[22]
   + start_service[21] - start_service[22] >= -999909.1632951815
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
 SettingServiceTime[2]: - 330 y[2,1] - 165 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 270 y[3,1] - 135 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 490 y[4,1] - 245 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 290 y[5,1] - 145 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 220 y[6,1] - 110 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 490 y[7,1] - 245 y[7,2] + service_time[7] = 0
 SettingServiceTime[8]: - 200 y[8,1] - 100 y[8,2] + service_time[8] = 0
 SettingServiceTime[9]: - 330 y[9,1] - 165 y[9,2] + service_time[9] = 0
 SettingServiceTime[10]: - 410 y[10,1] - 205 y[10,2] + service_time[10]
   = 0
 SettingServiceTime[11]: - 410 y[11,1] - 205 y[11,2] + service_time[11]
   = 0
 AverageTimeRobots: - 11.97107433944812 w[0,2] - 10.08043692569259 w[0,3]
   - 15.44270914528464 w[0,4] - 5.535245539939781 w[0,5]
   - 6.201249800700316 w[0,6] - 6.675532300287585 w[0,7]
   - 11.9888671926039 w[0,8] - 5.46050448438836 w[0,9]
   - 4.076582686952778 w[0,10] - 12.8338149740684 w[0,11]
   - 11.97107433944812 w[0,12] - 10.08043692569259 w[0,13]
   - 15.44270914528464 w[0,14] - 5.535245539939781 w[0,15]
   - 6.201249800700316 w[0,16] - 6.675532300287585 w[0,17]
   - 11.9888671926039 w[0,18] - 5.46050448438836 w[0,19]
   - 4.076582686952778 w[0,20] - 12.8338149740684 w[0,21]
   - 11.97107433944812 w[1,2] - 10.08043692569259 w[1,3]
   - 15.44270914528464 w[1,4] - 5.535245539939781 w[1,5]
   - 6.201249800700316 w[1,6] - 6.675532300287585 w[1,7]
   - 11.9888671926039 w[1,8] - 5.46050448438836 w[1,9]
   - 4.076582686952778 w[1,10] - 12.8338149740684 w[1,11]
   - 11.97107433944812 w[1,12] - 10.08043692569259 w[1,13]
   - 15.44270914528464 w[1,14] - 5.535245539939781 w[1,15]
   - 6.201249800700316 w[1,16] - 6.675532300287585 w[1,17]
   - 11.9888671926039 w[1,18] - 5.46050448438836 w[1,19]
   - 4.076582686952778 w[1,20] - 12.8338149740684 w[1,21]
   - 11.97107433944812 w[2,1] - 1.890702369715544 w[2,3]
   - 5.370912465281908 w[2,4] - 6.512106758163363 w[2,5]
   - 10.3787806879528 w[2,6] - 6.68757406934321 w[2,7]
   - 5.928139292094226 w[2,8] - 10.19451579016234 w[2,9]
   - 8.128825679286388 w[2,10] - 4.384803752805398 w[2,11]
   - 1.890702369715544 w[2,13] - 5.370912465281908 w[2,14]
   - 6.512106758163363 w[2,15] - 10.3787806879528 w[2,16]
   - 6.68757406934321 w[2,17] - 5.928139292094226 w[2,18]
   - 10.19451579016234 w[2,19] - 8.128825679286388 w[2,20]
   - 4.384803752805398 w[2,21] - 11.97107433944812 w[2,22]
   - 11.97107433944812 w[2,23] - 10.08043692569259 w[3,1]
   - 1.890702369715544 w[3,2] - 6.559623209498309 w[3,4]
   - 4.633695757011266 w[3,5] - 8.824751655344983 w[3,6]
   - 5.072012361531529 w[3,7] - 5.779271663800008 w[3,8]
   - 8.545868150771193 w[3,9] - 6.264212128054732 w[3,10]
   - 4.797756323164394 w[3,11] - 1.890702369715544 w[3,12]
   - 6.559623209498309 w[3,14] - 4.633695757011266 w[3,15]
   - 8.824751655344983 w[3,16] - 5.072012361531529 w[3,17]
   - 5.779271663800008 w[3,18] - 8.545868150771193 w[3,19]
   - 6.264212128054732 w[3,20] - 4.797756323164394 w[3,21]
   - 10.08043692569259 w[3,22] - 10.08043692569259 w[3,23]
   - 15.44270914528464 w[4,1] - 5.370912465281908 w[4,2]
   - 6.559623209498309 w[4,3] - 10.64645186538574 w[4,5]
   - 11.67433348970445 w[4,6] - 8.913142469494517 w[4,7]
   - 4.373200297189911 w[4,8] - 15.02407133311026 w[4,9]
   - 11.36775897442117 w[4,10] - 9.376245226071875 w[4,11]
   - 5.370912465281908 w[4,12] - 6.559623209498309 w[4,13]
   - 10.64645186538574 w[4,15] - 11.67433348970445 w[4,16]
   - 8.913142469494517 w[4,17] - 4.373200297189911 w[4,18]
   - 15.02407133311026 w[4,19] - 11.36775897442117 w[4,20]
   - 9.376245226071875 w[4,21] - 15.44270914528464 w[4,22]
   - 15.44270914528464 w[4,23] - 5.535245539939781 w[5,1]
   - 6.512106758163363 w[5,2] - 4.633695757011266 w[5,3]
   - 10.64645186538574 w[5,4] - 6.508788481480966 w[5,6]
   - 3.675883841619328 w[5,7] - 8.156036111079189 w[5,8]
   - 4.738090096313387 w[5,9] - 2.387039152183696 w[5,10]
   - 7.542955452120559 w[5,11] - 6.512106758163363 w[5,12]
   - 4.633695757011266 w[5,13] - 10.64645186538574 w[5,14]
   - 6.508788481480966 w[5,16] - 3.675883841619328 w[5,17]
   - 8.156036111079189 w[5,18] - 4.738090096313387 w[5,19]
   - 2.387039152183696 w[5,20] - 7.542955452120559 w[5,21]
   - 5.535245539939781 w[5,22] - 5.535245539939781 w[5,23]
   - 6.201249800700316 w[6,1] - 10.3787806879528 w[6,2]
   - 8.824751655344983 w[6,3] - 11.67433348970445 w[6,4]
   - 6.508788481480966 w[6,5] - 3.757435027339874 w[6,7]
   - 7.437834849136306 w[6,8] - 10.01008624017036 w[6,9]
   - 4.334882697528515 w[6,10] - 13.2546192835992 w[6,11]
   - 10.3787806879528 w[6,12] - 8.824751655344983 w[6,13]
   - 11.67433348970445 w[6,14] - 6.508788481480966 w[6,15]
   - 3.757435027339874 w[6,17] - 7.437834849136306 w[6,18]
   - 10.01008624017036 w[6,19] - 4.334882697528515 w[6,20]
   - 13.2546192835992 w[6,21] - 6.201249800700316 w[6,22]
   - 6.201249800700316 w[6,23] - 6.675532300287585 w[7,1]
   - 6.68757406934321 w[7,2] - 5.072012361531529 w[7,3]
   - 8.913142469494517 w[7,4] - 3.675883841619328 w[7,5]
   - 3.757435027339874 w[7,6] - 5.324823507597555 w[7,8]
   - 8.205718476339987 w[7,9] - 2.767681681833075 w[7,10]
   - 9.543157286307183 w[7,11] - 6.68757406934321 w[7,12]
   - 5.072012361531529 w[7,13] - 8.913142469494517 w[7,14]
   - 3.675883841619328 w[7,15] - 3.757435027339874 w[7,16]
   - 5.324823507597555 w[7,18] - 8.205718476339987 w[7,19]
   - 2.767681681833075 w[7,20] - 9.543157286307183 w[7,21]
   - 6.675532300287585 w[7,22] - 6.675532300287585 w[7,23]
   - 11.9888671926039 w[8,1] - 5.928139292094226 w[8,2]
   - 5.779271663800008 w[8,3] - 4.373200297189911 w[8,4]
   - 8.156036111079189 w[8,5] - 7.437834849136306 w[8,6]
   - 5.324823507597555 w[8,7] - 12.88398492928328 w[8,9]
   - 8.062112193681891 w[8,10] - 10.24842059863156 w[8,11]
   - 5.928139292094226 w[8,12] - 5.779271663800008 w[8,13]
   - 4.373200297189911 w[8,14] - 8.156036111079189 w[8,15]
   - 7.437834849136306 w[8,16] - 5.324823507597555 w[8,17]
   - 12.88398492928328 w[8,19] - 8.062112193681891 w[8,20]
   - 10.24842059863156 w[8,21] - 11.9888671926039 w[8,22]
   - 11.9888671926039 w[8,23] - 5.46050448438836 w[9,1]
   - 10.19451579016234 w[9,2] - 8.545868150771193 w[9,3]
   - 15.02407133311026 w[9,4] - 4.738090096313387 w[9,5]
   - 10.01008624017036 w[9,6] - 8.205718476339987 w[9,7]
   - 12.88398492928328 w[9,8] - 5.844223479241355 w[9,10]
   - 9.175129702551075 w[9,11] - 10.19451579016234 w[9,12]
   - 8.545868150771193 w[9,13] - 15.02407133311026 w[9,14]
   - 4.738090096313387 w[9,15] - 10.01008624017036 w[9,16]
   - 8.205718476339987 w[9,17] - 12.88398492928328 w[9,18]
   - 5.844223479241355 w[9,20] - 9.175129702551075 w[9,21]
   - 5.46050448438836 w[9,22] - 5.46050448438836 w[9,23]
   - 4.076582686952778 w[10,1] - 8.128825679286388 w[10,2]
   - 6.264212128054732 w[10,3] - 11.36775897442117 w[10,4]
   - 2.387039152183696 w[10,5] - 4.334882697528515 w[10,6]
   - 2.767681681833075 w[10,7] - 8.062112193681891 w[10,8]
   - 5.844223479241355 w[10,9] - 9.827753925442083 w[10,11]
   - 8.128825679286388 w[10,12] - 6.264212128054732 w[10,13]
   - 11.36775897442117 w[10,14] - 2.387039152183696 w[10,15]
   - 4.334882697528515 w[10,16] - 2.767681681833075 w[10,17]
   - 8.062112193681891 w[10,18] - 5.844223479241355 w[10,19]
   - 9.827753925442083 w[10,21] - 4.076582686952778 w[10,22]
   - 4.076582686952778 w[10,23] - 12.8338149740684 w[11,1]
   - 4.384803752805398 w[11,2] - 4.797756323164394 w[11,3]
   - 9.376245226071875 w[11,4] - 7.542955452120559 w[11,5]
   - 13.2546192835992 w[11,6] - 9.543157286307183 w[11,7]
   - 10.24842059863156 w[11,8] - 9.175129702551075 w[11,9]
   - 9.827753925442083 w[11,10] - 4.384803752805398 w[11,12]
   - 4.797756323164394 w[11,13] - 9.376245226071875 w[11,14]
   - 7.542955452120559 w[11,15] - 13.2546192835992 w[11,16]
   - 9.543157286307183 w[11,17] - 10.24842059863156 w[11,18]
   - 9.175129702551075 w[11,19] - 9.827753925442083 w[11,20]
   - 12.8338149740684 w[11,22] - 12.8338149740684 w[11,23]
   - 11.97107433944812 w[12,1] - 1.890702369715544 w[12,3]
   - 5.370912465281908 w[12,4] - 6.512106758163363 w[12,5]
   - 10.3787806879528 w[12,6] - 6.68757406934321 w[12,7]
   - 5.928139292094226 w[12,8] - 10.19451579016234 w[12,9]
   - 8.128825679286388 w[12,10] - 4.384803752805398 w[12,11]
   - 1.890702369715544 w[12,13] - 5.370912465281908 w[12,14]
   - 6.512106758163363 w[12,15] - 10.3787806879528 w[12,16]
   - 6.68757406934321 w[12,17] - 5.928139292094226 w[12,18]
   - 10.19451579016234 w[12,19] - 8.128825679286388 w[12,20]
   - 4.384803752805398 w[12,21] - 11.97107433944812 w[12,22]
   - 11.97107433944812 w[12,23] - 10.08043692569259 w[13,1]
   - 1.890702369715544 w[13,2] - 6.559623209498309 w[13,4]
   - 4.633695757011266 w[13,5] - 8.824751655344983 w[13,6]
   - 5.072012361531529 w[13,7] - 5.779271663800008 w[13,8]
   - 8.545868150771193 w[13,9] - 6.264212128054732 w[13,10]
   - 4.797756323164394 w[13,11] - 1.890702369715544 w[13,12]
   - 6.559623209498309 w[13,14] - 4.633695757011266 w[13,15]
   - 8.824751655344983 w[13,16] - 5.072012361531529 w[13,17]
   - 5.779271663800008 w[13,18] - 8.545868150771193 w[13,19]
   - 6.264212128054732 w[13,20] - 4.797756323164394 w[13,21]
   - 10.08043692569259 w[13,22] - 10.08043692569259 w[13,23]
   - 15.44270914528464 w[14,1] - 5.370912465281908 w[14,2]
   - 6.559623209498309 w[14,3] - 10.64645186538574 w[14,5]
   - 11.67433348970445 w[14,6] - 8.913142469494517 w[14,7]
   - 4.373200297189911 w[14,8] - 15.02407133311026 w[14,9]
   - 11.36775897442117 w[14,10] - 9.376245226071875 w[14,11]
   - 5.370912465281908 w[14,12] - 6.559623209498309 w[14,13]
   - 10.64645186538574 w[14,15] - 11.67433348970445 w[14,16]
   - 8.913142469494517 w[14,17] - 4.373200297189911 w[14,18]
   - 15.02407133311026 w[14,19] - 11.36775897442117 w[14,20]
   - 9.376245226071875 w[14,21] - 15.44270914528464 w[14,22]
   - 15.44270914528464 w[14,23] - 5.535245539939781 w[15,1]
   - 6.512106758163363 w[15,2] - 4.633695757011266 w[15,3]
   - 10.64645186538574 w[15,4] - 6.508788481480966 w[15,6]
   - 3.675883841619328 w[15,7] - 8.156036111079189 w[15,8]
   - 4.738090096313387 w[15,9] - 2.387039152183696 w[15,10]
   - 7.542955452120559 w[15,11] - 6.512106758163363 w[15,12]
   - 4.633695757011266 w[15,13] - 10.64645186538574 w[15,14]
   - 6.508788481480966 w[15,16] - 3.675883841619328 w[15,17]
   - 8.156036111079189 w[15,18] - 4.738090096313387 w[15,19]
   - 2.387039152183696 w[15,20] - 7.542955452120559 w[15,21]
   - 5.535245539939781 w[15,22] - 5.535245539939781 w[15,23]
   - 6.201249800700316 w[16,1] - 10.3787806879528 w[16,2]
   - 8.824751655344983 w[16,3] - 11.67433348970445 w[16,4]
   - 6.508788481480966 w[16,5] - 3.757435027339874 w[16,7]
   - 7.437834849136306 w[16,8] - 10.01008624017036 w[16,9]
   - 4.334882697528515 w[16,10] - 13.2546192835992 w[16,11]
   - 10.3787806879528 w[16,12] - 8.824751655344983 w[16,13]
   - 11.67433348970445 w[16,14] - 6.508788481480966 w[16,15]
   - 3.757435027339874 w[16,17] - 7.437834849136306 w[16,18]
   - 10.01008624017036 w[16,19] - 4.334882697528515 w[16,20]
   - 13.2546192835992 w[16,21] - 6.201249800700316 w[16,22]
   - 6.201249800700316 w[16,23] - 6.675532300287585 w[17,1]
   - 6.68757406934321 w[17,2] - 5.072012361531529 w[17,3]
   - 8.913142469494517 w[17,4] - 3.675883841619328 w[17,5]
   - 3.757435027339874 w[17,6] - 5.324823507597555 w[17,8]
   - 8.205718476339987 w[17,9] - 2.767681681833075 w[17,10]
   - 9.543157286307183 w[17,11] - 6.68757406934321 w[17,12]
   - 5.072012361531529 w[17,13] - 8.913142469494517 w[17,14]
   - 3.675883841619328 w[17,15] - 3.757435027339874 w[17,16]
   - 5.324823507597555 w[17,18] - 8.205718476339987 w[17,19]
   - 2.767681681833075 w[17,20] - 9.543157286307183 w[17,21]
   - 6.675532300287585 w[17,22] - 6.675532300287585 w[17,23]
   - 11.9888671926039 w[18,1] - 5.928139292094226 w[18,2]
   - 5.779271663800008 w[18,3] - 4.373200297189911 w[18,4]
   - 8.156036111079189 w[18,5] - 7.437834849136306 w[18,6]
   - 5.324823507597555 w[18,7] - 12.88398492928328 w[18,9]
   - 8.062112193681891 w[18,10] - 10.24842059863156 w[18,11]
   - 5.928139292094226 w[18,12] - 5.779271663800008 w[18,13]
   - 4.373200297189911 w[18,14] - 8.156036111079189 w[18,15]
   - 7.437834849136306 w[18,16] - 5.324823507597555 w[18,17]
   - 12.88398492928328 w[18,19] - 8.062112193681891 w[18,20]
   - 10.24842059863156 w[18,21] - 11.9888671926039 w[18,22]
   - 11.9888671926039 w[18,23] - 5.46050448438836 w[19,1]
   - 10.19451579016234 w[19,2] - 8.545868150771193 w[19,3]
   - 15.02407133311026 w[19,4] - 4.738090096313387 w[19,5]
   - 10.01008624017036 w[19,6] - 8.205718476339987 w[19,7]
   - 12.88398492928328 w[19,8] - 5.844223479241355 w[19,10]
   - 9.175129702551075 w[19,11] - 10.19451579016234 w[19,12]
   - 8.545868150771193 w[19,13] - 15.02407133311026 w[19,14]
   - 4.738090096313387 w[19,15] - 10.01008624017036 w[19,16]
   - 8.205718476339987 w[19,17] - 12.88398492928328 w[19,18]
   - 5.844223479241355 w[19,20] - 9.175129702551075 w[19,21]
   - 5.46050448438836 w[19,22] - 5.46050448438836 w[19,23]
   - 4.076582686952778 w[20,1] - 8.128825679286388 w[20,2]
   - 6.264212128054732 w[20,3] - 11.36775897442117 w[20,4]
   - 2.387039152183696 w[20,5] - 4.334882697528515 w[20,6]
   - 2.767681681833075 w[20,7] - 8.062112193681891 w[20,8]
   - 5.844223479241355 w[20,9] - 9.827753925442083 w[20,11]
   - 8.128825679286388 w[20,12] - 6.264212128054732 w[20,13]
   - 11.36775897442117 w[20,14] - 2.387039152183696 w[20,15]
   - 4.334882697528515 w[20,16] - 2.767681681833075 w[20,17]
   - 8.062112193681891 w[20,18] - 5.844223479241355 w[20,19]
   - 9.827753925442083 w[20,21] - 4.076582686952778 w[20,22]
   - 4.076582686952778 w[20,23] - 12.8338149740684 w[21,1]
   - 4.384803752805398 w[21,2] - 4.797756323164394 w[21,3]
   - 9.376245226071875 w[21,4] - 7.542955452120559 w[21,5]
   - 13.2546192835992 w[21,6] - 9.543157286307183 w[21,7]
   - 10.24842059863156 w[21,8] - 9.175129702551075 w[21,9]
   - 9.827753925442083 w[21,10] - 4.384803752805398 w[21,12]
   - 4.797756323164394 w[21,13] - 9.376245226071875 w[21,14]
   - 7.542955452120559 w[21,15] - 13.2546192835992 w[21,16]
   - 9.543157286307183 w[21,17] - 10.24842059863156 w[21,18]
   - 9.175129702551075 w[21,19] - 9.827753925442083 w[21,20]
   - 12.8338149740684 w[21,22] - 12.8338149740684 w[21,23]
   - 11.97107433944812 w[22,2] - 10.08043692569259 w[22,3]
   - 15.44270914528464 w[22,4] - 5.535245539939781 w[22,5]
   - 6.201249800700316 w[22,6] - 6.675532300287585 w[22,7]
   - 11.9888671926039 w[22,8] - 5.46050448438836 w[22,9]
   - 4.076582686952778 w[22,10] - 12.8338149740684 w[22,11]
   - 11.97107433944812 w[22,12] - 10.08043692569259 w[22,13]
   - 15.44270914528464 w[22,14] - 5.535245539939781 w[22,15]
   - 6.201249800700316 w[22,16] - 6.675532300287585 w[22,17]
   - 11.9888671926039 w[22,18] - 5.46050448438836 w[22,19]
   - 4.076582686952778 w[22,20] - 12.8338149740684 w[22,21]
   - 47.14285714285714 y[2,1] - 94.28571428571428 y[2,2]
   - 38.57142857142857 y[3,1] - 77.14285714285714 y[3,2] - 70 y[4,1]
   - 140 y[4,2] - 41.42857142857142 y[5,1] - 82.85714285714285 y[5,2]
   - 31.42857142857143 y[6,1] - 62.85714285714285 y[6,2] - 70 y[7,1]
   - 140 y[7,2] - 28.57142857142857 y[8,1] - 57.14285714285714 y[8,2]
   - 47.14285714285714 y[9,1] - 94.28571428571428 y[9,2]
   - 58.57142857142857 y[10,1] - 117.1428571428571 y[10,2]
   - 58.57142857142857 y[11,1] - 117.1428571428571 y[11,2]
   + start_service[23] >= 0
 AverageTimeVehices: - 27.93250679204562 x[0,2] - 23.52101949328271 x[0,3]
   - 36.03298800566417 x[0,4] - 12.91557292652615 x[0,5]
   - 14.46958286830074 x[0,6] - 15.57624203400437 x[0,7]
   - 27.97402344940909 x[0,8] - 12.74117713023951 x[0,9]
   - 9.512026269556483 x[0,10] - 29.94556827282627 x[0,11]
   - 27.93250679204562 x[0,12] - 23.52101949328271 x[0,13]
   - 36.03298800566417 x[0,14] - 12.91557292652615 x[0,15]
   - 14.46958286830074 x[0,16] - 15.57624203400437 x[0,17]
   - 27.97402344940909 x[0,18] - 12.74117713023951 x[0,19]
   - 9.512026269556483 x[0,20] - 29.94556827282627 x[0,21]
   - 27.93250679204562 x[1,2] - 23.52101949328271 x[1,3]
   - 36.03298800566417 x[1,4] - 12.91557292652615 x[1,5]
   - 14.46958286830074 x[1,6] - 15.57624203400437 x[1,7]
   - 27.97402344940909 x[1,8] - 12.74117713023951 x[1,9]
   - 9.512026269556483 x[1,10] - 29.94556827282627 x[1,11]
   - 27.93250679204562 x[1,12] - 23.52101949328271 x[1,13]
   - 36.03298800566417 x[1,14] - 12.91557292652615 x[1,15]
   - 14.46958286830074 x[1,16] - 15.57624203400437 x[1,17]
   - 27.97402344940909 x[1,18] - 12.74117713023951 x[1,19]
   - 9.512026269556483 x[1,20] - 29.94556827282627 x[1,21]
   - 27.93250679204562 x[2,1] - 4.411638862669602 x[2,3]
   - 12.53212908565779 x[2,4] - 15.19491576904785 x[2,5]
   - 24.21715493855654 x[2,6] - 15.60433949513416 x[2,7]
   - 13.83232501488653 x[2,8] - 23.7872035103788 x[2,9]
   - 18.9672599183349 x[2,10] - 10.23120875654593 x[2,11]
   - 4.411638862669602 x[2,13] - 12.53212908565779 x[2,14]
   - 15.19491576904785 x[2,15] - 24.21715493855654 x[2,16]
   - 15.60433949513416 x[2,17] - 13.83232501488653 x[2,18]
   - 23.7872035103788 x[2,19] - 18.9672599183349 x[2,20]
   - 10.23120875654593 x[2,21] - 27.93250679204562 x[2,22]
   - 27.93250679204562 x[2,23] - 23.52101949328271 x[3,1]
   - 4.411638862669602 x[3,2] - 15.30578748882939 x[3,4]
   - 10.81195676635962 x[3,5] - 20.59108719580496 x[3,6]
   - 11.83469551024023 x[3,7] - 13.48496721553335 x[3,8]
   - 19.94035901846612 x[3,9] - 14.61649496546104 x[3,10]
   - 11.19476475405025 x[3,11] - 4.411638862669602 x[3,12]
   - 15.30578748882939 x[3,14] - 10.81195676635962 x[3,15]
   - 20.59108719580496 x[3,16] - 11.83469551024023 x[3,17]
   - 13.48496721553335 x[3,18] - 19.94035901846612 x[3,19]
   - 14.61649496546104 x[3,20] - 11.19476475405025 x[3,21]
   - 23.52101949328271 x[3,22] - 23.52101949328271 x[3,23]
   - 36.03298800566417 x[4,1] - 12.53212908565779 x[4,2]
   - 15.30578748882939 x[4,3] - 24.84172101923338 x[4,5]
   - 27.24011147597705 x[4,6] - 20.79733242882054 x[4,7]
   - 10.20413402677646 x[4,8] - 35.05616644392395 x[4,9]
   - 26.52477094031607 x[4,10] - 21.87790552750104 x[4,11]
   - 12.53212908565779 x[4,12] - 15.30578748882939 x[4,13]
   - 24.84172101923338 x[4,15] - 27.24011147597705 x[4,16]
   - 20.79733242882054 x[4,17] - 10.20413402677646 x[4,18]
   - 35.05616644392395 x[4,19] - 26.52477094031607 x[4,20]
   - 21.87790552750104 x[4,21] - 36.03298800566417 x[4,22]
   - 36.03298800566417 x[4,23] - 12.91557292652615 x[5,1]
   - 15.19491576904785 x[5,2] - 10.81195676635962 x[5,3]
   - 24.84172101923338 x[5,4] - 15.18717312345559 x[5,6]
   - 8.577062297111764 x[5,7] - 19.03075092585144 x[5,8]
   - 11.05554355806457 x[5,9] - 5.569758021761957 x[5,10]
   - 17.6002293882813 x[5,11] - 15.19491576904785 x[5,12]
   - 10.81195676635962 x[5,13] - 24.84172101923338 x[5,14]
   - 15.18717312345559 x[5,16] - 8.577062297111764 x[5,17]
   - 19.03075092585144 x[5,18] - 11.05554355806457 x[5,19]
   - 5.569758021761957 x[5,20] - 17.6002293882813 x[5,21]
   - 12.91557292652615 x[5,22] - 12.91557292652615 x[5,23]
   - 14.46958286830074 x[6,1] - 24.21715493855654 x[6,2]
   - 20.59108719580496 x[6,3] - 27.24011147597705 x[6,4]
   - 15.18717312345559 x[6,5] - 8.767348397126373 x[6,7]
   - 17.35494798131805 x[6,8] - 23.35686789373084 x[6,9]
   - 10.1147262942332 x[6,10] - 30.92744499506479 x[6,11]
   - 24.21715493855654 x[6,12] - 20.59108719580496 x[6,13]
   - 27.24011147597705 x[6,14] - 15.18717312345559 x[6,15]
   - 8.767348397126373 x[6,17] - 17.35494798131805 x[6,18]
   - 23.35686789373084 x[6,19] - 10.1147262942332 x[6,20]
   - 30.92744499506479 x[6,21] - 14.46958286830074 x[6,22]
   - 14.46958286830074 x[6,23] - 15.57624203400437 x[7,1]
   - 15.60433949513416 x[7,2] - 11.83469551024023 x[7,3]
   - 20.79733242882054 x[7,4] - 8.577062297111764 x[7,5]
   - 8.767348397126373 x[7,6] - 12.4245881843943 x[7,8]
   - 19.1466764447933 x[7,9] - 6.457923924277173 x[7,10]
   - 22.26736700138343 x[7,11] - 15.60433949513416 x[7,12]
   - 11.83469551024023 x[7,13] - 20.79733242882054 x[7,14]
   - 8.577062297111764 x[7,15] - 8.767348397126373 x[7,16]
   - 12.4245881843943 x[7,18] - 19.1466764447933 x[7,19]
   - 6.457923924277173 x[7,20] - 22.26736700138343 x[7,21]
   - 15.57624203400437 x[7,22] - 15.57624203400437 x[7,23]
   - 27.97402344940909 x[8,1] - 13.83232501488653 x[8,2]
   - 13.48496721553335 x[8,3] - 10.20413402677646 x[8,4]
   - 19.03075092585144 x[8,5] - 17.35494798131805 x[8,6]
   - 12.4245881843943 x[8,7] - 30.06263150166098 x[8,9]
   - 18.81159511859108 x[8,10] - 23.91298139680697 x[8,11]
   - 13.83232501488653 x[8,12] - 13.48496721553335 x[8,13]
   - 10.20413402677646 x[8,14] - 19.03075092585144 x[8,15]
   - 17.35494798131805 x[8,16] - 12.4245881843943 x[8,17]
   - 30.06263150166098 x[8,19] - 18.81159511859108 x[8,20]
   - 23.91298139680697 x[8,21] - 27.97402344940909 x[8,22]
   - 27.97402344940909 x[8,23] - 12.74117713023951 x[9,1]
   - 23.7872035103788 x[9,2] - 19.94035901846612 x[9,3]
   - 35.05616644392395 x[9,4] - 11.05554355806457 x[9,5]
   - 23.35686789373084 x[9,6] - 19.1466764447933 x[9,7]
   - 30.06263150166098 x[9,8] - 13.63652145156316 x[9,10]
   - 21.40863597261917 x[9,11] - 23.7872035103788 x[9,12]
   - 19.94035901846612 x[9,13] - 35.05616644392395 x[9,14]
   - 11.05554355806457 x[9,15] - 23.35686789373084 x[9,16]
   - 19.1466764447933 x[9,17] - 30.06263150166098 x[9,18]
   - 13.63652145156316 x[9,20] - 21.40863597261917 x[9,21]
   - 12.74117713023951 x[9,22] - 12.74117713023951 x[9,23]
   - 9.512026269556483 x[10,1] - 18.9672599183349 x[10,2]
   - 14.61649496546104 x[10,3] - 26.52477094031607 x[10,4]
   - 5.569758021761957 x[10,5] - 10.1147262942332 x[10,6]
   - 6.457923924277173 x[10,7] - 18.81159511859108 x[10,8]
   - 13.63652145156316 x[10,9] - 22.93142582603153 x[10,11]
   - 18.9672599183349 x[10,12] - 14.61649496546104 x[10,13]
   - 26.52477094031607 x[10,14] - 5.569758021761957 x[10,15]
   - 10.1147262942332 x[10,16] - 6.457923924277173 x[10,17]
   - 18.81159511859108 x[10,18] - 13.63652145156316 x[10,19]
   - 22.93142582603153 x[10,21] - 9.512026269556483 x[10,22]
   - 9.512026269556483 x[10,23] - 29.94556827282627 x[11,1]
   - 10.23120875654593 x[11,2] - 11.19476475405025 x[11,3]
   - 21.87790552750104 x[11,4] - 17.6002293882813 x[11,5]
   - 30.92744499506479 x[11,6] - 22.26736700138343 x[11,7]
   - 23.91298139680697 x[11,8] - 21.40863597261917 x[11,9]
   - 22.93142582603153 x[11,10] - 10.23120875654593 x[11,12]
   - 11.19476475405025 x[11,13] - 21.87790552750104 x[11,14]
   - 17.6002293882813 x[11,15] - 30.92744499506479 x[11,16]
   - 22.26736700138343 x[11,17] - 23.91298139680697 x[11,18]
   - 21.40863597261917 x[11,19] - 22.93142582603153 x[11,20]
   - 29.94556827282627 x[11,22] - 29.94556827282627 x[11,23]
   - 27.93250679204562 x[12,1] - 4.411638862669602 x[12,3]
   - 12.53212908565779 x[12,4] - 15.19491576904785 x[12,5]
   - 24.21715493855654 x[12,6] - 15.60433949513416 x[12,7]
   - 13.83232501488653 x[12,8] - 23.7872035103788 x[12,9]
   - 18.9672599183349 x[12,10] - 10.23120875654593 x[12,11]
   - 4.411638862669602 x[12,13] - 12.53212908565779 x[12,14]
   - 15.19491576904785 x[12,15] - 24.21715493855654 x[12,16]
   - 15.60433949513416 x[12,17] - 13.83232501488653 x[12,18]
   - 23.7872035103788 x[12,19] - 18.9672599183349 x[12,20]
   - 10.23120875654593 x[12,21] - 27.93250679204562 x[12,22]
   - 27.93250679204562 x[12,23] - 23.52101949328271 x[13,1]
   - 4.411638862669602 x[13,2] - 15.30578748882939 x[13,4]
   - 10.81195676635962 x[13,5] - 20.59108719580496 x[13,6]
   - 11.83469551024023 x[13,7] - 13.48496721553335 x[13,8]
   - 19.94035901846612 x[13,9] - 14.61649496546104 x[13,10]
   - 11.19476475405025 x[13,11] - 4.411638862669602 x[13,12]
   - 15.30578748882939 x[13,14] - 10.81195676635962 x[13,15]
   - 20.59108719580496 x[13,16] - 11.83469551024023 x[13,17]
   - 13.48496721553335 x[13,18] - 19.94035901846612 x[13,19]
   - 14.61649496546104 x[13,20] - 11.19476475405025 x[13,21]
   - 23.52101949328271 x[13,22] - 23.52101949328271 x[13,23]
   - 36.03298800566417 x[14,1] - 12.53212908565779 x[14,2]
   - 15.30578748882939 x[14,3] - 24.84172101923338 x[14,5]
   - 27.24011147597705 x[14,6] - 20.79733242882054 x[14,7]
   - 10.20413402677646 x[14,8] - 35.05616644392395 x[14,9]
   - 26.52477094031607 x[14,10] - 21.87790552750104 x[14,11]
   - 12.53212908565779 x[14,12] - 15.30578748882939 x[14,13]
   - 24.84172101923338 x[14,15] - 27.24011147597705 x[14,16]
   - 20.79733242882054 x[14,17] - 10.20413402677646 x[14,18]
   - 35.05616644392395 x[14,19] - 26.52477094031607 x[14,20]
   - 21.87790552750104 x[14,21] - 36.03298800566417 x[14,22]
   - 36.03298800566417 x[14,23] - 12.91557292652615 x[15,1]
   - 15.19491576904785 x[15,2] - 10.81195676635962 x[15,3]
   - 24.84172101923338 x[15,4] - 15.18717312345559 x[15,6]
   - 8.577062297111764 x[15,7] - 19.03075092585144 x[15,8]
   - 11.05554355806457 x[15,9] - 5.569758021761957 x[15,10]
   - 17.6002293882813 x[15,11] - 15.19491576904785 x[15,12]
   - 10.81195676635962 x[15,13] - 24.84172101923338 x[15,14]
   - 15.18717312345559 x[15,16] - 8.577062297111764 x[15,17]
   - 19.03075092585144 x[15,18] - 11.05554355806457 x[15,19]
   - 5.569758021761957 x[15,20] - 17.6002293882813 x[15,21]
   - 12.91557292652615 x[15,22] - 12.91557292652615 x[15,23]
   - 14.46958286830074 x[16,1] - 24.21715493855654 x[16,2]
   - 20.59108719580496 x[16,3] - 27.24011147597705 x[16,4]
   - 15.18717312345559 x[16,5] - 8.767348397126373 x[16,7]
   - 17.35494798131805 x[16,8] - 23.35686789373084 x[16,9]
   - 10.1147262942332 x[16,10] - 30.92744499506479 x[16,11]
   - 24.21715493855654 x[16,12] - 20.59108719580496 x[16,13]
   - 27.24011147597705 x[16,14] - 15.18717312345559 x[16,15]
   - 8.767348397126373 x[16,17] - 17.35494798131805 x[16,18]
   - 23.35686789373084 x[16,19] - 10.1147262942332 x[16,20]
   - 30.92744499506479 x[16,21] - 14.46958286830074 x[16,22]
   - 14.46958286830074 x[16,23] - 15.57624203400437 x[17,1]
   - 15.60433949513416 x[17,2] - 11.83469551024023 x[17,3]
   - 20.79733242882054 x[17,4] - 8.577062297111764 x[17,5]
   - 8.767348397126373 x[17,6] - 12.4245881843943 x[17,8]
   - 19.1466764447933 x[17,9] - 6.457923924277173 x[17,10]
   - 22.26736700138343 x[17,11] - 15.60433949513416 x[17,12]
   - 11.83469551024023 x[17,13] - 20.79733242882054 x[17,14]
   - 8.577062297111764 x[17,15] - 8.767348397126373 x[17,16]
   - 12.4245881843943 x[17,18] - 19.1466764447933 x[17,19]
   - 6.457923924277173 x[17,20] - 22.26736700138343 x[17,21]
   - 15.57624203400437 x[17,22] - 15.57624203400437 x[17,23]
   - 27.97402344940909 x[18,1] - 13.83232501488653 x[18,2]
   - 13.48496721553335 x[18,3] - 10.20413402677646 x[18,4]
   - 19.03075092585144 x[18,5] - 17.35494798131805 x[18,6]
   - 12.4245881843943 x[18,7] - 30.06263150166098 x[18,9]
   - 18.81159511859108 x[18,10] - 23.91298139680697 x[18,11]
   - 13.83232501488653 x[18,12] - 13.48496721553335 x[18,13]
   - 10.20413402677646 x[18,14] - 19.03075092585144 x[18,15]
   - 17.35494798131805 x[18,16] - 12.4245881843943 x[18,17]
   - 30.06263150166098 x[18,19] - 18.81159511859108 x[18,20]
   - 23.91298139680697 x[18,21] - 27.97402344940909 x[18,22]
   - 27.97402344940909 x[18,23] - 12.74117713023951 x[19,1]
   - 23.7872035103788 x[19,2] - 19.94035901846612 x[19,3]
   - 35.05616644392395 x[19,4] - 11.05554355806457 x[19,5]
   - 23.35686789373084 x[19,6] - 19.1466764447933 x[19,7]
   - 30.06263150166098 x[19,8] - 13.63652145156316 x[19,10]
   - 21.40863597261917 x[19,11] - 23.7872035103788 x[19,12]
   - 19.94035901846612 x[19,13] - 35.05616644392395 x[19,14]
   - 11.05554355806457 x[19,15] - 23.35686789373084 x[19,16]
   - 19.1466764447933 x[19,17] - 30.06263150166098 x[19,18]
   - 13.63652145156316 x[19,20] - 21.40863597261917 x[19,21]
   - 12.74117713023951 x[19,22] - 12.74117713023951 x[19,23]
   - 9.512026269556483 x[20,1] - 18.9672599183349 x[20,2]
   - 14.61649496546104 x[20,3] - 26.52477094031607 x[20,4]
   - 5.569758021761957 x[20,5] - 10.1147262942332 x[20,6]
   - 6.457923924277173 x[20,7] - 18.81159511859108 x[20,8]
   - 13.63652145156316 x[20,9] - 22.93142582603153 x[20,11]
   - 18.9672599183349 x[20,12] - 14.61649496546104 x[20,13]
   - 26.52477094031607 x[20,14] - 5.569758021761957 x[20,15]
   - 10.1147262942332 x[20,16] - 6.457923924277173 x[20,17]
   - 18.81159511859108 x[20,18] - 13.63652145156316 x[20,19]
   - 22.93142582603153 x[20,21] - 9.512026269556483 x[20,22]
   - 9.512026269556483 x[20,23] - 29.94556827282627 x[21,1]
   - 10.23120875654593 x[21,2] - 11.19476475405025 x[21,3]
   - 21.87790552750104 x[21,4] - 17.6002293882813 x[21,5]
   - 30.92744499506479 x[21,6] - 22.26736700138343 x[21,7]
   - 23.91298139680697 x[21,8] - 21.40863597261917 x[21,9]
   - 22.93142582603153 x[21,10] - 10.23120875654593 x[21,12]
   - 11.19476475405025 x[21,13] - 21.87790552750104 x[21,14]
   - 17.6002293882813 x[21,15] - 30.92744499506479 x[21,16]
   - 22.26736700138343 x[21,17] - 23.91298139680697 x[21,18]
   - 21.40863597261917 x[21,19] - 22.93142582603153 x[21,20]
   - 29.94556827282627 x[21,22] - 29.94556827282627 x[21,23]
   - 27.93250679204562 x[22,2] - 23.52101949328271 x[22,3]
   - 36.03298800566417 x[22,4] - 12.91557292652615 x[22,5]
   - 14.46958286830074 x[22,6] - 15.57624203400437 x[22,7]
   - 27.97402344940909 x[22,8] - 12.74117713023951 x[22,9]
   - 9.512026269556483 x[22,10] - 29.94556827282627 x[22,11]
   - 27.93250679204562 x[22,12] - 23.52101949328271 x[22,13]
   - 36.03298800566417 x[22,14] - 12.91557292652615 x[22,15]
   - 14.46958286830074 x[22,16] - 15.57624203400437 x[22,17]
   - 27.97402344940909 x[22,18] - 12.74117713023951 x[22,19]
   - 9.512026269556483 x[22,20] - 29.94556827282627 x[22,21]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[0]: - 167.5950407522737 x[0,2]
   - 141.1261169596962 x[0,3] - 216.197928033985 x[0,4]
   - 77.49343755915693 x[0,5] - 86.81749720980443 x[0,6]
   - 93.4574522040262 x[0,7] - 167.8441406964546 x[0,8]
   - 76.44706278143704 x[0,9] - 57.0721576173389 x[0,10]
   - 179.6734096369576 x[0,11] - 167.5950407522737 x[0,12]
   - 141.1261169596962 x[0,13] - 216.197928033985 x[0,14]
   - 77.49343755915693 x[0,15] - 86.81749720980443 x[0,16]
   - 93.4574522040262 x[0,17] - 167.8441406964546 x[0,18]
   - 76.44706278143704 x[0,19] - 57.0721576173389 x[0,20]
   - 179.6734096369576 x[0,21] - service_time[0] - start_service[0]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[1]: - 167.5950407522737 x[1,2]
   - 141.1261169596962 x[1,3] - 216.197928033985 x[1,4]
   - 77.49343755915693 x[1,5] - 86.81749720980443 x[1,6]
   - 93.4574522040262 x[1,7] - 167.8441406964546 x[1,8]
   - 76.44706278143704 x[1,9] - 57.0721576173389 x[1,10]
   - 179.6734096369576 x[1,11] - 167.5950407522737 x[1,12]
   - 141.1261169596962 x[1,13] - 216.197928033985 x[1,14]
   - 77.49343755915693 x[1,15] - 86.81749720980443 x[1,16]
   - 93.4574522040262 x[1,17] - 167.8441406964546 x[1,18]
   - 76.44706278143704 x[1,19] - 57.0721576173389 x[1,20]
   - 179.6734096369576 x[1,21] - service_time[1] - start_service[1]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[2]: - 83.79752037613687 x[2,1]
   - 83.79752037613687 x[2,2] - 83.79797506785692 x[2,3]
   - 145.6953512739659 x[2,4] - 84.331466086722 x[2,5]
   - 116.0602134205718 x[2,6] - 93.54174458741556 x[2,7]
   - 125.4190453928869 x[2,8] - 109.5851419218549 x[2,9]
   - 85.43785856367417 x[2,10] - 120.5303310881166 x[2,11]
   - 83.79752037613687 x[2,12] - 83.79797506785692 x[2,13]
   - 145.6953512739659 x[2,14] - 84.331466086722 x[2,15]
   - 116.0602134205718 x[2,16] - 93.54174458741556 x[2,17]
   - 125.4190453928869 x[2,18] - 109.5851419218549 x[2,19]
   - 85.43785856367417 x[2,20] - 120.5303310881166 x[2,21]
   - 83.79752037613687 x[2,22] - 83.79752037613687 x[2,23]
   - service_time[2] - start_service[2] + start_service[23] >= 0
 LowerBoundRoutingVehicle[3]: - 70.56305847984812 x[3,1]
   - 97.03243696414569 x[3,2] - 70.56305847984812 x[3,3]
   - 154.0163264834807 x[3,4] - 71.18258907865733 x[3,5]
   - 105.1820101923171 x[3,6] - 82.23281263273381 x[3,7]
   - 124.3769719948273 x[3,8] - 98.04460844611688 x[3,9]
   - 72.38556370505258 x[3,10] - 123.4209990806296 x[3,11]
   - 97.03243696414569 x[3,12] - 70.56305847984812 x[3,13]
   - 154.0163264834807 x[3,14] - 71.18258907865733 x[3,15]
   - 105.1820101923171 x[3,16] - 82.23281263273381 x[3,17]
   - 124.3769719948273 x[3,18] - 98.04460844611688 x[3,19]
   - 72.38556370505258 x[3,20] - 123.4209990806296 x[3,21]
   - 70.56305847984812 x[3,22] - 70.56305847984812 x[3,23]
   - service_time[3] - start_service[3] + start_service[23] >= 0
 LowerBoundRoutingVehicle[4]: - 108.0989640169925 x[4,1]
   - 121.3939076331102 x[4,2] - 116.4804209463363 x[4,3]
   - 108.0989640169925 x[4,4] - 113.2718818372786 x[4,5]
   - 125.1290830328333 x[4,6] - 109.1207233884747 x[4,7]
   - 114.5344724285567 x[4,8] - 143.3920307224904 x[4,9]
   - 108.1103916296177 x[4,10] - 155.4704214009819 x[4,11]
   - 121.3939076331102 x[4,12] - 116.4804209463363 x[4,13]
   - 108.0989640169925 x[4,14] - 113.2718818372786 x[4,15]
   - 125.1290830328333 x[4,16] - 109.1207233884747 x[4,17]
   - 114.5344724285567 x[4,18] - 143.3920307224904 x[4,19]
   - 108.1103916296177 x[4,20] - 155.4704214009819 x[4,21]
   - 108.0989640169925 x[4,22] - 108.0989640169925 x[4,23]
   - service_time[4] - start_service[4] + start_service[23] >= 0
 LowerBoundRoutingVehicle[5]: - 38.74671877957847 x[5,1]
   - 129.3822676832804 x[5,2] - 102.998928778927 x[5,3]
   - 182.6241270746926 x[5,4] - 38.74671877957847 x[5,5]
   - 88.97026797526898 x[5,6] - 72.4599129933484 x[5,7]
   - 141.0143231257816 x[5,8] - 71.39016206491223 x[5,9]
   - 45.24535287395532 x[5,10] - 142.6373929833227 x[5,11]
   - 129.3822676832804 x[5,12] - 102.998928778927 x[5,13]
   - 182.6241270746926 x[5,14] - 38.74671877957847 x[5,15]
   - 88.97026797526898 x[5,16] - 72.4599129933484 x[5,17]
   - 141.0143231257816 x[5,18] - 71.39016206491223 x[5,19]
   - 45.24535287395532 x[5,20] - 142.6373929833227 x[5,21]
   - 38.74671877957847 x[5,22] - 38.74671877957847 x[5,23]
   - service_time[5] - start_service[5] + start_service[23] >= 0
 LowerBoundRoutingVehicle[6]: - 43.40874860490221 x[6,1]
   - 156.4489851918065 x[6,2] - 132.336320067263 x[6,3]
   - 189.8192984449236 x[6,4] - 84.30823814994523 x[6,5]
   - 43.40874860490221 x[6,6] - 73.03077129339222 x[6,7]
   - 135.9869142921814 x[6,8] - 108.294135071911 x[6,9]
   - 58.88025769136905 x[6,10] - 182.6190398036732 x[6,11]
   - 156.4489851918065 x[6,12] - 132.336320067263 x[6,13]
   - 189.8192984449236 x[6,14] - 84.30823814994523 x[6,15]
   - 43.40874860490221 x[6,16] - 73.03077129339222 x[6,17]
   - 135.9869142921814 x[6,18] - 108.294135071911 x[6,19]
   - 58.88025769136905 x[6,20] - 182.6190398036732 x[6,21]
   - 43.40874860490221 x[6,22] - 43.40874860490221 x[6,23]
   - service_time[6] - start_service[6] + start_service[23] >= 0
 LowerBoundRoutingVehicle[7]: - 46.7287261020131 x[7,1]
   - 130.6105388615393 x[7,2] - 106.0671450105688 x[7,3]
   - 170.4909613034541 x[7,4] - 64.47790567091376 x[7,5]
   - 69.71079379628134 x[7,6] - 46.7287261020131 x[7,7]
   - 121.1958349014102 x[7,8] - 95.66356072509844 x[7,9]
   - 47.90985058150097 x[7,10] - 156.6388058226291 x[7,11]
   - 130.6105388615393 x[7,12] - 106.0671450105688 x[7,13]
   - 170.4909613034541 x[7,14] - 64.47790567091376 x[7,15]
   - 69.71079379628134 x[7,16] - 46.7287261020131 x[7,17]
   - 121.1958349014102 x[7,18] - 95.66356072509844 x[7,19]
   - 47.90985058150097 x[7,20] - 156.6388058226291 x[7,21]
   - 46.7287261020131 x[7,22] - 46.7287261020131 x[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= 0
 LowerBoundRoutingVehicle[8]: - 83.92207034822728 x[8,1]
   - 125.2944954207965 x[8,2] - 111.0179601264482 x[8,3]
   - 138.7113660973219 x[8,4] - 95.83897155713279 x[8,5]
   - 95.47359254885636 x[8,6] - 84.00249065519598 x[8,7]
   - 83.92207034822728 x[8,8] - 128.4114258957015 x[8,9]
   - 84.97086416444269 x[8,10] - 161.5756490088997 x[8,11]
   - 125.2944954207965 x[8,12] - 111.0179601264482 x[8,13]
   - 138.7113660973219 x[8,14] - 95.83897155713279 x[8,15]
   - 95.47359254885636 x[8,16] - 84.00249065519598 x[8,17]
   - 83.92207034822728 x[8,18] - 128.4114258957015 x[8,19]
   - 84.97086416444269 x[8,20] - 161.5756490088997 x[8,21]
   - 83.92207034822728 x[8,22] - 83.92207034822728 x[8,23]
   - service_time[8] - start_service[8] + start_service[23] >= 0
 LowerBoundRoutingVehicle[9]: - 38.22353139071852 x[9,1]
   - 155.1591309072733 x[9,2] - 130.3841355352465 x[9,3]
   - 213.2674633487644 x[9,4] - 71.91334945377218 x[9,5]
   - 113.4793522860947 x[9,6] - 104.168755436393 x[9,7]
   - 174.1099648532102 x[9,8] - 38.22353139071852 x[9,9]
   - 69.44564316335894 x[9,10] - 154.0626127363363 x[9,11]
   - 155.1591309072733 x[9,12] - 130.3841355352465 x[9,13]
   - 213.2674633487644 x[9,14] - 71.91334945377218 x[9,15]
   - 113.4793522860947 x[9,16] - 104.168755436393 x[9,17]
   - 174.1099648532102 x[9,18] - 38.22353139071852 x[9,19]
   - 69.44564316335894 x[9,20] - 154.0626127363363 x[9,21]
   - 38.22353139071852 x[9,22] - 38.22353139071852 x[9,23]
   - service_time[9] - start_service[9] + start_service[23] >= 0
 LowerBoundRoutingVehicle[10]: - 28.53607880866945 x[10,1]
   - 140.6993001311416 x[10,2] - 114.4125433762312 x[10,3]
   - 187.6732768379407 x[10,4] - 55.45599284486434 x[10,5]
   - 73.75292748760182 x[10,6] - 66.10249787484462 x[10,7]
   - 140.3568557040005 x[10,8] - 79.13309574540801 x[10,9]
   - 28.53607880866945 x[10,10] - 158.6309822965734 x[10,11]
   - 140.6993001311416 x[10,12] - 114.4125433762312 x[10,13]
   - 187.6732768379407 x[10,14] - 55.45599284486434 x[10,15]
   - 73.75292748760182 x[10,16] - 66.10249787484462 x[10,17]
   - 140.3568557040005 x[10,18] - 79.13309574540801 x[10,19]
   - 28.53607880866945 x[10,20] - 158.6309822965734 x[10,21]
   - 28.53607880866945 x[10,22] - 28.53607880866945 x[10,23]
   - service_time[10] - start_service[10] + start_service[23] >= 0
 LowerBoundRoutingVehicle[11]: - 89.83670481847882 x[11,1]
   - 114.4911466457747 x[11,2] - 104.1473527419989 x[11,3]
   - 173.7326805994956 x[11,4] - 91.54740694442238 x[11,5]
   - 136.1910835900966 x[11,6] - 113.5308271061634 x[11,7]
   - 155.6610145386482 x[11,8] - 102.449439308576 x[11,9]
   - 97.33035628676403 x[11,10] - 89.83670481847882 x[11,11]
   - 114.4911466457747 x[11,12] - 104.1473527419989 x[11,13]
   - 173.7326805994956 x[11,14] - 91.54740694442238 x[11,15]
   - 136.1910835900966 x[11,16] - 113.5308271061634 x[11,17]
   - 155.6610145386482 x[11,18] - 102.449439308576 x[11,19]
   - 97.33035628676403 x[11,20] - 89.83670481847882 x[11,21]
   - 89.83670481847882 x[11,22] - 89.83670481847882 x[11,23]
   - service_time[11] - start_service[11] + start_service[23] >= 0
 LowerBoundRoutingVehicle[12]: - 83.79752037613687 x[12,1]
   - 83.79752037613687 x[12,2] - 83.79797506785692 x[12,3]
   - 145.6953512739659 x[12,4] - 84.331466086722 x[12,5]
   - 116.0602134205718 x[12,6] - 93.54174458741556 x[12,7]
   - 125.4190453928869 x[12,8] - 109.5851419218549 x[12,9]
   - 85.43785856367417 x[12,10] - 120.5303310881166 x[12,11]
   - 83.79752037613687 x[12,12] - 83.79797506785692 x[12,13]
   - 145.6953512739659 x[12,14] - 84.331466086722 x[12,15]
   - 116.0602134205718 x[12,16] - 93.54174458741556 x[12,17]
   - 125.4190453928869 x[12,18] - 109.5851419218549 x[12,19]
   - 85.43785856367417 x[12,20] - 120.5303310881166 x[12,21]
   - 83.79752037613687 x[12,22] - 83.79752037613687 x[12,23]
   - service_time[12] - start_service[12] + start_service[23] >= 0
 LowerBoundRoutingVehicle[13]: - 70.56305847984812 x[13,1]
   - 97.03243696414569 x[13,2] - 70.56305847984812 x[13,3]
   - 154.0163264834807 x[13,4] - 71.18258907865733 x[13,5]
   - 105.1820101923171 x[13,6] - 82.23281263273381 x[13,7]
   - 124.3769719948273 x[13,8] - 98.04460844611688 x[13,9]
   - 72.38556370505258 x[13,10] - 123.4209990806296 x[13,11]
   - 97.03243696414569 x[13,12] - 70.56305847984812 x[13,13]
   - 154.0163264834807 x[13,14] - 71.18258907865733 x[13,15]
   - 105.1820101923171 x[13,16] - 82.23281263273381 x[13,17]
   - 124.3769719948273 x[13,18] - 98.04460844611688 x[13,19]
   - 72.38556370505258 x[13,20] - 123.4209990806296 x[13,21]
   - 70.56305847984812 x[13,22] - 70.56305847984812 x[13,23]
   - service_time[13] - start_service[13] + start_service[23] >= 0
 LowerBoundRoutingVehicle[14]: - 108.0989640169925 x[14,1]
   - 121.3939076331102 x[14,2] - 116.4804209463363 x[14,3]
   - 108.0989640169925 x[14,4] - 113.2718818372786 x[14,5]
   - 125.1290830328333 x[14,6] - 109.1207233884747 x[14,7]
   - 114.5344724285567 x[14,8] - 143.3920307224904 x[14,9]
   - 108.1103916296177 x[14,10] - 155.4704214009819 x[14,11]
   - 121.3939076331102 x[14,12] - 116.4804209463363 x[14,13]
   - 108.0989640169925 x[14,14] - 113.2718818372786 x[14,15]
   - 125.1290830328333 x[14,16] - 109.1207233884747 x[14,17]
   - 114.5344724285567 x[14,18] - 143.3920307224904 x[14,19]
   - 108.1103916296177 x[14,20] - 155.4704214009819 x[14,21]
   - 108.0989640169925 x[14,22] - 108.0989640169925 x[14,23]
   - service_time[14] - start_service[14] + start_service[23] >= 0
 LowerBoundRoutingVehicle[15]: - 38.74671877957847 x[15,1]
   - 129.3822676832804 x[15,2] - 102.998928778927 x[15,3]
   - 182.6241270746926 x[15,4] - 38.74671877957847 x[15,5]
   - 88.97026797526898 x[15,6] - 72.4599129933484 x[15,7]
   - 141.0143231257816 x[15,8] - 71.39016206491223 x[15,9]
   - 45.24535287395532 x[15,10] - 142.6373929833227 x[15,11]
   - 129.3822676832804 x[15,12] - 102.998928778927 x[15,13]
   - 182.6241270746926 x[15,14] - 38.74671877957847 x[15,15]
   - 88.97026797526898 x[15,16] - 72.4599129933484 x[15,17]
   - 141.0143231257816 x[15,18] - 71.39016206491223 x[15,19]
   - 45.24535287395532 x[15,20] - 142.6373929833227 x[15,21]
   - 38.74671877957847 x[15,22] - 38.74671877957847 x[15,23]
   - service_time[15] - start_service[15] + start_service[23] >= 0
 LowerBoundRoutingVehicle[16]: - 43.40874860490221 x[16,1]
   - 156.4489851918065 x[16,2] - 132.336320067263 x[16,3]
   - 189.8192984449236 x[16,4] - 84.30823814994523 x[16,5]
   - 43.40874860490221 x[16,6] - 73.03077129339222 x[16,7]
   - 135.9869142921814 x[16,8] - 108.294135071911 x[16,9]
   - 58.88025769136905 x[16,10] - 182.6190398036732 x[16,11]
   - 156.4489851918065 x[16,12] - 132.336320067263 x[16,13]
   - 189.8192984449236 x[16,14] - 84.30823814994523 x[16,15]
   - 43.40874860490221 x[16,16] - 73.03077129339222 x[16,17]
   - 135.9869142921814 x[16,18] - 108.294135071911 x[16,19]
   - 58.88025769136905 x[16,20] - 182.6190398036732 x[16,21]
   - 43.40874860490221 x[16,22] - 43.40874860490221 x[16,23]
   - service_time[16] - start_service[16] + start_service[23] >= 0
 LowerBoundRoutingVehicle[17]: - 46.7287261020131 x[17,1]
   - 130.6105388615393 x[17,2] - 106.0671450105688 x[17,3]
   - 170.4909613034541 x[17,4] - 64.47790567091376 x[17,5]
   - 69.71079379628134 x[17,6] - 46.7287261020131 x[17,7]
   - 121.1958349014102 x[17,8] - 95.66356072509844 x[17,9]
   - 47.90985058150097 x[17,10] - 156.6388058226291 x[17,11]
   - 130.6105388615393 x[17,12] - 106.0671450105688 x[17,13]
   - 170.4909613034541 x[17,14] - 64.47790567091376 x[17,15]
   - 69.71079379628134 x[17,16] - 46.7287261020131 x[17,17]
   - 121.1958349014102 x[17,18] - 95.66356072509844 x[17,19]
   - 47.90985058150097 x[17,20] - 156.6388058226291 x[17,21]
   - 46.7287261020131 x[17,22] - 46.7287261020131 x[17,23]
   - service_time[17] - start_service[17] + start_service[23] >= 0
 LowerBoundRoutingVehicle[18]: - 83.92207034822728 x[18,1]
   - 125.2944954207965 x[18,2] - 111.0179601264482 x[18,3]
   - 138.7113660973219 x[18,4] - 95.83897155713279 x[18,5]
   - 95.47359254885636 x[18,6] - 84.00249065519598 x[18,7]
   - 83.92207034822728 x[18,8] - 128.4114258957015 x[18,9]
   - 84.97086416444269 x[18,10] - 161.5756490088997 x[18,11]
   - 125.2944954207965 x[18,12] - 111.0179601264482 x[18,13]
   - 138.7113660973219 x[18,14] - 95.83897155713279 x[18,15]
   - 95.47359254885636 x[18,16] - 84.00249065519598 x[18,17]
   - 83.92207034822728 x[18,18] - 128.4114258957015 x[18,19]
   - 84.97086416444269 x[18,20] - 161.5756490088997 x[18,21]
   - 83.92207034822728 x[18,22] - 83.92207034822728 x[18,23]
   - service_time[18] - start_service[18] + start_service[23] >= 0
 LowerBoundRoutingVehicle[19]: - 38.22353139071852 x[19,1]
   - 155.1591309072733 x[19,2] - 130.3841355352465 x[19,3]
   - 213.2674633487644 x[19,4] - 71.91334945377218 x[19,5]
   - 113.4793522860947 x[19,6] - 104.168755436393 x[19,7]
   - 174.1099648532102 x[19,8] - 38.22353139071852 x[19,9]
   - 69.44564316335894 x[19,10] - 154.0626127363363 x[19,11]
   - 155.1591309072733 x[19,12] - 130.3841355352465 x[19,13]
   - 213.2674633487644 x[19,14] - 71.91334945377218 x[19,15]
   - 113.4793522860947 x[19,16] - 104.168755436393 x[19,17]
   - 174.1099648532102 x[19,18] - 38.22353139071852 x[19,19]
   - 69.44564316335894 x[19,20] - 154.0626127363363 x[19,21]
   - 38.22353139071852 x[19,22] - 38.22353139071852 x[19,23]
   - service_time[19] - start_service[19] + start_service[23] >= 0
 LowerBoundRoutingVehicle[20]: - 28.53607880866945 x[20,1]
   - 140.6993001311416 x[20,2] - 114.4125433762312 x[20,3]
   - 187.6732768379407 x[20,4] - 55.45599284486434 x[20,5]
   - 73.75292748760182 x[20,6] - 66.10249787484462 x[20,7]
   - 140.3568557040005 x[20,8] - 79.13309574540801 x[20,9]
   - 28.53607880866945 x[20,10] - 158.6309822965734 x[20,11]
   - 140.6993001311416 x[20,12] - 114.4125433762312 x[20,13]
   - 187.6732768379407 x[20,14] - 55.45599284486434 x[20,15]
   - 73.75292748760182 x[20,16] - 66.10249787484462 x[20,17]
   - 140.3568557040005 x[20,18] - 79.13309574540801 x[20,19]
   - 28.53607880866945 x[20,20] - 158.6309822965734 x[20,21]
   - 28.53607880866945 x[20,22] - 28.53607880866945 x[20,23]
   - service_time[20] - start_service[20] + start_service[23] >= 0
 LowerBoundRoutingVehicle[21]: - 89.83670481847882 x[21,1]
   - 114.4911466457747 x[21,2] - 104.1473527419989 x[21,3]
   - 173.7326805994956 x[21,4] - 91.54740694442238 x[21,5]
   - 136.1910835900966 x[21,6] - 113.5308271061634 x[21,7]
   - 155.6610145386482 x[21,8] - 102.449439308576 x[21,9]
   - 97.33035628676403 x[21,10] - 89.83670481847882 x[21,11]
   - 114.4911466457747 x[21,12] - 104.1473527419989 x[21,13]
   - 173.7326805994956 x[21,14] - 91.54740694442238 x[21,15]
   - 136.1910835900966 x[21,16] - 113.5308271061634 x[21,17]
   - 155.6610145386482 x[21,18] - 102.449439308576 x[21,19]
   - 97.33035628676403 x[21,20] - 89.83670481847882 x[21,21]
   - 89.83670481847882 x[21,22] - 89.83670481847882 x[21,23]
   - service_time[21] - start_service[21] + start_service[23] >= 0
 LowerBoundRoutingVehicle[22]: - 167.5950407522737 x[22,2]
   - 141.1261169596962 x[22,3] - 216.197928033985 x[22,4]
   - 77.49343755915693 x[22,5] - 86.81749720980443 x[22,6]
   - 93.4574522040262 x[22,7] - 167.8441406964546 x[22,8]
   - 76.44706278143704 x[22,9] - 57.0721576173389 x[22,10]
   - 179.6734096369576 x[22,11] - 167.5950407522737 x[22,12]
   - 141.1261169596962 x[22,13] - 216.197928033985 x[22,14]
   - 77.49343755915693 x[22,15] - 86.81749720980443 x[22,16]
   - 93.4574522040262 x[22,17] - 167.8441406964546 x[22,18]
   - 76.44706278143704 x[22,19] - 57.0721576173389 x[22,20]
   - 179.6734096369576 x[22,21] - service_time[22] - start_service[22]
   + start_service[23] >= 0
 LowerBoundArrivalVehicle[1]: - 167.5950407522737 x[2,1]
   - 141.1261169596962 x[3,1] - 216.197928033985 x[4,1]
   - 77.49343755915693 x[5,1] - 86.81749720980443 x[6,1]
   - 93.4574522040262 x[7,1] - 167.8441406964546 x[8,1]
   - 76.44706278143704 x[9,1] - 57.0721576173389 x[10,1]
   - 179.6734096369576 x[11,1] - 167.5950407522737 x[12,1]
   - 141.1261169596962 x[13,1] - 216.197928033985 x[14,1]
   - 77.49343755915693 x[15,1] - 86.81749720980443 x[16,1]
   - 93.4574522040262 x[17,1] - 167.8441406964546 x[18,1]
   - 76.44706278143704 x[19,1] - 57.0721576173389 x[20,1]
   - 179.6734096369576 x[21,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 83.79752037613687 x[0,2]
   - 83.79752037613687 x[1,2] - 83.79752037613687 x[2,2]
   - 83.79797506785692 x[3,2] - 145.6953512739659 x[4,2]
   - 84.331466086722 x[5,2] - 116.0602134205718 x[6,2]
   - 93.54174458741556 x[7,2] - 125.4190453928869 x[8,2]
   - 109.5851419218549 x[9,2] - 85.43785856367417 x[10,2]
   - 120.5303310881166 x[11,2] - 83.79752037613687 x[12,2]
   - 83.79797506785692 x[13,2] - 145.6953512739659 x[14,2]
   - 84.331466086722 x[15,2] - 116.0602134205718 x[16,2]
   - 93.54174458741556 x[17,2] - 125.4190453928869 x[18,2]
   - 109.5851419218549 x[19,2] - 85.43785856367417 x[20,2]
   - 120.5303310881166 x[21,2] - 83.79752037613687 x[22,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 70.56305847984812 x[0,3]
   - 70.56305847984812 x[1,3] - 97.03243696414569 x[2,3]
   - 70.56305847984812 x[3,3] - 154.0163264834807 x[4,3]
   - 71.18258907865733 x[5,3] - 105.1820101923171 x[6,3]
   - 82.23281263273381 x[7,3] - 124.3769719948273 x[8,3]
   - 98.04460844611688 x[9,3] - 72.38556370505258 x[10,3]
   - 123.4209990806296 x[11,3] - 97.03243696414569 x[12,3]
   - 70.56305847984812 x[13,3] - 154.0163264834807 x[14,3]
   - 71.18258907865733 x[15,3] - 105.1820101923171 x[16,3]
   - 82.23281263273381 x[17,3] - 124.3769719948273 x[18,3]
   - 98.04460844611688 x[19,3] - 72.38556370505258 x[20,3]
   - 123.4209990806296 x[21,3] - 70.56305847984812 x[22,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 108.0989640169925 x[0,4]
   - 108.0989640169925 x[1,4] - 121.3939076331102 x[2,4]
   - 116.4804209463363 x[3,4] - 108.0989640169925 x[4,4]
   - 113.2718818372786 x[5,4] - 125.1290830328333 x[6,4]
   - 109.1207233884747 x[7,4] - 114.5344724285567 x[8,4]
   - 143.3920307224904 x[9,4] - 108.1103916296177 x[10,4]
   - 155.4704214009819 x[11,4] - 121.3939076331102 x[12,4]
   - 116.4804209463363 x[13,4] - 108.0989640169925 x[14,4]
   - 113.2718818372786 x[15,4] - 125.1290830328333 x[16,4]
   - 109.1207233884747 x[17,4] - 114.5344724285567 x[18,4]
   - 143.3920307224904 x[19,4] - 108.1103916296177 x[20,4]
   - 155.4704214009819 x[21,4] - 108.0989640169925 x[22,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 38.74671877957847 x[0,5]
   - 38.74671877957847 x[1,5] - 129.3822676832804 x[2,5]
   - 102.998928778927 x[3,5] - 182.6241270746926 x[4,5]
   - 38.74671877957847 x[5,5] - 88.97026797526898 x[6,5]
   - 72.4599129933484 x[7,5] - 141.0143231257816 x[8,5]
   - 71.39016206491223 x[9,5] - 45.24535287395532 x[10,5]
   - 142.6373929833227 x[11,5] - 129.3822676832804 x[12,5]
   - 102.998928778927 x[13,5] - 182.6241270746926 x[14,5]
   - 38.74671877957847 x[15,5] - 88.97026797526898 x[16,5]
   - 72.4599129933484 x[17,5] - 141.0143231257816 x[18,5]
   - 71.39016206491223 x[19,5] - 45.24535287395532 x[20,5]
   - 142.6373929833227 x[21,5] - 38.74671877957847 x[22,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 43.40874860490221 x[0,6]
   - 43.40874860490221 x[1,6] - 156.4489851918065 x[2,6]
   - 132.336320067263 x[3,6] - 189.8192984449236 x[4,6]
   - 84.30823814994523 x[5,6] - 43.40874860490221 x[6,6]
   - 73.03077129339222 x[7,6] - 135.9869142921814 x[8,6]
   - 108.294135071911 x[9,6] - 58.88025769136905 x[10,6]
   - 182.6190398036732 x[11,6] - 156.4489851918065 x[12,6]
   - 132.336320067263 x[13,6] - 189.8192984449236 x[14,6]
   - 84.30823814994523 x[15,6] - 43.40874860490221 x[16,6]
   - 73.03077129339222 x[17,6] - 135.9869142921814 x[18,6]
   - 108.294135071911 x[19,6] - 58.88025769136905 x[20,6]
   - 182.6190398036732 x[21,6] - 43.40874860490221 x[22,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 46.7287261020131 x[0,7]
   - 46.7287261020131 x[1,7] - 130.6105388615393 x[2,7]
   - 106.0671450105688 x[3,7] - 170.4909613034541 x[4,7]
   - 64.47790567091376 x[5,7] - 69.71079379628134 x[6,7]
   - 46.7287261020131 x[7,7] - 121.1958349014102 x[8,7]
   - 95.66356072509844 x[9,7] - 47.90985058150097 x[10,7]
   - 156.6388058226291 x[11,7] - 130.6105388615393 x[12,7]
   - 106.0671450105688 x[13,7] - 170.4909613034541 x[14,7]
   - 64.47790567091376 x[15,7] - 69.71079379628134 x[16,7]
   - 46.7287261020131 x[17,7] - 121.1958349014102 x[18,7]
   - 95.66356072509844 x[19,7] - 47.90985058150097 x[20,7]
   - 156.6388058226291 x[21,7] - 46.7287261020131 x[22,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 83.92207034822728 x[0,8]
   - 83.92207034822728 x[1,8] - 125.2944954207965 x[2,8]
   - 111.0179601264482 x[3,8] - 138.7113660973219 x[4,8]
   - 95.83897155713279 x[5,8] - 95.47359254885636 x[6,8]
   - 84.00249065519598 x[7,8] - 83.92207034822728 x[8,8]
   - 128.4114258957015 x[9,8] - 84.97086416444269 x[10,8]
   - 161.5756490088997 x[11,8] - 125.2944954207965 x[12,8]
   - 111.0179601264482 x[13,8] - 138.7113660973219 x[14,8]
   - 95.83897155713279 x[15,8] - 95.47359254885636 x[16,8]
   - 84.00249065519598 x[17,8] - 83.92207034822728 x[18,8]
   - 128.4114258957015 x[19,8] - 84.97086416444269 x[20,8]
   - 161.5756490088997 x[21,8] - 83.92207034822728 x[22,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 38.22353139071852 x[0,9]
   - 38.22353139071852 x[1,9] - 155.1591309072733 x[2,9]
   - 130.3841355352465 x[3,9] - 213.2674633487644 x[4,9]
   - 71.91334945377218 x[5,9] - 113.4793522860947 x[6,9]
   - 104.168755436393 x[7,9] - 174.1099648532102 x[8,9]
   - 38.22353139071852 x[9,9] - 69.44564316335894 x[10,9]
   - 154.0626127363363 x[11,9] - 155.1591309072733 x[12,9]
   - 130.3841355352465 x[13,9] - 213.2674633487644 x[14,9]
   - 71.91334945377218 x[15,9] - 113.4793522860947 x[16,9]
   - 104.168755436393 x[17,9] - 174.1099648532102 x[18,9]
   - 38.22353139071852 x[19,9] - 69.44564316335894 x[20,9]
   - 154.0626127363363 x[21,9] - 38.22353139071852 x[22,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 28.53607880866945 x[0,10]
   - 28.53607880866945 x[1,10] - 140.6993001311416 x[2,10]
   - 114.4125433762312 x[3,10] - 187.6732768379407 x[4,10]
   - 55.45599284486434 x[5,10] - 73.75292748760182 x[6,10]
   - 66.10249787484462 x[7,10] - 140.3568557040005 x[8,10]
   - 79.13309574540801 x[9,10] - 28.53607880866945 x[10,10]
   - 158.6309822965734 x[11,10] - 140.6993001311416 x[12,10]
   - 114.4125433762312 x[13,10] - 187.6732768379407 x[14,10]
   - 55.45599284486434 x[15,10] - 73.75292748760182 x[16,10]
   - 66.10249787484462 x[17,10] - 140.3568557040005 x[18,10]
   - 79.13309574540801 x[19,10] - 28.53607880866945 x[20,10]
   - 158.6309822965734 x[21,10] - 28.53607880866945 x[22,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 89.83670481847882 x[0,11]
   - 89.83670481847882 x[1,11] - 114.4911466457747 x[2,11]
   - 104.1473527419989 x[3,11] - 173.7326805994956 x[4,11]
   - 91.54740694442238 x[5,11] - 136.1910835900966 x[6,11]
   - 113.5308271061634 x[7,11] - 155.6610145386482 x[8,11]
   - 102.449439308576 x[9,11] - 97.33035628676403 x[10,11]
   - 89.83670481847882 x[11,11] - 114.4911466457747 x[12,11]
   - 104.1473527419989 x[13,11] - 173.7326805994956 x[14,11]
   - 91.54740694442238 x[15,11] - 136.1910835900966 x[16,11]
   - 113.5308271061634 x[17,11] - 155.6610145386482 x[18,11]
   - 102.449439308576 x[19,11] - 97.33035628676403 x[20,11]
   - 89.83670481847882 x[21,11] - 89.83670481847882 x[22,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 83.79752037613687 x[0,12]
   - 83.79752037613687 x[1,12] - 83.79752037613687 x[2,12]
   - 83.79797506785692 x[3,12] - 145.6953512739659 x[4,12]
   - 84.331466086722 x[5,12] - 116.0602134205718 x[6,12]
   - 93.54174458741556 x[7,12] - 125.4190453928869 x[8,12]
   - 109.5851419218549 x[9,12] - 85.43785856367417 x[10,12]
   - 120.5303310881166 x[11,12] - 83.79752037613687 x[12,12]
   - 83.79797506785692 x[13,12] - 145.6953512739659 x[14,12]
   - 84.331466086722 x[15,12] - 116.0602134205718 x[16,12]
   - 93.54174458741556 x[17,12] - 125.4190453928869 x[18,12]
   - 109.5851419218549 x[19,12] - 85.43785856367417 x[20,12]
   - 120.5303310881166 x[21,12] - 83.79752037613687 x[22,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 70.56305847984812 x[0,13]
   - 70.56305847984812 x[1,13] - 97.03243696414569 x[2,13]
   - 70.56305847984812 x[3,13] - 154.0163264834807 x[4,13]
   - 71.18258907865733 x[5,13] - 105.1820101923171 x[6,13]
   - 82.23281263273381 x[7,13] - 124.3769719948273 x[8,13]
   - 98.04460844611688 x[9,13] - 72.38556370505258 x[10,13]
   - 123.4209990806296 x[11,13] - 97.03243696414569 x[12,13]
   - 70.56305847984812 x[13,13] - 154.0163264834807 x[14,13]
   - 71.18258907865733 x[15,13] - 105.1820101923171 x[16,13]
   - 82.23281263273381 x[17,13] - 124.3769719948273 x[18,13]
   - 98.04460844611688 x[19,13] - 72.38556370505258 x[20,13]
   - 123.4209990806296 x[21,13] - 70.56305847984812 x[22,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 108.0989640169925 x[0,14]
   - 108.0989640169925 x[1,14] - 121.3939076331102 x[2,14]
   - 116.4804209463363 x[3,14] - 108.0989640169925 x[4,14]
   - 113.2718818372786 x[5,14] - 125.1290830328333 x[6,14]
   - 109.1207233884747 x[7,14] - 114.5344724285567 x[8,14]
   - 143.3920307224904 x[9,14] - 108.1103916296177 x[10,14]
   - 155.4704214009819 x[11,14] - 121.3939076331102 x[12,14]
   - 116.4804209463363 x[13,14] - 108.0989640169925 x[14,14]
   - 113.2718818372786 x[15,14] - 125.1290830328333 x[16,14]
   - 109.1207233884747 x[17,14] - 114.5344724285567 x[18,14]
   - 143.3920307224904 x[19,14] - 108.1103916296177 x[20,14]
   - 155.4704214009819 x[21,14] - 108.0989640169925 x[22,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 38.74671877957847 x[0,15]
   - 38.74671877957847 x[1,15] - 129.3822676832804 x[2,15]
   - 102.998928778927 x[3,15] - 182.6241270746926 x[4,15]
   - 38.74671877957847 x[5,15] - 88.97026797526898 x[6,15]
   - 72.4599129933484 x[7,15] - 141.0143231257816 x[8,15]
   - 71.39016206491223 x[9,15] - 45.24535287395532 x[10,15]
   - 142.6373929833227 x[11,15] - 129.3822676832804 x[12,15]
   - 102.998928778927 x[13,15] - 182.6241270746926 x[14,15]
   - 38.74671877957847 x[15,15] - 88.97026797526898 x[16,15]
   - 72.4599129933484 x[17,15] - 141.0143231257816 x[18,15]
   - 71.39016206491223 x[19,15] - 45.24535287395532 x[20,15]
   - 142.6373929833227 x[21,15] - 38.74671877957847 x[22,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 43.40874860490221 x[0,16]
   - 43.40874860490221 x[1,16] - 156.4489851918065 x[2,16]
   - 132.336320067263 x[3,16] - 189.8192984449236 x[4,16]
   - 84.30823814994523 x[5,16] - 43.40874860490221 x[6,16]
   - 73.03077129339222 x[7,16] - 135.9869142921814 x[8,16]
   - 108.294135071911 x[9,16] - 58.88025769136905 x[10,16]
   - 182.6190398036732 x[11,16] - 156.4489851918065 x[12,16]
   - 132.336320067263 x[13,16] - 189.8192984449236 x[14,16]
   - 84.30823814994523 x[15,16] - 43.40874860490221 x[16,16]
   - 73.03077129339222 x[17,16] - 135.9869142921814 x[18,16]
   - 108.294135071911 x[19,16] - 58.88025769136905 x[20,16]
   - 182.6190398036732 x[21,16] - 43.40874860490221 x[22,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 46.7287261020131 x[0,17]
   - 46.7287261020131 x[1,17] - 130.6105388615393 x[2,17]
   - 106.0671450105688 x[3,17] - 170.4909613034541 x[4,17]
   - 64.47790567091376 x[5,17] - 69.71079379628134 x[6,17]
   - 46.7287261020131 x[7,17] - 121.1958349014102 x[8,17]
   - 95.66356072509844 x[9,17] - 47.90985058150097 x[10,17]
   - 156.6388058226291 x[11,17] - 130.6105388615393 x[12,17]
   - 106.0671450105688 x[13,17] - 170.4909613034541 x[14,17]
   - 64.47790567091376 x[15,17] - 69.71079379628134 x[16,17]
   - 46.7287261020131 x[17,17] - 121.1958349014102 x[18,17]
   - 95.66356072509844 x[19,17] - 47.90985058150097 x[20,17]
   - 156.6388058226291 x[21,17] - 46.7287261020131 x[22,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 83.92207034822728 x[0,18]
   - 83.92207034822728 x[1,18] - 125.2944954207965 x[2,18]
   - 111.0179601264482 x[3,18] - 138.7113660973219 x[4,18]
   - 95.83897155713279 x[5,18] - 95.47359254885636 x[6,18]
   - 84.00249065519598 x[7,18] - 83.92207034822728 x[8,18]
   - 128.4114258957015 x[9,18] - 84.97086416444269 x[10,18]
   - 161.5756490088997 x[11,18] - 125.2944954207965 x[12,18]
   - 111.0179601264482 x[13,18] - 138.7113660973219 x[14,18]
   - 95.83897155713279 x[15,18] - 95.47359254885636 x[16,18]
   - 84.00249065519598 x[17,18] - 83.92207034822728 x[18,18]
   - 128.4114258957015 x[19,18] - 84.97086416444269 x[20,18]
   - 161.5756490088997 x[21,18] - 83.92207034822728 x[22,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 38.22353139071852 x[0,19]
   - 38.22353139071852 x[1,19] - 155.1591309072733 x[2,19]
   - 130.3841355352465 x[3,19] - 213.2674633487644 x[4,19]
   - 71.91334945377218 x[5,19] - 113.4793522860947 x[6,19]
   - 104.168755436393 x[7,19] - 174.1099648532102 x[8,19]
   - 38.22353139071852 x[9,19] - 69.44564316335894 x[10,19]
   - 154.0626127363363 x[11,19] - 155.1591309072733 x[12,19]
   - 130.3841355352465 x[13,19] - 213.2674633487644 x[14,19]
   - 71.91334945377218 x[15,19] - 113.4793522860947 x[16,19]
   - 104.168755436393 x[17,19] - 174.1099648532102 x[18,19]
   - 38.22353139071852 x[19,19] - 69.44564316335894 x[20,19]
   - 154.0626127363363 x[21,19] - 38.22353139071852 x[22,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 28.53607880866945 x[0,20]
   - 28.53607880866945 x[1,20] - 140.6993001311416 x[2,20]
   - 114.4125433762312 x[3,20] - 187.6732768379407 x[4,20]
   - 55.45599284486434 x[5,20] - 73.75292748760182 x[6,20]
   - 66.10249787484462 x[7,20] - 140.3568557040005 x[8,20]
   - 79.13309574540801 x[9,20] - 28.53607880866945 x[10,20]
   - 158.6309822965734 x[11,20] - 140.6993001311416 x[12,20]
   - 114.4125433762312 x[13,20] - 187.6732768379407 x[14,20]
   - 55.45599284486434 x[15,20] - 73.75292748760182 x[16,20]
   - 66.10249787484462 x[17,20] - 140.3568557040005 x[18,20]
   - 79.13309574540801 x[19,20] - 28.53607880866945 x[20,20]
   - 158.6309822965734 x[21,20] - 28.53607880866945 x[22,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 89.83670481847882 x[0,21]
   - 89.83670481847882 x[1,21] - 114.4911466457747 x[2,21]
   - 104.1473527419989 x[3,21] - 173.7326805994956 x[4,21]
   - 91.54740694442238 x[5,21] - 136.1910835900966 x[6,21]
   - 113.5308271061634 x[7,21] - 155.6610145386482 x[8,21]
   - 102.449439308576 x[9,21] - 97.33035628676403 x[10,21]
   - 89.83670481847882 x[11,21] - 114.4911466457747 x[12,21]
   - 104.1473527419989 x[13,21] - 173.7326805994956 x[14,21]
   - 91.54740694442238 x[15,21] - 136.1910835900966 x[16,21]
   - 113.5308271061634 x[17,21] - 155.6610145386482 x[18,21]
   - 102.449439308576 x[19,21] - 97.33035628676403 x[20,21]
   - 89.83670481847882 x[21,21] - 89.83670481847882 x[22,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 167.5950407522737 x[2,22]
   - 141.1261169596962 x[3,22] - 216.197928033985 x[4,22]
   - 77.49343755915693 x[5,22] - 86.81749720980443 x[6,22]
   - 93.4574522040262 x[7,22] - 167.8441406964546 x[8,22]
   - 76.44706278143704 x[9,22] - 57.0721576173389 x[10,22]
   - 179.6734096369576 x[11,22] - 167.5950407522737 x[12,22]
   - 141.1261169596962 x[13,22] - 216.197928033985 x[14,22]
   - 77.49343755915693 x[15,22] - 86.81749720980443 x[16,22]
   - 93.4574522040262 x[17,22] - 167.8441406964546 x[18,22]
   - 76.44706278143704 x[19,22] - 57.0721576173389 x[20,22]
   - 179.6734096369576 x[21,22] - start_service[0] + start_service[22]
   >= 0
 LowerBoundArrivalVehicle[23]: - 167.5950407522737 x[2,23]
   - 141.1261169596962 x[3,23] - 216.197928033985 x[4,23]
   - 77.49343755915693 x[5,23] - 86.81749720980443 x[6,23]
   - 93.4574522040262 x[7,23] - 167.8441406964546 x[8,23]
   - 76.44706278143704 x[9,23] - 57.0721576173389 x[10,23]
   - 179.6734096369576 x[11,23] - 167.5950407522737 x[12,23]
   - 141.1261169596962 x[13,23] - 216.197928033985 x[14,23]
   - 77.49343755915693 x[15,23] - 86.81749720980443 x[16,23]
   - 93.4574522040262 x[17,23] - 167.8441406964546 x[18,23]
   - 76.44706278143704 x[19,23] - 57.0721576173389 x[20,23]
   - 179.6734096369576 x[21,23] - start_service[0] + start_service[23]
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
