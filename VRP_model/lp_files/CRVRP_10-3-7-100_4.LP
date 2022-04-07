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
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - service_time[0]
   - start_service[0] + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 1.0000401805928103e+06 x[0,2]
   - service_time[0] - start_service[0] + start_service[2] >= -999999
 NodeArrivalVehicles[0,3]: - 1.0000857578554453e+06 x[0,3]
   - service_time[0] - start_service[0] + start_service[3] >= -999999
 NodeArrivalVehicles[0,4]: - 1.0000147366179901e+06 x[0,4]
   - service_time[0] - start_service[0] + start_service[4] >= -999999
 NodeArrivalVehicles[0,5]: - 1.0000457716821638e+06 x[0,5]
   - service_time[0] - start_service[0] + start_service[5] >= -999999
 NodeArrivalVehicles[0,6]: - 1.0000930531941031e+06 x[0,6]
   - service_time[0] - start_service[0] + start_service[6] >= -999999
 NodeArrivalVehicles[0,7]: - 1.0001128471096832e+06 x[0,7]
   - service_time[0] - start_service[0] + start_service[7] >= -999999
 NodeArrivalVehicles[0,8]: - 1.0000733867898309e+06 x[0,8]
   - service_time[0] - start_service[0] + start_service[8] >= -999999
 NodeArrivalVehicles[0,9]: - 1.0001135742778145e+06 x[0,9]
   - service_time[0] - start_service[0] + start_service[9] >= -999999
 NodeArrivalVehicles[0,10]: - 1.0000665133082913e+06 x[0,10]
   - service_time[0] - start_service[0] + start_service[10] >= -999999
 NodeArrivalVehicles[0,11]: - 1.0000632444353482e+06 x[0,11]
   - service_time[0] - start_service[0] + start_service[11] >= -999999
 NodeArrivalVehicles[0,12]: - 1.0000401805928103e+06 x[0,12]
   - service_time[0] - start_service[0] + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 1.0000857578554453e+06 x[0,13]
   - service_time[0] - start_service[0] + start_service[13] >= -999999
 NodeArrivalVehicles[0,14]: - 1.0000147366179901e+06 x[0,14]
   - service_time[0] - start_service[0] + start_service[14] >= -999999
 NodeArrivalVehicles[0,15]: - 1.0000457716821638e+06 x[0,15]
   - service_time[0] - start_service[0] + start_service[15] >= -999999
 NodeArrivalVehicles[0,16]: - 1.0000930531941031e+06 x[0,16]
   - service_time[0] - start_service[0] + start_service[16] >= -999999
 NodeArrivalVehicles[0,17]: - 1.0001128471096832e+06 x[0,17]
   - service_time[0] - start_service[0] + start_service[17] >= -999999
 NodeArrivalVehicles[0,18]: - 1.0000733867898309e+06 x[0,18]
   - service_time[0] - start_service[0] + start_service[18] >= -999999
 NodeArrivalVehicles[0,19]: - 1.0001135742778145e+06 x[0,19]
   - service_time[0] - start_service[0] + start_service[19] >= -999999
 NodeArrivalVehicles[0,20]: - 1.0000665133082913e+06 x[0,20]
   - service_time[0] - start_service[0] + start_service[20] >= -999999
 NodeArrivalVehicles[0,21]: - 1.0000632444353482e+06 x[0,21]
   - service_time[0] - start_service[0] + start_service[21] >= -999999
 NodeArrivalVehicles[0,22]: - 999999 x[0,22] - service_time[0]
   - start_service[0] + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999 x[0,23] - service_time[0]
   - start_service[0] + start_service[23] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] - service_time[1] >= -999999
 NodeArrivalVehicles[1,2]: - 1.0000401805928103e+06 x[1,2]
   - service_time[1] - start_service[1] + start_service[2] >= -999999
 NodeArrivalVehicles[1,3]: - 1.0000857578554453e+06 x[1,3]
   - service_time[1] - start_service[1] + start_service[3] >= -999999
 NodeArrivalVehicles[1,4]: - 1.0000147366179901e+06 x[1,4]
   - service_time[1] - start_service[1] + start_service[4] >= -999999
 NodeArrivalVehicles[1,5]: - 1.0000457716821638e+06 x[1,5]
   - service_time[1] - start_service[1] + start_service[5] >= -999999
 NodeArrivalVehicles[1,6]: - 1.0000930531941031e+06 x[1,6]
   - service_time[1] - start_service[1] + start_service[6] >= -999999
 NodeArrivalVehicles[1,7]: - 1.0001128471096832e+06 x[1,7]
   - service_time[1] - start_service[1] + start_service[7] >= -999999
 NodeArrivalVehicles[1,8]: - 1.0000733867898309e+06 x[1,8]
   - service_time[1] - start_service[1] + start_service[8] >= -999999
 NodeArrivalVehicles[1,9]: - 1.0001135742778145e+06 x[1,9]
   - service_time[1] - start_service[1] + start_service[9] >= -999999
 NodeArrivalVehicles[1,10]: - 1.0000665133082913e+06 x[1,10]
   - service_time[1] - start_service[1] + start_service[10] >= -999999
 NodeArrivalVehicles[1,11]: - 1.0000632444353482e+06 x[1,11]
   - service_time[1] - start_service[1] + start_service[11] >= -999999
 NodeArrivalVehicles[1,12]: - 1.0000401805928103e+06 x[1,12]
   - service_time[1] - start_service[1] + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 1.0000857578554453e+06 x[1,13]
   - service_time[1] - start_service[1] + start_service[13] >= -999999
 NodeArrivalVehicles[1,14]: - 1.0000147366179901e+06 x[1,14]
   - service_time[1] - start_service[1] + start_service[14] >= -999999
 NodeArrivalVehicles[1,15]: - 1.0000457716821638e+06 x[1,15]
   - service_time[1] - start_service[1] + start_service[15] >= -999999
 NodeArrivalVehicles[1,16]: - 1.0000930531941031e+06 x[1,16]
   - service_time[1] - start_service[1] + start_service[16] >= -999999
 NodeArrivalVehicles[1,17]: - 1.0001128471096832e+06 x[1,17]
   - service_time[1] - start_service[1] + start_service[17] >= -999999
 NodeArrivalVehicles[1,18]: - 1.0000733867898309e+06 x[1,18]
   - service_time[1] - start_service[1] + start_service[18] >= -999999
 NodeArrivalVehicles[1,19]: - 1.0001135742778145e+06 x[1,19]
   - service_time[1] - start_service[1] + start_service[19] >= -999999
 NodeArrivalVehicles[1,20]: - 1.0000665133082913e+06 x[1,20]
   - service_time[1] - start_service[1] + start_service[20] >= -999999
 NodeArrivalVehicles[1,21]: - 1.0000632444353482e+06 x[1,21]
   - service_time[1] - start_service[1] + start_service[21] >= -999999
 NodeArrivalVehicles[1,22]: - 999999 x[1,22] - service_time[1]
   - start_service[1] + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999 x[1,23] - service_time[1]
   - start_service[1] + start_service[23] >= -999999
 NodeArrivalVehicles[2,1]: - 1.0000401805928103e+06 x[2,1]
   - service_time[2] + start_service[1] - start_service[2] >= -999999
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] - service_time[2] >= -999999
 NodeArrivalVehicles[2,3]: - 1.0000500807426213e+06 x[2,3]
   - service_time[2] - start_service[2] + start_service[3] >= -999999
 NodeArrivalVehicles[2,4]: - 1.000030792514505e+06 x[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999999
 NodeArrivalVehicles[2,5]: - 1.0000071143237508e+06 x[2,5]
   - service_time[2] - start_service[2] + start_service[5] >= -999999
 NodeArrivalVehicles[2,6]: - 1.0000533055907747e+06 x[2,6]
   - service_time[2] - start_service[2] + start_service[6] >= -999999
 NodeArrivalVehicles[2,7]: - 1.0000778020430846e+06 x[2,7]
   - service_time[2] - start_service[2] + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 1.0000332531050219e+06 x[2,8]
   - service_time[2] - start_service[2] + start_service[8] >= -999999
 NodeArrivalVehicles[2,9]: - 1.0000784142247903e+06 x[2,9]
   - service_time[2] - start_service[2] + start_service[9] >= -999999
 NodeArrivalVehicles[2,10]: - 1.0000253782266368e+06 x[2,10]
   - service_time[2] - start_service[2] + start_service[10] >= -999999
 NodeArrivalVehicles[2,11]: - 1.0000265613006055e+06 x[2,11]
   - service_time[2] - start_service[2] + start_service[11] >= -999999
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 1.0000500807426213e+06 x[2,13]
   - service_time[2] - start_service[2] + start_service[13] >= -999999
 NodeArrivalVehicles[2,14]: - 1.000030792514505e+06 x[2,14]
   - service_time[2] - start_service[2] + start_service[14] >= -999999
 NodeArrivalVehicles[2,15]: - 1.0000071143237508e+06 x[2,15]
   - service_time[2] - start_service[2] + start_service[15] >= -999999
 NodeArrivalVehicles[2,16]: - 1.0000533055907747e+06 x[2,16]
   - service_time[2] - start_service[2] + start_service[16] >= -999999
 NodeArrivalVehicles[2,17]: - 1.0000778020430846e+06 x[2,17]
   - service_time[2] - start_service[2] + start_service[17] >= -999999
 NodeArrivalVehicles[2,18]: - 1.0000332531050219e+06 x[2,18]
   - service_time[2] - start_service[2] + start_service[18] >= -999999
 NodeArrivalVehicles[2,19]: - 1.0000784142247903e+06 x[2,19]
   - service_time[2] - start_service[2] + start_service[19] >= -999999
 NodeArrivalVehicles[2,20]: - 1.0000253782266368e+06 x[2,20]
   - service_time[2] - start_service[2] + start_service[20] >= -999999
 NodeArrivalVehicles[2,21]: - 1.0000265613006055e+06 x[2,21]
   - service_time[2] - start_service[2] + start_service[21] >= -999999
 NodeArrivalVehicles[2,22]: - 999999 x[2,22] - service_time[2]
   - start_service[2] + start_service[22] >= -999999
 NodeArrivalVehicles[2,23]: - 999999 x[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000857578554453e+06 x[3,1]
   - service_time[3] + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.0000500807426213e+06 x[3,2]
   - service_time[3] + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] - service_time[3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0000713740287239e+06 x[3,4]
   - service_time[3] - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.000042124286405e+06 x[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.0000518389269916e+06 x[3,6]
   - service_time[3] - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.0000267523298946e+06 x[3,7]
   - service_time[3] - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 1.000021622481183e+06 x[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0000273896253681e+06 x[3,9]
   - service_time[3] - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.0000361135195905e+06 x[3,10]
   - service_time[3] - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.0000225783656818e+06 x[3,11]
   - service_time[3] - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 1.0000500807426213e+06 x[3,12]
   - service_time[3] - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 1.0000713740287239e+06 x[3,14]
   - service_time[3] - start_service[3] + start_service[14] >= -999999
 NodeArrivalVehicles[3,15]: - 1.000042124286405e+06 x[3,15]
   - service_time[3] - start_service[3] + start_service[15] >= -999999
 NodeArrivalVehicles[3,16]: - 1.0000518389269916e+06 x[3,16]
   - service_time[3] - start_service[3] + start_service[16] >= -999999
 NodeArrivalVehicles[3,17]: - 1.0000267523298946e+06 x[3,17]
   - service_time[3] - start_service[3] + start_service[17] >= -999999
 NodeArrivalVehicles[3,18]: - 1.000021622481183e+06 x[3,18]
   - service_time[3] - start_service[3] + start_service[18] >= -999999
 NodeArrivalVehicles[3,19]: - 1.0000273896253681e+06 x[3,19]
   - service_time[3] - start_service[3] + start_service[19] >= -999999
 NodeArrivalVehicles[3,20]: - 1.0000361135195905e+06 x[3,20]
   - service_time[3] - start_service[3] + start_service[20] >= -999999
 NodeArrivalVehicles[3,21]: - 1.0000225783656818e+06 x[3,21]
   - service_time[3] - start_service[3] + start_service[21] >= -999999
 NodeArrivalVehicles[3,22]: - 999999 x[3,22] - service_time[3]
   - start_service[3] + start_service[22] >= -999999
 NodeArrivalVehicles[3,23]: - 999999 x[3,23] - service_time[3]
   - start_service[3] + start_service[23] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000147366179901e+06 x[4,1]
   - service_time[4] + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.000030792514505e+06 x[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0000713740287239e+06 x[4,3]
   - service_time[4] + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] - service_time[4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.0000343374844047e+06 x[4,5]
   - service_time[4] - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000850981022327e+06 x[4,6]
   - service_time[4] - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000980247615908e+06 x[4,7]
   - service_time[4] - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0000612054747479e+06 x[4,8]
   - service_time[4] - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 1.0000987723651387e+06 x[4,9]
   - service_time[4] - start_service[4] + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.0000566241376635e+06 x[4,10]
   - service_time[4] - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000497453427232e+06 x[4,11]
   - service_time[4] - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 1.000030792514505e+06 x[4,12]
   - service_time[4] - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 1.0000713740287239e+06 x[4,13]
   - service_time[4] - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[4,14]: - 999999 x[4,14] - service_time[4]
   - start_service[4] + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 1.0000343374844047e+06 x[4,15]
   - service_time[4] - start_service[4] + start_service[15] >= -999999
 NodeArrivalVehicles[4,16]: - 1.0000850981022327e+06 x[4,16]
   - service_time[4] - start_service[4] + start_service[16] >= -999999
 NodeArrivalVehicles[4,17]: - 1.0000980247615908e+06 x[4,17]
   - service_time[4] - start_service[4] + start_service[17] >= -999999
 NodeArrivalVehicles[4,18]: - 1.0000612054747479e+06 x[4,18]
   - service_time[4] - start_service[4] + start_service[18] >= -999999
 NodeArrivalVehicles[4,19]: - 1.0000987723651387e+06 x[4,19]
   - service_time[4] - start_service[4] + start_service[19] >= -999999
 NodeArrivalVehicles[4,20]: - 1.0000566241376635e+06 x[4,20]
   - service_time[4] - start_service[4] + start_service[20] >= -999999
 NodeArrivalVehicles[4,21]: - 1.0000497453427232e+06 x[4,21]
   - service_time[4] - start_service[4] + start_service[21] >= -999999
 NodeArrivalVehicles[4,22]: - 999999 x[4,22] - service_time[4]
   - start_service[4] + start_service[22] >= -999999
 NodeArrivalVehicles[4,23]: - 999999 x[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000457716821638e+06 x[5,1]
   - service_time[5] + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000071143237508e+06 x[5,2]
   - service_time[5] + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.000042124286405e+06 x[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.0000343374844047e+06 x[5,4]
   - service_time[5] + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] - service_time[5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000511580216251e+06 x[5,6]
   - service_time[5] - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000698714195783e+06 x[5,7]
   - service_time[5] - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.0000266352600046e+06 x[5,8]
   - service_time[5] - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.0000704969536402e+06 x[5,9]
   - service_time[5] - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 1.0000216805964843e+06 x[5,10]
   - service_time[5] - start_service[5] + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000185543180438e+06 x[5,11]
   - service_time[5] - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 1.0000071143237508e+06 x[5,12]
   - service_time[5] - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 1.000042124286405e+06 x[5,13]
   - service_time[5] - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[5,14]: - 1.0000343374844047e+06 x[5,14]
   - service_time[5] - start_service[5] + start_service[14] >= -999999
 NodeArrivalVehicles[5,15]: - 999999 x[5,15] - service_time[5]
   - start_service[5] + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 1.0000511580216251e+06 x[5,16]
   - service_time[5] - start_service[5] + start_service[16] >= -999999
 NodeArrivalVehicles[5,17]: - 1.0000698714195783e+06 x[5,17]
   - service_time[5] - start_service[5] + start_service[17] >= -999999
 NodeArrivalVehicles[5,18]: - 1.0000266352600046e+06 x[5,18]
   - service_time[5] - start_service[5] + start_service[18] >= -999999
 NodeArrivalVehicles[5,19]: - 1.0000704969536402e+06 x[5,19]
   - service_time[5] - start_service[5] + start_service[19] >= -999999
 NodeArrivalVehicles[5,20]: - 1.0000216805964843e+06 x[5,20]
   - service_time[5] - start_service[5] + start_service[20] >= -999999
 NodeArrivalVehicles[5,21]: - 1.0000185543180438e+06 x[5,21]
   - service_time[5] - start_service[5] + start_service[21] >= -999999
 NodeArrivalVehicles[5,22]: - 999999 x[5,22] - service_time[5]
   - start_service[5] + start_service[22] >= -999999
 NodeArrivalVehicles[5,23]: - 999999 x[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= -999999
 NodeArrivalVehicles[6,1]: - 1.0000930531941031e+06 x[6,1]
   - service_time[6] + start_service[1] - start_service[6] >= -999999
 NodeArrivalVehicles[6,2]: - 1.0000533055907747e+06 x[6,2]
   - service_time[6] + start_service[2] - start_service[6] >= -999999
 NodeArrivalVehicles[6,3]: - 1.0000518389269916e+06 x[6,3]
   - service_time[6] + start_service[3] - start_service[6] >= -999999
 NodeArrivalVehicles[6,4]: - 1.0000850981022327e+06 x[6,4]
   - service_time[6] + start_service[4] - start_service[6] >= -999999
 NodeArrivalVehicles[6,5]: - 1.0000511580216251e+06 x[6,5]
   - service_time[6] + start_service[5] - start_service[6] >= -999999
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] - service_time[6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000678379998124e+06 x[6,7]
   - service_time[6] - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.0000342428720902e+06 x[6,8]
   - service_time[6] - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000678991589429e+06 x[6,9]
   - service_time[6] - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000287929886794e+06 x[6,10]
   - service_time[6] - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 1.0000472983184841e+06 x[6,11]
   - service_time[6] - start_service[6] + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 1.0000533055907747e+06 x[6,12]
   - service_time[6] - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 1.0000518389269916e+06 x[6,13]
   - service_time[6] - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[6,14]: - 1.0000850981022327e+06 x[6,14]
   - service_time[6] - start_service[6] + start_service[14] >= -999999
 NodeArrivalVehicles[6,15]: - 1.0000511580216251e+06 x[6,15]
   - service_time[6] - start_service[6] + start_service[15] >= -999999
 NodeArrivalVehicles[6,16]: - 999999 x[6,16] - service_time[6]
   - start_service[6] + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 1.0000678379998124e+06 x[6,17]
   - service_time[6] - start_service[6] + start_service[17] >= -999999
 NodeArrivalVehicles[6,18]: - 1.0000342428720902e+06 x[6,18]
   - service_time[6] - start_service[6] + start_service[18] >= -999999
 NodeArrivalVehicles[6,19]: - 1.0000678991589429e+06 x[6,19]
   - service_time[6] - start_service[6] + start_service[19] >= -999999
 NodeArrivalVehicles[6,20]: - 1.0000287929886794e+06 x[6,20]
   - service_time[6] - start_service[6] + start_service[20] >= -999999
 NodeArrivalVehicles[6,21]: - 1.0000472983184841e+06 x[6,21]
   - service_time[6] - start_service[6] + start_service[21] >= -999999
 NodeArrivalVehicles[6,22]: - 999999 x[6,22] - service_time[6]
   - start_service[6] + start_service[22] >= -999999
 NodeArrivalVehicles[6,23]: - 999999 x[6,23] - service_time[6]
   - start_service[6] + start_service[23] >= -999999
 NodeArrivalVehicles[7,1]: - 1.0001128471096832e+06 x[7,1]
   - service_time[7] + start_service[1] - start_service[7] >= -999999
 NodeArrivalVehicles[7,2]: - 1.0000778020430846e+06 x[7,2]
   - service_time[7] + start_service[2] - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 1.0000267523298946e+06 x[7,3]
   - service_time[7] + start_service[3] - start_service[7] >= -999999
 NodeArrivalVehicles[7,4]: - 1.0000980247615908e+06 x[7,4]
   - service_time[7] + start_service[4] - start_service[7] >= -999999
 NodeArrivalVehicles[7,5]: - 1.0000698714195783e+06 x[7,5]
   - service_time[7] + start_service[5] - start_service[7] >= -999999
 NodeArrivalVehicles[7,6]: - 1.0000678379998124e+06 x[7,6]
   - service_time[7] + start_service[6] - start_service[7] >= -999999
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] - service_time[7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.0000469444748068e+06 x[7,8]
   - service_time[7] - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 999999.8437675878 x[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.00006122839988e+06 x[7,10]
   - service_time[7] - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000503303973848e+06 x[7,11]
   - service_time[7] - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 1.0000778020430846e+06 x[7,12]
   - service_time[7] - start_service[7] + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 1.0000267523298946e+06 x[7,13]
   - service_time[7] - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[7,14]: - 1.0000980247615908e+06 x[7,14]
   - service_time[7] - start_service[7] + start_service[14] >= -999999
 NodeArrivalVehicles[7,15]: - 1.0000698714195783e+06 x[7,15]
   - service_time[7] - start_service[7] + start_service[15] >= -999999
 NodeArrivalVehicles[7,16]: - 1.0000678379998124e+06 x[7,16]
   - service_time[7] - start_service[7] + start_service[16] >= -999999
 NodeArrivalVehicles[7,17]: - 999999 x[7,17] - service_time[7]
   - start_service[7] + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 1.0000469444748068e+06 x[7,18]
   - service_time[7] - start_service[7] + start_service[18] >= -999999
 NodeArrivalVehicles[7,19]: - 999999.8437675878 x[7,19] - service_time[7]
   - start_service[7] + start_service[19] >= -999999
 NodeArrivalVehicles[7,20]: - 1.00006122839988e+06 x[7,20]
   - service_time[7] - start_service[7] + start_service[20] >= -999999
 NodeArrivalVehicles[7,21]: - 1.0000503303973848e+06 x[7,21]
   - service_time[7] - start_service[7] + start_service[21] >= -999999
 NodeArrivalVehicles[7,22]: - 999999 x[7,22] - service_time[7]
   - start_service[7] + start_service[22] >= -999999
 NodeArrivalVehicles[7,23]: - 999999 x[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000733867898309e+06 x[8,1]
   - service_time[8] + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.0000332531050219e+06 x[8,2]
   - service_time[8] + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 1.000021622481183e+06 x[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0000612054747479e+06 x[8,4]
   - service_time[8] + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.0000266352600046e+06 x[8,5]
   - service_time[8] + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.0000342428720902e+06 x[8,6]
   - service_time[8] + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.0000469444748068e+06 x[8,7]
   - service_time[8] + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] - service_time[8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0000474019358882e+06 x[8,9]
   - service_time[8] - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.0000135594043691e+06 x[8,10]
   - service_time[8] - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.0000131302091408e+06 x[8,11]
   - service_time[8] - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 1.0000332531050219e+06 x[8,12]
   - service_time[8] - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 1.000021622481183e+06 x[8,13]
   - service_time[8] - start_service[8] + start_service[13] >= -999999
 NodeArrivalVehicles[8,14]: - 1.0000612054747479e+06 x[8,14]
   - service_time[8] - start_service[8] + start_service[14] >= -999999
 NodeArrivalVehicles[8,15]: - 1.0000266352600046e+06 x[8,15]
   - service_time[8] - start_service[8] + start_service[15] >= -999999
 NodeArrivalVehicles[8,16]: - 1.0000342428720902e+06 x[8,16]
   - service_time[8] - start_service[8] + start_service[16] >= -999999
 NodeArrivalVehicles[8,17]: - 1.0000469444748068e+06 x[8,17]
   - service_time[8] - start_service[8] + start_service[17] >= -999999
 NodeArrivalVehicles[8,18]: - 999999 x[8,18] - service_time[8]
   - start_service[8] + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 1.0000474019358882e+06 x[8,19]
   - service_time[8] - start_service[8] + start_service[19] >= -999999
 NodeArrivalVehicles[8,20]: - 1.0000135594043691e+06 x[8,20]
   - service_time[8] - start_service[8] + start_service[20] >= -999999
 NodeArrivalVehicles[8,21]: - 1.0000131302091408e+06 x[8,21]
   - service_time[8] - start_service[8] + start_service[21] >= -999999
 NodeArrivalVehicles[8,22]: - 999999 x[8,22] - service_time[8]
   - start_service[8] + start_service[22] >= -999999
 NodeArrivalVehicles[8,23]: - 999999 x[8,23] - service_time[8]
   - start_service[8] + start_service[23] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0001135742778145e+06 x[9,1]
   - service_time[9] + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000784142247903e+06 x[9,2]
   - service_time[9] + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0000273896253681e+06 x[9,3]
   - service_time[9] + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 1.0000987723651387e+06 x[9,4]
   - service_time[9] + start_service[4] - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.0000704969536402e+06 x[9,5]
   - service_time[9] + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000678991589429e+06 x[9,6]
   - service_time[9] + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 999999.8437675878 x[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0000474019358882e+06 x[9,8]
   - service_time[9] + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] - service_time[9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.0000616475755909e+06 x[9,10]
   - service_time[9] - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000509634543548e+06 x[9,11]
   - service_time[9] - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 1.0000784142247903e+06 x[9,12]
   - service_time[9] - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 1.0000273896253681e+06 x[9,13]
   - service_time[9] - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[9,14]: - 1.0000987723651387e+06 x[9,14]
   - service_time[9] - start_service[9] + start_service[14] >= -999999
 NodeArrivalVehicles[9,15]: - 1.0000704969536402e+06 x[9,15]
   - service_time[9] - start_service[9] + start_service[15] >= -999999
 NodeArrivalVehicles[9,16]: - 1.0000678991589429e+06 x[9,16]
   - service_time[9] - start_service[9] + start_service[16] >= -999999
 NodeArrivalVehicles[9,17]: - 999999.8437675878 x[9,17] - service_time[9]
   - start_service[9] + start_service[17] >= -999999
 NodeArrivalVehicles[9,18]: - 1.0000474019358882e+06 x[9,18]
   - service_time[9] - start_service[9] + start_service[18] >= -999999
 NodeArrivalVehicles[9,19]: - 999999 x[9,19] - service_time[9]
   - start_service[9] + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 1.0000616475755909e+06 x[9,20]
   - service_time[9] - start_service[9] + start_service[20] >= -999999
 NodeArrivalVehicles[9,21]: - 1.0000509634543548e+06 x[9,21]
   - service_time[9] - start_service[9] + start_service[21] >= -999999
 NodeArrivalVehicles[9,22]: - 999999 x[9,22] - service_time[9]
   - start_service[9] + start_service[22] >= -999999
 NodeArrivalVehicles[9,23]: - 999999 x[9,23] - service_time[9]
   - start_service[9] + start_service[23] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000665133082913e+06 x[10,1]
   - service_time[10] + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000253782266368e+06 x[10,2]
   - service_time[10] + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.0000361135195905e+06 x[10,3]
   - service_time[10] + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.0000566241376635e+06 x[10,4]
   - service_time[10] + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 1.0000216805964843e+06 x[10,5]
   - service_time[10] + start_service[5] - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000287929886794e+06 x[10,6]
   - service_time[10] + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.00006122839988e+06 x[10,7]
   - service_time[10] + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.0000135594043691e+06 x[10,8]
   - service_time[10] + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.0000616475755909e+06 x[10,9]
   - service_time[10] + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] - service_time[10]
   >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000204820647413e+06 x[10,11]
   - service_time[10] - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 1.0000253782266368e+06 x[10,12]
   - service_time[10] - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 1.0000361135195905e+06 x[10,13]
   - service_time[10] - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[10,14]: - 1.0000566241376635e+06 x[10,14]
   - service_time[10] - start_service[10] + start_service[14] >= -999999
 NodeArrivalVehicles[10,15]: - 1.0000216805964843e+06 x[10,15]
   - service_time[10] - start_service[10] + start_service[15] >= -999999
 NodeArrivalVehicles[10,16]: - 1.0000287929886794e+06 x[10,16]
   - service_time[10] - start_service[10] + start_service[16] >= -999999
 NodeArrivalVehicles[10,17]: - 1.00006122839988e+06 x[10,17]
   - service_time[10] - start_service[10] + start_service[17] >= -999999
 NodeArrivalVehicles[10,18]: - 1.0000135594043691e+06 x[10,18]
   - service_time[10] - start_service[10] + start_service[18] >= -999999
 NodeArrivalVehicles[10,19]: - 1.0000616475755909e+06 x[10,19]
   - service_time[10] - start_service[10] + start_service[19] >= -999999
 NodeArrivalVehicles[10,20]: - 999999 x[10,20] - service_time[10]
   - start_service[10] + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 1.0000204820647413e+06 x[10,21]
   - service_time[10] - start_service[10] + start_service[21] >= -999999
 NodeArrivalVehicles[10,22]: - 999999 x[10,22] - service_time[10]
   - start_service[10] + start_service[22] >= -999999
 NodeArrivalVehicles[10,23]: - 999999 x[10,23] - service_time[10]
   - start_service[10] + start_service[23] >= -999999
 NodeArrivalVehicles[11,1]: - 1.0000632444353482e+06 x[11,1]
   - service_time[11] + start_service[1] - start_service[11] >= -999999
 NodeArrivalVehicles[11,2]: - 1.0000265613006055e+06 x[11,2]
   - service_time[11] + start_service[2] - start_service[11] >= -999999
 NodeArrivalVehicles[11,3]: - 1.0000225783656818e+06 x[11,3]
   - service_time[11] + start_service[3] - start_service[11] >= -999999
 NodeArrivalVehicles[11,4]: - 1.0000497453427232e+06 x[11,4]
   - service_time[11] + start_service[4] - start_service[11] >= -999999
 NodeArrivalVehicles[11,5]: - 1.0000185543180438e+06 x[11,5]
   - service_time[11] + start_service[5] - start_service[11] >= -999999
 NodeArrivalVehicles[11,6]: - 1.0000472983184841e+06 x[11,6]
   - service_time[11] + start_service[6] - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000503303973848e+06 x[11,7]
   - service_time[11] + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.0000131302091408e+06 x[11,8]
   - service_time[11] + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000509634543548e+06 x[11,9]
   - service_time[11] + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000204820647413e+06 x[11,10]
   - service_time[11] + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] - service_time[11]
   >= -999999
 NodeArrivalVehicles[11,12]: - 1.0000265613006055e+06 x[11,12]
   - service_time[11] - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 1.0000225783656818e+06 x[11,13]
   - service_time[11] - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[11,14]: - 1.0000497453427232e+06 x[11,14]
   - service_time[11] - start_service[11] + start_service[14] >= -999999
 NodeArrivalVehicles[11,15]: - 1.0000185543180438e+06 x[11,15]
   - service_time[11] - start_service[11] + start_service[15] >= -999999
 NodeArrivalVehicles[11,16]: - 1.0000472983184841e+06 x[11,16]
   - service_time[11] - start_service[11] + start_service[16] >= -999999
 NodeArrivalVehicles[11,17]: - 1.0000503303973848e+06 x[11,17]
   - service_time[11] - start_service[11] + start_service[17] >= -999999
 NodeArrivalVehicles[11,18]: - 1.0000131302091408e+06 x[11,18]
   - service_time[11] - start_service[11] + start_service[18] >= -999999
 NodeArrivalVehicles[11,19]: - 1.0000509634543548e+06 x[11,19]
   - service_time[11] - start_service[11] + start_service[19] >= -999999
 NodeArrivalVehicles[11,20]: - 1.0000204820647413e+06 x[11,20]
   - service_time[11] - start_service[11] + start_service[20] >= -999999
 NodeArrivalVehicles[11,21]: - 999999 x[11,21] - service_time[11]
   - start_service[11] + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 999999 x[11,22] - service_time[11]
   - start_service[11] + start_service[22] >= -999999
 NodeArrivalVehicles[11,23]: - 999999 x[11,23] - service_time[11]
   - start_service[11] + start_service[23] >= -999999
 NodeArrivalVehicles[12,1]: - 1.0000401805928103e+06 x[12,1]
   - service_time[12] + start_service[1] - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 1.0000500807426213e+06 x[12,3]
   - service_time[12] + start_service[3] - start_service[12] >= -999999
 NodeArrivalVehicles[12,4]: - 1.000030792514505e+06 x[12,4]
   - service_time[12] + start_service[4] - start_service[12] >= -999999
 NodeArrivalVehicles[12,5]: - 1.0000071143237508e+06 x[12,5]
   - service_time[12] + start_service[5] - start_service[12] >= -999999
 NodeArrivalVehicles[12,6]: - 1.0000533055907747e+06 x[12,6]
   - service_time[12] + start_service[6] - start_service[12] >= -999999
 NodeArrivalVehicles[12,7]: - 1.0000778020430846e+06 x[12,7]
   - service_time[12] + start_service[7] - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000332531050219e+06 x[12,8]
   - service_time[12] + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000784142247903e+06 x[12,9]
   - service_time[12] + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000253782266368e+06 x[12,10]
   - service_time[12] + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.0000265613006055e+06 x[12,11]
   - service_time[12] + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] - service_time[12]
   >= -999999
 NodeArrivalVehicles[12,13]: - 1.0000500807426213e+06 x[12,13]
   - service_time[12] - start_service[12] + start_service[13] >= -999999
 NodeArrivalVehicles[12,14]: - 1.000030792514505e+06 x[12,14]
   - service_time[12] - start_service[12] + start_service[14] >= -999999
 NodeArrivalVehicles[12,15]: - 1.0000071143237508e+06 x[12,15]
   - service_time[12] - start_service[12] + start_service[15] >= -999999
 NodeArrivalVehicles[12,16]: - 1.0000533055907747e+06 x[12,16]
   - service_time[12] - start_service[12] + start_service[16] >= -999999
 NodeArrivalVehicles[12,17]: - 1.0000778020430846e+06 x[12,17]
   - service_time[12] - start_service[12] + start_service[17] >= -999999
 NodeArrivalVehicles[12,18]: - 1.0000332531050219e+06 x[12,18]
   - service_time[12] - start_service[12] + start_service[18] >= -999999
 NodeArrivalVehicles[12,19]: - 1.0000784142247903e+06 x[12,19]
   - service_time[12] - start_service[12] + start_service[19] >= -999999
 NodeArrivalVehicles[12,20]: - 1.0000253782266368e+06 x[12,20]
   - service_time[12] - start_service[12] + start_service[20] >= -999999
 NodeArrivalVehicles[12,21]: - 1.0000265613006055e+06 x[12,21]
   - service_time[12] - start_service[12] + start_service[21] >= -999999
 NodeArrivalVehicles[12,22]: - 999999 x[12,22] - service_time[12]
   - start_service[12] + start_service[22] >= -999999
 NodeArrivalVehicles[12,23]: - 999999 x[12,23] - service_time[12]
   - start_service[12] + start_service[23] >= -999999
 NodeArrivalVehicles[13,1]: - 1.0000857578554453e+06 x[13,1]
   - service_time[13] + start_service[1] - start_service[13] >= -999999
 NodeArrivalVehicles[13,2]: - 1.0000500807426213e+06 x[13,2]
   - service_time[13] + start_service[2] - start_service[13] >= -999999
 NodeArrivalVehicles[13,3]: - 999999 x[13,3] - service_time[13]
   + start_service[3] - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 1.0000713740287239e+06 x[13,4]
   - service_time[13] + start_service[4] - start_service[13] >= -999999
 NodeArrivalVehicles[13,5]: - 1.000042124286405e+06 x[13,5]
   - service_time[13] + start_service[5] - start_service[13] >= -999999
 NodeArrivalVehicles[13,6]: - 1.0000518389269916e+06 x[13,6]
   - service_time[13] + start_service[6] - start_service[13] >= -999999
 NodeArrivalVehicles[13,7]: - 1.0000267523298946e+06 x[13,7]
   - service_time[13] + start_service[7] - start_service[13] >= -999999
 NodeArrivalVehicles[13,8]: - 1.000021622481183e+06 x[13,8]
   - service_time[13] + start_service[8] - start_service[13] >= -999999
 NodeArrivalVehicles[13,9]: - 1.0000273896253681e+06 x[13,9]
   - service_time[13] + start_service[9] - start_service[13] >= -999999
 NodeArrivalVehicles[13,10]: - 1.0000361135195905e+06 x[13,10]
   - service_time[13] + start_service[10] - start_service[13] >= -999999
 NodeArrivalVehicles[13,11]: - 1.0000225783656818e+06 x[13,11]
   - service_time[13] + start_service[11] - start_service[13] >= -999999
 NodeArrivalVehicles[13,12]: - 1.0000500807426213e+06 x[13,12]
   - service_time[13] + start_service[12] - start_service[13] >= -999999
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] - service_time[13]
   >= -999999
 NodeArrivalVehicles[13,14]: - 1.0000713740287239e+06 x[13,14]
   - service_time[13] - start_service[13] + start_service[14] >= -999999
 NodeArrivalVehicles[13,15]: - 1.000042124286405e+06 x[13,15]
   - service_time[13] - start_service[13] + start_service[15] >= -999999
 NodeArrivalVehicles[13,16]: - 1.0000518389269916e+06 x[13,16]
   - service_time[13] - start_service[13] + start_service[16] >= -999999
 NodeArrivalVehicles[13,17]: - 1.0000267523298946e+06 x[13,17]
   - service_time[13] - start_service[13] + start_service[17] >= -999999
 NodeArrivalVehicles[13,18]: - 1.000021622481183e+06 x[13,18]
   - service_time[13] - start_service[13] + start_service[18] >= -999999
 NodeArrivalVehicles[13,19]: - 1.0000273896253681e+06 x[13,19]
   - service_time[13] - start_service[13] + start_service[19] >= -999999
 NodeArrivalVehicles[13,20]: - 1.0000361135195905e+06 x[13,20]
   - service_time[13] - start_service[13] + start_service[20] >= -999999
 NodeArrivalVehicles[13,21]: - 1.0000225783656818e+06 x[13,21]
   - service_time[13] - start_service[13] + start_service[21] >= -999999
 NodeArrivalVehicles[13,22]: - 999999 x[13,22] - service_time[13]
   - start_service[13] + start_service[22] >= -999999
 NodeArrivalVehicles[13,23]: - 999999 x[13,23] - service_time[13]
   - start_service[13] + start_service[23] >= -999999
 NodeArrivalVehicles[14,1]: - 1.0000147366179901e+06 x[14,1]
   - service_time[14] + start_service[1] - start_service[14] >= -999999
 NodeArrivalVehicles[14,2]: - 1.000030792514505e+06 x[14,2]
   - service_time[14] + start_service[2] - start_service[14] >= -999999
 NodeArrivalVehicles[14,3]: - 1.0000713740287239e+06 x[14,3]
   - service_time[14] + start_service[3] - start_service[14] >= -999999
 NodeArrivalVehicles[14,4]: - 999999 x[14,4] - service_time[14]
   + start_service[4] - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 1.0000343374844047e+06 x[14,5]
   - service_time[14] + start_service[5] - start_service[14] >= -999999
 NodeArrivalVehicles[14,6]: - 1.0000850981022327e+06 x[14,6]
   - service_time[14] + start_service[6] - start_service[14] >= -999999
 NodeArrivalVehicles[14,7]: - 1.0000980247615908e+06 x[14,7]
   - service_time[14] + start_service[7] - start_service[14] >= -999999
 NodeArrivalVehicles[14,8]: - 1.0000612054747479e+06 x[14,8]
   - service_time[14] + start_service[8] - start_service[14] >= -999999
 NodeArrivalVehicles[14,9]: - 1.0000987723651387e+06 x[14,9]
   - service_time[14] + start_service[9] - start_service[14] >= -999999
 NodeArrivalVehicles[14,10]: - 1.0000566241376635e+06 x[14,10]
   - service_time[14] + start_service[10] - start_service[14] >= -999999
 NodeArrivalVehicles[14,11]: - 1.0000497453427232e+06 x[14,11]
   - service_time[14] + start_service[11] - start_service[14] >= -999999
 NodeArrivalVehicles[14,12]: - 1.000030792514505e+06 x[14,12]
   - service_time[14] + start_service[12] - start_service[14] >= -999999
 NodeArrivalVehicles[14,13]: - 1.0000713740287239e+06 x[14,13]
   - service_time[14] + start_service[13] - start_service[14] >= -999999
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] - service_time[14]
   >= -999999
 NodeArrivalVehicles[14,15]: - 1.0000343374844047e+06 x[14,15]
   - service_time[14] - start_service[14] + start_service[15] >= -999999
 NodeArrivalVehicles[14,16]: - 1.0000850981022327e+06 x[14,16]
   - service_time[14] - start_service[14] + start_service[16] >= -999999
 NodeArrivalVehicles[14,17]: - 1.0000980247615908e+06 x[14,17]
   - service_time[14] - start_service[14] + start_service[17] >= -999999
 NodeArrivalVehicles[14,18]: - 1.0000612054747479e+06 x[14,18]
   - service_time[14] - start_service[14] + start_service[18] >= -999999
 NodeArrivalVehicles[14,19]: - 1.0000987723651387e+06 x[14,19]
   - service_time[14] - start_service[14] + start_service[19] >= -999999
 NodeArrivalVehicles[14,20]: - 1.0000566241376635e+06 x[14,20]
   - service_time[14] - start_service[14] + start_service[20] >= -999999
 NodeArrivalVehicles[14,21]: - 1.0000497453427232e+06 x[14,21]
   - service_time[14] - start_service[14] + start_service[21] >= -999999
 NodeArrivalVehicles[14,22]: - 999999 x[14,22] - service_time[14]
   - start_service[14] + start_service[22] >= -999999
 NodeArrivalVehicles[14,23]: - 999999 x[14,23] - service_time[14]
   - start_service[14] + start_service[23] >= -999999
 NodeArrivalVehicles[15,1]: - 1.0000457716821638e+06 x[15,1]
   - service_time[15] + start_service[1] - start_service[15] >= -999999
 NodeArrivalVehicles[15,2]: - 1.0000071143237508e+06 x[15,2]
   - service_time[15] + start_service[2] - start_service[15] >= -999999
 NodeArrivalVehicles[15,3]: - 1.000042124286405e+06 x[15,3]
   - service_time[15] + start_service[3] - start_service[15] >= -999999
 NodeArrivalVehicles[15,4]: - 1.0000343374844047e+06 x[15,4]
   - service_time[15] + start_service[4] - start_service[15] >= -999999
 NodeArrivalVehicles[15,5]: - 999999 x[15,5] - service_time[15]
   + start_service[5] - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 1.0000511580216251e+06 x[15,6]
   - service_time[15] + start_service[6] - start_service[15] >= -999999
 NodeArrivalVehicles[15,7]: - 1.0000698714195783e+06 x[15,7]
   - service_time[15] + start_service[7] - start_service[15] >= -999999
 NodeArrivalVehicles[15,8]: - 1.0000266352600046e+06 x[15,8]
   - service_time[15] + start_service[8] - start_service[15] >= -999999
 NodeArrivalVehicles[15,9]: - 1.0000704969536402e+06 x[15,9]
   - service_time[15] + start_service[9] - start_service[15] >= -999999
 NodeArrivalVehicles[15,10]: - 1.0000216805964843e+06 x[15,10]
   - service_time[15] + start_service[10] - start_service[15] >= -999999
 NodeArrivalVehicles[15,11]: - 1.0000185543180438e+06 x[15,11]
   - service_time[15] + start_service[11] - start_service[15] >= -999999
 NodeArrivalVehicles[15,12]: - 1.0000071143237508e+06 x[15,12]
   - service_time[15] + start_service[12] - start_service[15] >= -999999
 NodeArrivalVehicles[15,13]: - 1.000042124286405e+06 x[15,13]
   - service_time[15] + start_service[13] - start_service[15] >= -999999
 NodeArrivalVehicles[15,14]: - 1.0000343374844047e+06 x[15,14]
   - service_time[15] + start_service[14] - start_service[15] >= -999999
 NodeArrivalVehicles[15,15]: - 999999 x[15,15] - service_time[15]
   >= -999999
 NodeArrivalVehicles[15,16]: - 1.0000511580216251e+06 x[15,16]
   - service_time[15] - start_service[15] + start_service[16] >= -999999
 NodeArrivalVehicles[15,17]: - 1.0000698714195783e+06 x[15,17]
   - service_time[15] - start_service[15] + start_service[17] >= -999999
 NodeArrivalVehicles[15,18]: - 1.0000266352600046e+06 x[15,18]
   - service_time[15] - start_service[15] + start_service[18] >= -999999
 NodeArrivalVehicles[15,19]: - 1.0000704969536402e+06 x[15,19]
   - service_time[15] - start_service[15] + start_service[19] >= -999999
 NodeArrivalVehicles[15,20]: - 1.0000216805964843e+06 x[15,20]
   - service_time[15] - start_service[15] + start_service[20] >= -999999
 NodeArrivalVehicles[15,21]: - 1.0000185543180438e+06 x[15,21]
   - service_time[15] - start_service[15] + start_service[21] >= -999999
 NodeArrivalVehicles[15,22]: - 999999 x[15,22] - service_time[15]
   - start_service[15] + start_service[22] >= -999999
 NodeArrivalVehicles[15,23]: - 999999 x[15,23] - service_time[15]
   - start_service[15] + start_service[23] >= -999999
 NodeArrivalVehicles[16,1]: - 1.0000930531941031e+06 x[16,1]
   - service_time[16] + start_service[1] - start_service[16] >= -999999
 NodeArrivalVehicles[16,2]: - 1.0000533055907747e+06 x[16,2]
   - service_time[16] + start_service[2] - start_service[16] >= -999999
 NodeArrivalVehicles[16,3]: - 1.0000518389269916e+06 x[16,3]
   - service_time[16] + start_service[3] - start_service[16] >= -999999
 NodeArrivalVehicles[16,4]: - 1.0000850981022327e+06 x[16,4]
   - service_time[16] + start_service[4] - start_service[16] >= -999999
 NodeArrivalVehicles[16,5]: - 1.0000511580216251e+06 x[16,5]
   - service_time[16] + start_service[5] - start_service[16] >= -999999
 NodeArrivalVehicles[16,6]: - 999999 x[16,6] - service_time[16]
   + start_service[6] - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 1.0000678379998124e+06 x[16,7]
   - service_time[16] + start_service[7] - start_service[16] >= -999999
 NodeArrivalVehicles[16,8]: - 1.0000342428720902e+06 x[16,8]
   - service_time[16] + start_service[8] - start_service[16] >= -999999
 NodeArrivalVehicles[16,9]: - 1.0000678991589429e+06 x[16,9]
   - service_time[16] + start_service[9] - start_service[16] >= -999999
 NodeArrivalVehicles[16,10]: - 1.0000287929886794e+06 x[16,10]
   - service_time[16] + start_service[10] - start_service[16] >= -999999
 NodeArrivalVehicles[16,11]: - 1.0000472983184841e+06 x[16,11]
   - service_time[16] + start_service[11] - start_service[16] >= -999999
 NodeArrivalVehicles[16,12]: - 1.0000533055907747e+06 x[16,12]
   - service_time[16] + start_service[12] - start_service[16] >= -999999
 NodeArrivalVehicles[16,13]: - 1.0000518389269916e+06 x[16,13]
   - service_time[16] + start_service[13] - start_service[16] >= -999999
 NodeArrivalVehicles[16,14]: - 1.0000850981022327e+06 x[16,14]
   - service_time[16] + start_service[14] - start_service[16] >= -999999
 NodeArrivalVehicles[16,15]: - 1.0000511580216251e+06 x[16,15]
   - service_time[16] + start_service[15] - start_service[16] >= -999999
 NodeArrivalVehicles[16,16]: - 999999 x[16,16] - service_time[16]
   >= -999999
 NodeArrivalVehicles[16,17]: - 1.0000678379998124e+06 x[16,17]
   - service_time[16] - start_service[16] + start_service[17] >= -999999
 NodeArrivalVehicles[16,18]: - 1.0000342428720902e+06 x[16,18]
   - service_time[16] - start_service[16] + start_service[18] >= -999999
 NodeArrivalVehicles[16,19]: - 1.0000678991589429e+06 x[16,19]
   - service_time[16] - start_service[16] + start_service[19] >= -999999
 NodeArrivalVehicles[16,20]: - 1.0000287929886794e+06 x[16,20]
   - service_time[16] - start_service[16] + start_service[20] >= -999999
 NodeArrivalVehicles[16,21]: - 1.0000472983184841e+06 x[16,21]
   - service_time[16] - start_service[16] + start_service[21] >= -999999
 NodeArrivalVehicles[16,22]: - 999999 x[16,22] - service_time[16]
   - start_service[16] + start_service[22] >= -999999
 NodeArrivalVehicles[16,23]: - 999999 x[16,23] - service_time[16]
   - start_service[16] + start_service[23] >= -999999
 NodeArrivalVehicles[17,1]: - 1.0001128471096832e+06 x[17,1]
   - service_time[17] + start_service[1] - start_service[17] >= -999999
 NodeArrivalVehicles[17,2]: - 1.0000778020430846e+06 x[17,2]
   - service_time[17] + start_service[2] - start_service[17] >= -999999
 NodeArrivalVehicles[17,3]: - 1.0000267523298946e+06 x[17,3]
   - service_time[17] + start_service[3] - start_service[17] >= -999999
 NodeArrivalVehicles[17,4]: - 1.0000980247615908e+06 x[17,4]
   - service_time[17] + start_service[4] - start_service[17] >= -999999
 NodeArrivalVehicles[17,5]: - 1.0000698714195783e+06 x[17,5]
   - service_time[17] + start_service[5] - start_service[17] >= -999999
 NodeArrivalVehicles[17,6]: - 1.0000678379998124e+06 x[17,6]
   - service_time[17] + start_service[6] - start_service[17] >= -999999
 NodeArrivalVehicles[17,7]: - 999999 x[17,7] - service_time[17]
   + start_service[7] - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 1.0000469444748068e+06 x[17,8]
   - service_time[17] + start_service[8] - start_service[17] >= -999999
 NodeArrivalVehicles[17,9]: - 999999.8437675878 x[17,9] - service_time[17]
   + start_service[9] - start_service[17] >= -999999
 NodeArrivalVehicles[17,10]: - 1.00006122839988e+06 x[17,10]
   - service_time[17] + start_service[10] - start_service[17] >= -999999
 NodeArrivalVehicles[17,11]: - 1.0000503303973848e+06 x[17,11]
   - service_time[17] + start_service[11] - start_service[17] >= -999999
 NodeArrivalVehicles[17,12]: - 1.0000778020430846e+06 x[17,12]
   - service_time[17] + start_service[12] - start_service[17] >= -999999
 NodeArrivalVehicles[17,13]: - 1.0000267523298946e+06 x[17,13]
   - service_time[17] + start_service[13] - start_service[17] >= -999999
 NodeArrivalVehicles[17,14]: - 1.0000980247615908e+06 x[17,14]
   - service_time[17] + start_service[14] - start_service[17] >= -999999
 NodeArrivalVehicles[17,15]: - 1.0000698714195783e+06 x[17,15]
   - service_time[17] + start_service[15] - start_service[17] >= -999999
 NodeArrivalVehicles[17,16]: - 1.0000678379998124e+06 x[17,16]
   - service_time[17] + start_service[16] - start_service[17] >= -999999
 NodeArrivalVehicles[17,17]: - 999999 x[17,17] - service_time[17]
   >= -999999
 NodeArrivalVehicles[17,18]: - 1.0000469444748068e+06 x[17,18]
   - service_time[17] - start_service[17] + start_service[18] >= -999999
 NodeArrivalVehicles[17,19]: - 999999.8437675878 x[17,19]
   - service_time[17] - start_service[17] + start_service[19] >= -999999
 NodeArrivalVehicles[17,20]: - 1.00006122839988e+06 x[17,20]
   - service_time[17] - start_service[17] + start_service[20] >= -999999
 NodeArrivalVehicles[17,21]: - 1.0000503303973848e+06 x[17,21]
   - service_time[17] - start_service[17] + start_service[21] >= -999999
 NodeArrivalVehicles[17,22]: - 999999 x[17,22] - service_time[17]
   - start_service[17] + start_service[22] >= -999999
 NodeArrivalVehicles[17,23]: - 999999 x[17,23] - service_time[17]
   - start_service[17] + start_service[23] >= -999999
 NodeArrivalVehicles[18,1]: - 1.0000733867898309e+06 x[18,1]
   - service_time[18] + start_service[1] - start_service[18] >= -999999
 NodeArrivalVehicles[18,2]: - 1.0000332531050219e+06 x[18,2]
   - service_time[18] + start_service[2] - start_service[18] >= -999999
 NodeArrivalVehicles[18,3]: - 1.000021622481183e+06 x[18,3]
   - service_time[18] + start_service[3] - start_service[18] >= -999999
 NodeArrivalVehicles[18,4]: - 1.0000612054747479e+06 x[18,4]
   - service_time[18] + start_service[4] - start_service[18] >= -999999
 NodeArrivalVehicles[18,5]: - 1.0000266352600046e+06 x[18,5]
   - service_time[18] + start_service[5] - start_service[18] >= -999999
 NodeArrivalVehicles[18,6]: - 1.0000342428720902e+06 x[18,6]
   - service_time[18] + start_service[6] - start_service[18] >= -999999
 NodeArrivalVehicles[18,7]: - 1.0000469444748068e+06 x[18,7]
   - service_time[18] + start_service[7] - start_service[18] >= -999999
 NodeArrivalVehicles[18,8]: - 999999 x[18,8] - service_time[18]
   + start_service[8] - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 1.0000474019358882e+06 x[18,9]
   - service_time[18] + start_service[9] - start_service[18] >= -999999
 NodeArrivalVehicles[18,10]: - 1.0000135594043691e+06 x[18,10]
   - service_time[18] + start_service[10] - start_service[18] >= -999999
 NodeArrivalVehicles[18,11]: - 1.0000131302091408e+06 x[18,11]
   - service_time[18] + start_service[11] - start_service[18] >= -999999
 NodeArrivalVehicles[18,12]: - 1.0000332531050219e+06 x[18,12]
   - service_time[18] + start_service[12] - start_service[18] >= -999999
 NodeArrivalVehicles[18,13]: - 1.000021622481183e+06 x[18,13]
   - service_time[18] + start_service[13] - start_service[18] >= -999999
 NodeArrivalVehicles[18,14]: - 1.0000612054747479e+06 x[18,14]
   - service_time[18] + start_service[14] - start_service[18] >= -999999
 NodeArrivalVehicles[18,15]: - 1.0000266352600046e+06 x[18,15]
   - service_time[18] + start_service[15] - start_service[18] >= -999999
 NodeArrivalVehicles[18,16]: - 1.0000342428720902e+06 x[18,16]
   - service_time[18] + start_service[16] - start_service[18] >= -999999
 NodeArrivalVehicles[18,17]: - 1.0000469444748068e+06 x[18,17]
   - service_time[18] + start_service[17] - start_service[18] >= -999999
 NodeArrivalVehicles[18,18]: - 999999 x[18,18] - service_time[18]
   >= -999999
 NodeArrivalVehicles[18,19]: - 1.0000474019358882e+06 x[18,19]
   - service_time[18] - start_service[18] + start_service[19] >= -999999
 NodeArrivalVehicles[18,20]: - 1.0000135594043691e+06 x[18,20]
   - service_time[18] - start_service[18] + start_service[20] >= -999999
 NodeArrivalVehicles[18,21]: - 1.0000131302091408e+06 x[18,21]
   - service_time[18] - start_service[18] + start_service[21] >= -999999
 NodeArrivalVehicles[18,22]: - 999999 x[18,22] - service_time[18]
   - start_service[18] + start_service[22] >= -999999
 NodeArrivalVehicles[18,23]: - 999999 x[18,23] - service_time[18]
   - start_service[18] + start_service[23] >= -999999
 NodeArrivalVehicles[19,1]: - 1.0001135742778145e+06 x[19,1]
   - service_time[19] + start_service[1] - start_service[19] >= -999999
 NodeArrivalVehicles[19,2]: - 1.0000784142247903e+06 x[19,2]
   - service_time[19] + start_service[2] - start_service[19] >= -999999
 NodeArrivalVehicles[19,3]: - 1.0000273896253681e+06 x[19,3]
   - service_time[19] + start_service[3] - start_service[19] >= -999999
 NodeArrivalVehicles[19,4]: - 1.0000987723651387e+06 x[19,4]
   - service_time[19] + start_service[4] - start_service[19] >= -999999
 NodeArrivalVehicles[19,5]: - 1.0000704969536402e+06 x[19,5]
   - service_time[19] + start_service[5] - start_service[19] >= -999999
 NodeArrivalVehicles[19,6]: - 1.0000678991589429e+06 x[19,6]
   - service_time[19] + start_service[6] - start_service[19] >= -999999
 NodeArrivalVehicles[19,7]: - 999999.8437675878 x[19,7] - service_time[19]
   + start_service[7] - start_service[19] >= -999999
 NodeArrivalVehicles[19,8]: - 1.0000474019358882e+06 x[19,8]
   - service_time[19] + start_service[8] - start_service[19] >= -999999
 NodeArrivalVehicles[19,9]: - 999999 x[19,9] - service_time[19]
   + start_service[9] - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 1.0000616475755909e+06 x[19,10]
   - service_time[19] + start_service[10] - start_service[19] >= -999999
 NodeArrivalVehicles[19,11]: - 1.0000509634543548e+06 x[19,11]
   - service_time[19] + start_service[11] - start_service[19] >= -999999
 NodeArrivalVehicles[19,12]: - 1.0000784142247903e+06 x[19,12]
   - service_time[19] + start_service[12] - start_service[19] >= -999999
 NodeArrivalVehicles[19,13]: - 1.0000273896253681e+06 x[19,13]
   - service_time[19] + start_service[13] - start_service[19] >= -999999
 NodeArrivalVehicles[19,14]: - 1.0000987723651387e+06 x[19,14]
   - service_time[19] + start_service[14] - start_service[19] >= -999999
 NodeArrivalVehicles[19,15]: - 1.0000704969536402e+06 x[19,15]
   - service_time[19] + start_service[15] - start_service[19] >= -999999
 NodeArrivalVehicles[19,16]: - 1.0000678991589429e+06 x[19,16]
   - service_time[19] + start_service[16] - start_service[19] >= -999999
 NodeArrivalVehicles[19,17]: - 999999.8437675878 x[19,17]
   - service_time[19] + start_service[17] - start_service[19] >= -999999
 NodeArrivalVehicles[19,18]: - 1.0000474019358882e+06 x[19,18]
   - service_time[19] + start_service[18] - start_service[19] >= -999999
 NodeArrivalVehicles[19,19]: - 999999 x[19,19] - service_time[19]
   >= -999999
 NodeArrivalVehicles[19,20]: - 1.0000616475755909e+06 x[19,20]
   - service_time[19] - start_service[19] + start_service[20] >= -999999
 NodeArrivalVehicles[19,21]: - 1.0000509634543548e+06 x[19,21]
   - service_time[19] - start_service[19] + start_service[21] >= -999999
 NodeArrivalVehicles[19,22]: - 999999 x[19,22] - service_time[19]
   - start_service[19] + start_service[22] >= -999999
 NodeArrivalVehicles[19,23]: - 999999 x[19,23] - service_time[19]
   - start_service[19] + start_service[23] >= -999999
 NodeArrivalVehicles[20,1]: - 1.0000665133082913e+06 x[20,1]
   - service_time[20] + start_service[1] - start_service[20] >= -999999
 NodeArrivalVehicles[20,2]: - 1.0000253782266368e+06 x[20,2]
   - service_time[20] + start_service[2] - start_service[20] >= -999999
 NodeArrivalVehicles[20,3]: - 1.0000361135195905e+06 x[20,3]
   - service_time[20] + start_service[3] - start_service[20] >= -999999
 NodeArrivalVehicles[20,4]: - 1.0000566241376635e+06 x[20,4]
   - service_time[20] + start_service[4] - start_service[20] >= -999999
 NodeArrivalVehicles[20,5]: - 1.0000216805964843e+06 x[20,5]
   - service_time[20] + start_service[5] - start_service[20] >= -999999
 NodeArrivalVehicles[20,6]: - 1.0000287929886794e+06 x[20,6]
   - service_time[20] + start_service[6] - start_service[20] >= -999999
 NodeArrivalVehicles[20,7]: - 1.00006122839988e+06 x[20,7]
   - service_time[20] + start_service[7] - start_service[20] >= -999999
 NodeArrivalVehicles[20,8]: - 1.0000135594043691e+06 x[20,8]
   - service_time[20] + start_service[8] - start_service[20] >= -999999
 NodeArrivalVehicles[20,9]: - 1.0000616475755909e+06 x[20,9]
   - service_time[20] + start_service[9] - start_service[20] >= -999999
 NodeArrivalVehicles[20,10]: - 999999 x[20,10] - service_time[20]
   + start_service[10] - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 1.0000204820647413e+06 x[20,11]
   - service_time[20] + start_service[11] - start_service[20] >= -999999
 NodeArrivalVehicles[20,12]: - 1.0000253782266368e+06 x[20,12]
   - service_time[20] + start_service[12] - start_service[20] >= -999999
 NodeArrivalVehicles[20,13]: - 1.0000361135195905e+06 x[20,13]
   - service_time[20] + start_service[13] - start_service[20] >= -999999
 NodeArrivalVehicles[20,14]: - 1.0000566241376635e+06 x[20,14]
   - service_time[20] + start_service[14] - start_service[20] >= -999999
 NodeArrivalVehicles[20,15]: - 1.0000216805964843e+06 x[20,15]
   - service_time[20] + start_service[15] - start_service[20] >= -999999
 NodeArrivalVehicles[20,16]: - 1.0000287929886794e+06 x[20,16]
   - service_time[20] + start_service[16] - start_service[20] >= -999999
 NodeArrivalVehicles[20,17]: - 1.00006122839988e+06 x[20,17]
   - service_time[20] + start_service[17] - start_service[20] >= -999999
 NodeArrivalVehicles[20,18]: - 1.0000135594043691e+06 x[20,18]
   - service_time[20] + start_service[18] - start_service[20] >= -999999
 NodeArrivalVehicles[20,19]: - 1.0000616475755909e+06 x[20,19]
   - service_time[20] + start_service[19] - start_service[20] >= -999999
 NodeArrivalVehicles[20,20]: - 999999 x[20,20] - service_time[20]
   >= -999999
 NodeArrivalVehicles[20,21]: - 1.0000204820647413e+06 x[20,21]
   - service_time[20] - start_service[20] + start_service[21] >= -999999
 NodeArrivalVehicles[20,22]: - 999999 x[20,22] - service_time[20]
   - start_service[20] + start_service[22] >= -999999
 NodeArrivalVehicles[20,23]: - 999999 x[20,23] - service_time[20]
   - start_service[20] + start_service[23] >= -999999
 NodeArrivalVehicles[21,1]: - 1.0000632444353482e+06 x[21,1]
   - service_time[21] + start_service[1] - start_service[21] >= -999999
 NodeArrivalVehicles[21,2]: - 1.0000265613006055e+06 x[21,2]
   - service_time[21] + start_service[2] - start_service[21] >= -999999
 NodeArrivalVehicles[21,3]: - 1.0000225783656818e+06 x[21,3]
   - service_time[21] + start_service[3] - start_service[21] >= -999999
 NodeArrivalVehicles[21,4]: - 1.0000497453427232e+06 x[21,4]
   - service_time[21] + start_service[4] - start_service[21] >= -999999
 NodeArrivalVehicles[21,5]: - 1.0000185543180438e+06 x[21,5]
   - service_time[21] + start_service[5] - start_service[21] >= -999999
 NodeArrivalVehicles[21,6]: - 1.0000472983184841e+06 x[21,6]
   - service_time[21] + start_service[6] - start_service[21] >= -999999
 NodeArrivalVehicles[21,7]: - 1.0000503303973848e+06 x[21,7]
   - service_time[21] + start_service[7] - start_service[21] >= -999999
 NodeArrivalVehicles[21,8]: - 1.0000131302091408e+06 x[21,8]
   - service_time[21] + start_service[8] - start_service[21] >= -999999
 NodeArrivalVehicles[21,9]: - 1.0000509634543548e+06 x[21,9]
   - service_time[21] + start_service[9] - start_service[21] >= -999999
 NodeArrivalVehicles[21,10]: - 1.0000204820647413e+06 x[21,10]
   - service_time[21] + start_service[10] - start_service[21] >= -999999
 NodeArrivalVehicles[21,11]: - 999999 x[21,11] - service_time[21]
   + start_service[11] - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 1.0000265613006055e+06 x[21,12]
   - service_time[21] + start_service[12] - start_service[21] >= -999999
 NodeArrivalVehicles[21,13]: - 1.0000225783656818e+06 x[21,13]
   - service_time[21] + start_service[13] - start_service[21] >= -999999
 NodeArrivalVehicles[21,14]: - 1.0000497453427232e+06 x[21,14]
   - service_time[21] + start_service[14] - start_service[21] >= -999999
 NodeArrivalVehicles[21,15]: - 1.0000185543180438e+06 x[21,15]
   - service_time[21] + start_service[15] - start_service[21] >= -999999
 NodeArrivalVehicles[21,16]: - 1.0000472983184841e+06 x[21,16]
   - service_time[21] + start_service[16] - start_service[21] >= -999999
 NodeArrivalVehicles[21,17]: - 1.0000503303973848e+06 x[21,17]
   - service_time[21] + start_service[17] - start_service[21] >= -999999
 NodeArrivalVehicles[21,18]: - 1.0000131302091408e+06 x[21,18]
   - service_time[21] + start_service[18] - start_service[21] >= -999999
 NodeArrivalVehicles[21,19]: - 1.0000509634543548e+06 x[21,19]
   - service_time[21] + start_service[19] - start_service[21] >= -999999
 NodeArrivalVehicles[21,20]: - 1.0000204820647413e+06 x[21,20]
   - service_time[21] + start_service[20] - start_service[21] >= -999999
 NodeArrivalVehicles[21,21]: - 999999 x[21,21] - service_time[21]
   >= -999999
 NodeArrivalVehicles[21,22]: - 999999 x[21,22] - service_time[21]
   - start_service[21] + start_service[22] >= -999999
 NodeArrivalVehicles[21,23]: - 999999 x[21,23] - service_time[21]
   - start_service[21] + start_service[23] >= -999999
 NodeArrivalVehicles[22,1]: - 999999 x[22,1] - service_time[22]
   + start_service[1] - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 1.0000401805928103e+06 x[22,2]
   - service_time[22] + start_service[2] - start_service[22] >= -999999
 NodeArrivalVehicles[22,3]: - 1.0000857578554453e+06 x[22,3]
   - service_time[22] + start_service[3] - start_service[22] >= -999999
 NodeArrivalVehicles[22,4]: - 1.0000147366179901e+06 x[22,4]
   - service_time[22] + start_service[4] - start_service[22] >= -999999
 NodeArrivalVehicles[22,5]: - 1.0000457716821638e+06 x[22,5]
   - service_time[22] + start_service[5] - start_service[22] >= -999999
 NodeArrivalVehicles[22,6]: - 1.0000930531941031e+06 x[22,6]
   - service_time[22] + start_service[6] - start_service[22] >= -999999
 NodeArrivalVehicles[22,7]: - 1.0001128471096832e+06 x[22,7]
   - service_time[22] + start_service[7] - start_service[22] >= -999999
 NodeArrivalVehicles[22,8]: - 1.0000733867898309e+06 x[22,8]
   - service_time[22] + start_service[8] - start_service[22] >= -999999
 NodeArrivalVehicles[22,9]: - 1.0001135742778145e+06 x[22,9]
   - service_time[22] + start_service[9] - start_service[22] >= -999999
 NodeArrivalVehicles[22,10]: - 1.0000665133082913e+06 x[22,10]
   - service_time[22] + start_service[10] - start_service[22] >= -999999
 NodeArrivalVehicles[22,11]: - 1.0000632444353482e+06 x[22,11]
   - service_time[22] + start_service[11] - start_service[22] >= -999999
 NodeArrivalVehicles[22,12]: - 1.0000401805928103e+06 x[22,12]
   - service_time[22] + start_service[12] - start_service[22] >= -999999
 NodeArrivalVehicles[22,13]: - 1.0000857578554453e+06 x[22,13]
   - service_time[22] + start_service[13] - start_service[22] >= -999999
 NodeArrivalVehicles[22,14]: - 1.0000147366179901e+06 x[22,14]
   - service_time[22] + start_service[14] - start_service[22] >= -999999
 NodeArrivalVehicles[22,15]: - 1.0000457716821638e+06 x[22,15]
   - service_time[22] + start_service[15] - start_service[22] >= -999999
 NodeArrivalVehicles[22,16]: - 1.0000930531941031e+06 x[22,16]
   - service_time[22] + start_service[16] - start_service[22] >= -999999
 NodeArrivalVehicles[22,17]: - 1.0001128471096832e+06 x[22,17]
   - service_time[22] + start_service[17] - start_service[22] >= -999999
 NodeArrivalVehicles[22,18]: - 1.0000733867898309e+06 x[22,18]
   - service_time[22] + start_service[18] - start_service[22] >= -999999
 NodeArrivalVehicles[22,19]: - 1.0001135742778145e+06 x[22,19]
   - service_time[22] + start_service[19] - start_service[22] >= -999999
 NodeArrivalVehicles[22,20]: - 1.0000665133082913e+06 x[22,20]
   - service_time[22] + start_service[20] - start_service[22] >= -999999
 NodeArrivalVehicles[22,21]: - 1.0000632444353482e+06 x[22,21]
   - service_time[22] + start_service[21] - start_service[22] >= -999999
 NodeArrivalVehicles[22,22]: - 999999 x[22,22] - service_time[22]
   >= -999999
 NodeArrivalVehicles[22,23]: - 999999 x[22,23] - service_time[22]
   - start_service[22] + start_service[23] >= -999999
 R574: w[0,5] = 2
 R575: w[0,2] = 2
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
 StartServiceRobots[0,2]: - 1.0000401805928103e+06 v[0,2] - service_time[0]
   - start_service[0] + start_service[2] >= -999999
 StartServiceRobots[0,3]: - 1.0000857578554453e+06 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999999
 StartServiceRobots[0,4]: - 1.0000147366179901e+06 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999999
 StartServiceRobots[0,5]: - 1.0000457716821638e+06 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999999
 StartServiceRobots[0,6]: - 1.0000930531941031e+06 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999999
 StartServiceRobots[0,7]: - 1.0001128471096832e+06 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999999
 StartServiceRobots[0,8]: - 1.0000733867898309e+06 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999999
 StartServiceRobots[0,9]: - 1.0001135742778145e+06 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999999
 StartServiceRobots[0,10]: - 1.0000665133082913e+06 v[0,10]
   - service_time[0] - start_service[0] + start_service[10] >= -999999
 StartServiceRobots[0,11]: - 1.0000632444353482e+06 v[0,11]
   - service_time[0] - start_service[0] + start_service[11] >= -999999
 StartServiceRobots[0,12]: - 1.0000401805928103e+06 v[0,12]
   - service_time[0] - start_service[0] + start_service[12] >= -999999
 StartServiceRobots[0,13]: - 1.0000857578554453e+06 v[0,13]
   - service_time[0] - start_service[0] + start_service[13] >= -999999
 StartServiceRobots[0,14]: - 1.0000147366179901e+06 v[0,14]
   - service_time[0] - start_service[0] + start_service[14] >= -999999
 StartServiceRobots[0,15]: - 1.0000457716821638e+06 v[0,15]
   - service_time[0] - start_service[0] + start_service[15] >= -999999
 StartServiceRobots[0,16]: - 1.0000930531941031e+06 v[0,16]
   - service_time[0] - start_service[0] + start_service[16] >= -999999
 StartServiceRobots[0,17]: - 1.0001128471096832e+06 v[0,17]
   - service_time[0] - start_service[0] + start_service[17] >= -999999
 StartServiceRobots[0,18]: - 1.0000733867898309e+06 v[0,18]
   - service_time[0] - start_service[0] + start_service[18] >= -999999
 StartServiceRobots[0,19]: - 1.0001135742778145e+06 v[0,19]
   - service_time[0] - start_service[0] + start_service[19] >= -999999
 StartServiceRobots[0,20]: - 1.0000665133082913e+06 v[0,20]
   - service_time[0] - start_service[0] + start_service[20] >= -999999
 StartServiceRobots[0,21]: - 1.0000632444353482e+06 v[0,21]
   - service_time[0] - start_service[0] + start_service[21] >= -999999
 StartServiceRobots[0,22]: - 999999 v[0,22] - service_time[0]
   - start_service[0] + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999 v[0,23] - service_time[0]
   - start_service[0] + start_service[23] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 1.0000401805928103e+06 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999999
 StartServiceRobots[1,3]: - 1.0000857578554453e+06 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999999
 StartServiceRobots[1,4]: - 1.0000147366179901e+06 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999999
 StartServiceRobots[1,5]: - 1.0000457716821638e+06 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999999
 StartServiceRobots[1,6]: - 1.0000930531941031e+06 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999999
 StartServiceRobots[1,7]: - 1.0001128471096832e+06 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999999
 StartServiceRobots[1,8]: - 1.0000733867898309e+06 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999999
 StartServiceRobots[1,9]: - 1.0001135742778145e+06 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999999
 StartServiceRobots[1,10]: - 1.0000665133082913e+06 v[1,10]
   - service_time[1] - start_service[1] + start_service[10] >= -999999
 StartServiceRobots[1,11]: - 1.0000632444353482e+06 v[1,11]
   - service_time[1] - start_service[1] + start_service[11] >= -999999
 StartServiceRobots[1,12]: - 1.0000401805928103e+06 v[1,12]
   - service_time[1] - start_service[1] + start_service[12] >= -999999
 StartServiceRobots[1,13]: - 1.0000857578554453e+06 v[1,13]
   - service_time[1] - start_service[1] + start_service[13] >= -999999
 StartServiceRobots[1,14]: - 1.0000147366179901e+06 v[1,14]
   - service_time[1] - start_service[1] + start_service[14] >= -999999
 StartServiceRobots[1,15]: - 1.0000457716821638e+06 v[1,15]
   - service_time[1] - start_service[1] + start_service[15] >= -999999
 StartServiceRobots[1,16]: - 1.0000930531941031e+06 v[1,16]
   - service_time[1] - start_service[1] + start_service[16] >= -999999
 StartServiceRobots[1,17]: - 1.0001128471096832e+06 v[1,17]
   - service_time[1] - start_service[1] + start_service[17] >= -999999
 StartServiceRobots[1,18]: - 1.0000733867898309e+06 v[1,18]
   - service_time[1] - start_service[1] + start_service[18] >= -999999
 StartServiceRobots[1,19]: - 1.0001135742778145e+06 v[1,19]
   - service_time[1] - start_service[1] + start_service[19] >= -999999
 StartServiceRobots[1,20]: - 1.0000665133082913e+06 v[1,20]
   - service_time[1] - start_service[1] + start_service[20] >= -999999
 StartServiceRobots[1,21]: - 1.0000632444353482e+06 v[1,21]
   - service_time[1] - start_service[1] + start_service[21] >= -999999
 StartServiceRobots[1,22]: - 999999 v[1,22] - service_time[1]
   - start_service[1] + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999 v[1,23] - service_time[1]
   - start_service[1] + start_service[23] >= -999999
 StartServiceRobots[2,1]: - 1.0000401805928103e+06 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999999
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 1.0000500807426213e+06 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999999
 StartServiceRobots[2,4]: - 1.000030792514505e+06 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999999
 StartServiceRobots[2,5]: - 1.0000071143237508e+06 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999999
 StartServiceRobots[2,6]: - 1.0000533055907747e+06 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999999
 StartServiceRobots[2,7]: - 1.0000778020430846e+06 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999999
 StartServiceRobots[2,8]: - 1.0000332531050219e+06 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999999
 StartServiceRobots[2,9]: - 1.0000784142247903e+06 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999999
 StartServiceRobots[2,10]: - 1.0000253782266368e+06 v[2,10]
   - service_time[2] - start_service[2] + start_service[10] >= -999999
 StartServiceRobots[2,11]: - 1.0000265613006055e+06 v[2,11]
   - service_time[2] - start_service[2] + start_service[11] >= -999999
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 1.0000500807426213e+06 v[2,13]
   - service_time[2] - start_service[2] + start_service[13] >= -999999
 StartServiceRobots[2,14]: - 1.000030792514505e+06 v[2,14]
   - service_time[2] - start_service[2] + start_service[14] >= -999999
 StartServiceRobots[2,15]: - 1.0000071143237508e+06 v[2,15]
   - service_time[2] - start_service[2] + start_service[15] >= -999999
 StartServiceRobots[2,16]: - 1.0000533055907747e+06 v[2,16]
   - service_time[2] - start_service[2] + start_service[16] >= -999999
 StartServiceRobots[2,17]: - 1.0000778020430846e+06 v[2,17]
   - service_time[2] - start_service[2] + start_service[17] >= -999999
 StartServiceRobots[2,18]: - 1.0000332531050219e+06 v[2,18]
   - service_time[2] - start_service[2] + start_service[18] >= -999999
 StartServiceRobots[2,19]: - 1.0000784142247903e+06 v[2,19]
   - service_time[2] - start_service[2] + start_service[19] >= -999999
 StartServiceRobots[2,20]: - 1.0000253782266368e+06 v[2,20]
   - service_time[2] - start_service[2] + start_service[20] >= -999999
 StartServiceRobots[2,21]: - 1.0000265613006055e+06 v[2,21]
   - service_time[2] - start_service[2] + start_service[21] >= -999999
 StartServiceRobots[2,22]: - 999999 v[2,22] - service_time[2]
   - start_service[2] + start_service[22] >= -999999
 StartServiceRobots[2,23]: - 999999 v[2,23] - service_time[2]
   - start_service[2] + start_service[23] >= -999999
 StartServiceRobots[3,1]: - 1.0000857578554453e+06 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999999
 StartServiceRobots[3,2]: - 1.0000500807426213e+06 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999999
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 1.0000713740287239e+06 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999999
 StartServiceRobots[3,5]: - 1.000042124286405e+06 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999999
 StartServiceRobots[3,6]: - 1.0000518389269916e+06 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999999
 StartServiceRobots[3,7]: - 1.0000267523298946e+06 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999999
 StartServiceRobots[3,8]: - 1.000021622481183e+06 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999999
 StartServiceRobots[3,9]: - 1.0000273896253681e+06 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999999
 StartServiceRobots[3,10]: - 1.0000361135195905e+06 v[3,10]
   - service_time[3] - start_service[3] + start_service[10] >= -999999
 StartServiceRobots[3,11]: - 1.0000225783656818e+06 v[3,11]
   - service_time[3] - start_service[3] + start_service[11] >= -999999
 StartServiceRobots[3,12]: - 1.0000500807426213e+06 v[3,12]
   - service_time[3] - start_service[3] + start_service[12] >= -999999
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 1.0000713740287239e+06 v[3,14]
   - service_time[3] - start_service[3] + start_service[14] >= -999999
 StartServiceRobots[3,15]: - 1.000042124286405e+06 v[3,15]
   - service_time[3] - start_service[3] + start_service[15] >= -999999
 StartServiceRobots[3,16]: - 1.0000518389269916e+06 v[3,16]
   - service_time[3] - start_service[3] + start_service[16] >= -999999
 StartServiceRobots[3,17]: - 1.0000267523298946e+06 v[3,17]
   - service_time[3] - start_service[3] + start_service[17] >= -999999
 StartServiceRobots[3,18]: - 1.000021622481183e+06 v[3,18]
   - service_time[3] - start_service[3] + start_service[18] >= -999999
 StartServiceRobots[3,19]: - 1.0000273896253681e+06 v[3,19]
   - service_time[3] - start_service[3] + start_service[19] >= -999999
 StartServiceRobots[3,20]: - 1.0000361135195905e+06 v[3,20]
   - service_time[3] - start_service[3] + start_service[20] >= -999999
 StartServiceRobots[3,21]: - 1.0000225783656818e+06 v[3,21]
   - service_time[3] - start_service[3] + start_service[21] >= -999999
 StartServiceRobots[3,22]: - 999999 v[3,22] - service_time[3]
   - start_service[3] + start_service[22] >= -999999
 StartServiceRobots[3,23]: - 999999 v[3,23] - service_time[3]
   - start_service[3] + start_service[23] >= -999999
 StartServiceRobots[4,1]: - 1.0000147366179901e+06 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999999
 StartServiceRobots[4,2]: - 1.000030792514505e+06 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999999
 StartServiceRobots[4,3]: - 1.0000713740287239e+06 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999999
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 1.0000343374844047e+06 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999999
 StartServiceRobots[4,6]: - 1.0000850981022327e+06 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999999
 StartServiceRobots[4,7]: - 1.0000980247615908e+06 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999999
 StartServiceRobots[4,8]: - 1.0000612054747479e+06 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999999
 StartServiceRobots[4,9]: - 1.0000987723651387e+06 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999999
 StartServiceRobots[4,10]: - 1.0000566241376635e+06 v[4,10]
   - service_time[4] - start_service[4] + start_service[10] >= -999999
 StartServiceRobots[4,11]: - 1.0000497453427232e+06 v[4,11]
   - service_time[4] - start_service[4] + start_service[11] >= -999999
 StartServiceRobots[4,12]: - 1.000030792514505e+06 v[4,12]
   - service_time[4] - start_service[4] + start_service[12] >= -999999
 StartServiceRobots[4,13]: - 1.0000713740287239e+06 v[4,13]
   - service_time[4] - start_service[4] + start_service[13] >= -999999
 StartServiceRobots[4,14]: - 999999 v[4,14] - service_time[4]
   - start_service[4] + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 1.0000343374844047e+06 v[4,15]
   - service_time[4] - start_service[4] + start_service[15] >= -999999
 StartServiceRobots[4,16]: - 1.0000850981022327e+06 v[4,16]
   - service_time[4] - start_service[4] + start_service[16] >= -999999
 StartServiceRobots[4,17]: - 1.0000980247615908e+06 v[4,17]
   - service_time[4] - start_service[4] + start_service[17] >= -999999
 StartServiceRobots[4,18]: - 1.0000612054747479e+06 v[4,18]
   - service_time[4] - start_service[4] + start_service[18] >= -999999
 StartServiceRobots[4,19]: - 1.0000987723651387e+06 v[4,19]
   - service_time[4] - start_service[4] + start_service[19] >= -999999
 StartServiceRobots[4,20]: - 1.0000566241376635e+06 v[4,20]
   - service_time[4] - start_service[4] + start_service[20] >= -999999
 StartServiceRobots[4,21]: - 1.0000497453427232e+06 v[4,21]
   - service_time[4] - start_service[4] + start_service[21] >= -999999
 StartServiceRobots[4,22]: - 999999 v[4,22] - service_time[4]
   - start_service[4] + start_service[22] >= -999999
 StartServiceRobots[4,23]: - 999999 v[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= -999999
 StartServiceRobots[5,1]: - 1.0000457716821638e+06 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999999
 StartServiceRobots[5,2]: - 1.0000071143237508e+06 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999999
 StartServiceRobots[5,3]: - 1.000042124286405e+06 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999999
 StartServiceRobots[5,4]: - 1.0000343374844047e+06 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999999
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 1.0000511580216251e+06 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999999
 StartServiceRobots[5,7]: - 1.0000698714195783e+06 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999999
 StartServiceRobots[5,8]: - 1.0000266352600046e+06 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999999
 StartServiceRobots[5,9]: - 1.0000704969536402e+06 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999999
 StartServiceRobots[5,10]: - 1.0000216805964843e+06 v[5,10]
   - service_time[5] - start_service[5] + start_service[10] >= -999999
 StartServiceRobots[5,11]: - 1.0000185543180438e+06 v[5,11]
   - service_time[5] - start_service[5] + start_service[11] >= -999999
 StartServiceRobots[5,12]: - 1.0000071143237508e+06 v[5,12]
   - service_time[5] - start_service[5] + start_service[12] >= -999999
 StartServiceRobots[5,13]: - 1.000042124286405e+06 v[5,13]
   - service_time[5] - start_service[5] + start_service[13] >= -999999
 StartServiceRobots[5,14]: - 1.0000343374844047e+06 v[5,14]
   - service_time[5] - start_service[5] + start_service[14] >= -999999
 StartServiceRobots[5,15]: - 999999 v[5,15] - service_time[5]
   - start_service[5] + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 1.0000511580216251e+06 v[5,16]
   - service_time[5] - start_service[5] + start_service[16] >= -999999
 StartServiceRobots[5,17]: - 1.0000698714195783e+06 v[5,17]
   - service_time[5] - start_service[5] + start_service[17] >= -999999
 StartServiceRobots[5,18]: - 1.0000266352600046e+06 v[5,18]
   - service_time[5] - start_service[5] + start_service[18] >= -999999
 StartServiceRobots[5,19]: - 1.0000704969536402e+06 v[5,19]
   - service_time[5] - start_service[5] + start_service[19] >= -999999
 StartServiceRobots[5,20]: - 1.0000216805964843e+06 v[5,20]
   - service_time[5] - start_service[5] + start_service[20] >= -999999
 StartServiceRobots[5,21]: - 1.0000185543180438e+06 v[5,21]
   - service_time[5] - start_service[5] + start_service[21] >= -999999
 StartServiceRobots[5,22]: - 999999 v[5,22] - service_time[5]
   - start_service[5] + start_service[22] >= -999999
 StartServiceRobots[5,23]: - 999999 v[5,23] - service_time[5]
   - start_service[5] + start_service[23] >= -999999
 StartServiceRobots[6,1]: - 1.0000930531941031e+06 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999999
 StartServiceRobots[6,2]: - 1.0000533055907747e+06 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999999
 StartServiceRobots[6,3]: - 1.0000518389269916e+06 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999999
 StartServiceRobots[6,4]: - 1.0000850981022327e+06 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999999
 StartServiceRobots[6,5]: - 1.0000511580216251e+06 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999999
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 1.0000678379998124e+06 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999999
 StartServiceRobots[6,8]: - 1.0000342428720902e+06 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999999
 StartServiceRobots[6,9]: - 1.0000678991589429e+06 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999999
 StartServiceRobots[6,10]: - 1.0000287929886794e+06 v[6,10]
   - service_time[6] - start_service[6] + start_service[10] >= -999999
 StartServiceRobots[6,11]: - 1.0000472983184841e+06 v[6,11]
   - service_time[6] - start_service[6] + start_service[11] >= -999999
 StartServiceRobots[6,12]: - 1.0000533055907747e+06 v[6,12]
   - service_time[6] - start_service[6] + start_service[12] >= -999999
 StartServiceRobots[6,13]: - 1.0000518389269916e+06 v[6,13]
   - service_time[6] - start_service[6] + start_service[13] >= -999999
 StartServiceRobots[6,14]: - 1.0000850981022327e+06 v[6,14]
   - service_time[6] - start_service[6] + start_service[14] >= -999999
 StartServiceRobots[6,15]: - 1.0000511580216251e+06 v[6,15]
   - service_time[6] - start_service[6] + start_service[15] >= -999999
 StartServiceRobots[6,16]: - 999999 v[6,16] - service_time[6]
   - start_service[6] + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 1.0000678379998124e+06 v[6,17]
   - service_time[6] - start_service[6] + start_service[17] >= -999999
 StartServiceRobots[6,18]: - 1.0000342428720902e+06 v[6,18]
   - service_time[6] - start_service[6] + start_service[18] >= -999999
 StartServiceRobots[6,19]: - 1.0000678991589429e+06 v[6,19]
   - service_time[6] - start_service[6] + start_service[19] >= -999999
 StartServiceRobots[6,20]: - 1.0000287929886794e+06 v[6,20]
   - service_time[6] - start_service[6] + start_service[20] >= -999999
 StartServiceRobots[6,21]: - 1.0000472983184841e+06 v[6,21]
   - service_time[6] - start_service[6] + start_service[21] >= -999999
 StartServiceRobots[6,22]: - 999999 v[6,22] - service_time[6]
   - start_service[6] + start_service[22] >= -999999
 StartServiceRobots[6,23]: - 999999 v[6,23] - service_time[6]
   - start_service[6] + start_service[23] >= -999999
 StartServiceRobots[7,1]: - 1.0001128471096832e+06 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999999
 StartServiceRobots[7,2]: - 1.0000778020430846e+06 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999999
 StartServiceRobots[7,3]: - 1.0000267523298946e+06 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999999
 StartServiceRobots[7,4]: - 1.0000980247615908e+06 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999999
 StartServiceRobots[7,5]: - 1.0000698714195783e+06 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999999
 StartServiceRobots[7,6]: - 1.0000678379998124e+06 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999999
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 1.0000469444748068e+06 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999999
 StartServiceRobots[7,9]: - 999999.8437675878 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999999
 StartServiceRobots[7,10]: - 1.00006122839988e+06 v[7,10] - service_time[7]
   - start_service[7] + start_service[10] >= -999999
 StartServiceRobots[7,11]: - 1.0000503303973848e+06 v[7,11]
   - service_time[7] - start_service[7] + start_service[11] >= -999999
 StartServiceRobots[7,12]: - 1.0000778020430846e+06 v[7,12]
   - service_time[7] - start_service[7] + start_service[12] >= -999999
 StartServiceRobots[7,13]: - 1.0000267523298946e+06 v[7,13]
   - service_time[7] - start_service[7] + start_service[13] >= -999999
 StartServiceRobots[7,14]: - 1.0000980247615908e+06 v[7,14]
   - service_time[7] - start_service[7] + start_service[14] >= -999999
 StartServiceRobots[7,15]: - 1.0000698714195783e+06 v[7,15]
   - service_time[7] - start_service[7] + start_service[15] >= -999999
 StartServiceRobots[7,16]: - 1.0000678379998124e+06 v[7,16]
   - service_time[7] - start_service[7] + start_service[16] >= -999999
 StartServiceRobots[7,17]: - 999999 v[7,17] - service_time[7]
   - start_service[7] + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 1.0000469444748068e+06 v[7,18]
   - service_time[7] - start_service[7] + start_service[18] >= -999999
 StartServiceRobots[7,19]: - 999999.8437675878 v[7,19] - service_time[7]
   - start_service[7] + start_service[19] >= -999999
 StartServiceRobots[7,20]: - 1.00006122839988e+06 v[7,20] - service_time[7]
   - start_service[7] + start_service[20] >= -999999
 StartServiceRobots[7,21]: - 1.0000503303973848e+06 v[7,21]
   - service_time[7] - start_service[7] + start_service[21] >= -999999
 StartServiceRobots[7,22]: - 999999 v[7,22] - service_time[7]
   - start_service[7] + start_service[22] >= -999999
 StartServiceRobots[7,23]: - 999999 v[7,23] - service_time[7]
   - start_service[7] + start_service[23] >= -999999
 StartServiceRobots[8,1]: - 1.0000733867898309e+06 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999999
 StartServiceRobots[8,2]: - 1.0000332531050219e+06 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999999
 StartServiceRobots[8,3]: - 1.000021622481183e+06 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999999
 StartServiceRobots[8,4]: - 1.0000612054747479e+06 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999999
 StartServiceRobots[8,5]: - 1.0000266352600046e+06 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999999
 StartServiceRobots[8,6]: - 1.0000342428720902e+06 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999999
 StartServiceRobots[8,7]: - 1.0000469444748068e+06 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999999
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 1.0000474019358882e+06 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999999
 StartServiceRobots[8,10]: - 1.0000135594043691e+06 v[8,10]
   - service_time[8] - start_service[8] + start_service[10] >= -999999
 StartServiceRobots[8,11]: - 1.0000131302091408e+06 v[8,11]
   - service_time[8] - start_service[8] + start_service[11] >= -999999
 StartServiceRobots[8,12]: - 1.0000332531050219e+06 v[8,12]
   - service_time[8] - start_service[8] + start_service[12] >= -999999
 StartServiceRobots[8,13]: - 1.000021622481183e+06 v[8,13]
   - service_time[8] - start_service[8] + start_service[13] >= -999999
 StartServiceRobots[8,14]: - 1.0000612054747479e+06 v[8,14]
   - service_time[8] - start_service[8] + start_service[14] >= -999999
 StartServiceRobots[8,15]: - 1.0000266352600046e+06 v[8,15]
   - service_time[8] - start_service[8] + start_service[15] >= -999999
 StartServiceRobots[8,16]: - 1.0000342428720902e+06 v[8,16]
   - service_time[8] - start_service[8] + start_service[16] >= -999999
 StartServiceRobots[8,17]: - 1.0000469444748068e+06 v[8,17]
   - service_time[8] - start_service[8] + start_service[17] >= -999999
 StartServiceRobots[8,18]: - 999999 v[8,18] - service_time[8]
   - start_service[8] + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 1.0000474019358882e+06 v[8,19]
   - service_time[8] - start_service[8] + start_service[19] >= -999999
 StartServiceRobots[8,20]: - 1.0000135594043691e+06 v[8,20]
   - service_time[8] - start_service[8] + start_service[20] >= -999999
 StartServiceRobots[8,21]: - 1.0000131302091408e+06 v[8,21]
   - service_time[8] - start_service[8] + start_service[21] >= -999999
 StartServiceRobots[8,22]: - 999999 v[8,22] - service_time[8]
   - start_service[8] + start_service[22] >= -999999
 StartServiceRobots[8,23]: - 999999 v[8,23] - service_time[8]
   - start_service[8] + start_service[23] >= -999999
 StartServiceRobots[9,1]: - 1.0001135742778145e+06 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999999
 StartServiceRobots[9,2]: - 1.0000784142247903e+06 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999999
 StartServiceRobots[9,3]: - 1.0000273896253681e+06 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999999
 StartServiceRobots[9,4]: - 1.0000987723651387e+06 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999999
 StartServiceRobots[9,5]: - 1.0000704969536402e+06 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999999
 StartServiceRobots[9,6]: - 1.0000678991589429e+06 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999999
 StartServiceRobots[9,7]: - 999999.8437675878 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999999
 StartServiceRobots[9,8]: - 1.0000474019358882e+06 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999999
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 1.0000616475755909e+06 v[9,10]
   - service_time[9] - start_service[9] + start_service[10] >= -999999
 StartServiceRobots[9,11]: - 1.0000509634543548e+06 v[9,11]
   - service_time[9] - start_service[9] + start_service[11] >= -999999
 StartServiceRobots[9,12]: - 1.0000784142247903e+06 v[9,12]
   - service_time[9] - start_service[9] + start_service[12] >= -999999
 StartServiceRobots[9,13]: - 1.0000273896253681e+06 v[9,13]
   - service_time[9] - start_service[9] + start_service[13] >= -999999
 StartServiceRobots[9,14]: - 1.0000987723651387e+06 v[9,14]
   - service_time[9] - start_service[9] + start_service[14] >= -999999
 StartServiceRobots[9,15]: - 1.0000704969536402e+06 v[9,15]
   - service_time[9] - start_service[9] + start_service[15] >= -999999
 StartServiceRobots[9,16]: - 1.0000678991589429e+06 v[9,16]
   - service_time[9] - start_service[9] + start_service[16] >= -999999
 StartServiceRobots[9,17]: - 999999.8437675878 v[9,17] - service_time[9]
   - start_service[9] + start_service[17] >= -999999
 StartServiceRobots[9,18]: - 1.0000474019358882e+06 v[9,18]
   - service_time[9] - start_service[9] + start_service[18] >= -999999
 StartServiceRobots[9,19]: - 999999 v[9,19] - service_time[9]
   - start_service[9] + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 1.0000616475755909e+06 v[9,20]
   - service_time[9] - start_service[9] + start_service[20] >= -999999
 StartServiceRobots[9,21]: - 1.0000509634543548e+06 v[9,21]
   - service_time[9] - start_service[9] + start_service[21] >= -999999
 StartServiceRobots[9,22]: - 999999 v[9,22] - service_time[9]
   - start_service[9] + start_service[22] >= -999999
 StartServiceRobots[9,23]: - 999999 v[9,23] - service_time[9]
   - start_service[9] + start_service[23] >= -999999
 StartServiceRobots[10,1]: - 1.0000665133082913e+06 v[10,1]
   - service_time[10] + start_service[1] - start_service[10] >= -999999
 StartServiceRobots[10,2]: - 1.0000253782266368e+06 v[10,2]
   - service_time[10] + start_service[2] - start_service[10] >= -999999
 StartServiceRobots[10,3]: - 1.0000361135195905e+06 v[10,3]
   - service_time[10] + start_service[3] - start_service[10] >= -999999
 StartServiceRobots[10,4]: - 1.0000566241376635e+06 v[10,4]
   - service_time[10] + start_service[4] - start_service[10] >= -999999
 StartServiceRobots[10,5]: - 1.0000216805964843e+06 v[10,5]
   - service_time[10] + start_service[5] - start_service[10] >= -999999
 StartServiceRobots[10,6]: - 1.0000287929886794e+06 v[10,6]
   - service_time[10] + start_service[6] - start_service[10] >= -999999
 StartServiceRobots[10,7]: - 1.00006122839988e+06 v[10,7]
   - service_time[10] + start_service[7] - start_service[10] >= -999999
 StartServiceRobots[10,8]: - 1.0000135594043691e+06 v[10,8]
   - service_time[10] + start_service[8] - start_service[10] >= -999999
 StartServiceRobots[10,9]: - 1.0000616475755909e+06 v[10,9]
   - service_time[10] + start_service[9] - start_service[10] >= -999999
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 1.0000204820647413e+06 v[10,11]
   - service_time[10] - start_service[10] + start_service[11] >= -999999
 StartServiceRobots[10,12]: - 1.0000253782266368e+06 v[10,12]
   - service_time[10] - start_service[10] + start_service[12] >= -999999
 StartServiceRobots[10,13]: - 1.0000361135195905e+06 v[10,13]
   - service_time[10] - start_service[10] + start_service[13] >= -999999
 StartServiceRobots[10,14]: - 1.0000566241376635e+06 v[10,14]
   - service_time[10] - start_service[10] + start_service[14] >= -999999
 StartServiceRobots[10,15]: - 1.0000216805964843e+06 v[10,15]
   - service_time[10] - start_service[10] + start_service[15] >= -999999
 StartServiceRobots[10,16]: - 1.0000287929886794e+06 v[10,16]
   - service_time[10] - start_service[10] + start_service[16] >= -999999
 StartServiceRobots[10,17]: - 1.00006122839988e+06 v[10,17]
   - service_time[10] - start_service[10] + start_service[17] >= -999999
 StartServiceRobots[10,18]: - 1.0000135594043691e+06 v[10,18]
   - service_time[10] - start_service[10] + start_service[18] >= -999999
 StartServiceRobots[10,19]: - 1.0000616475755909e+06 v[10,19]
   - service_time[10] - start_service[10] + start_service[19] >= -999999
 StartServiceRobots[10,20]: - 999999 v[10,20] - service_time[10]
   - start_service[10] + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 1.0000204820647413e+06 v[10,21]
   - service_time[10] - start_service[10] + start_service[21] >= -999999
 StartServiceRobots[10,22]: - 999999 v[10,22] - service_time[10]
   - start_service[10] + start_service[22] >= -999999
 StartServiceRobots[10,23]: - 999999 v[10,23] - service_time[10]
   - start_service[10] + start_service[23] >= -999999
 StartServiceRobots[11,1]: - 1.0000632444353482e+06 v[11,1]
   - service_time[11] + start_service[1] - start_service[11] >= -999999
 StartServiceRobots[11,2]: - 1.0000265613006055e+06 v[11,2]
   - service_time[11] + start_service[2] - start_service[11] >= -999999
 StartServiceRobots[11,3]: - 1.0000225783656818e+06 v[11,3]
   - service_time[11] + start_service[3] - start_service[11] >= -999999
 StartServiceRobots[11,4]: - 1.0000497453427232e+06 v[11,4]
   - service_time[11] + start_service[4] - start_service[11] >= -999999
 StartServiceRobots[11,5]: - 1.0000185543180438e+06 v[11,5]
   - service_time[11] + start_service[5] - start_service[11] >= -999999
 StartServiceRobots[11,6]: - 1.0000472983184841e+06 v[11,6]
   - service_time[11] + start_service[6] - start_service[11] >= -999999
 StartServiceRobots[11,7]: - 1.0000503303973848e+06 v[11,7]
   - service_time[11] + start_service[7] - start_service[11] >= -999999
 StartServiceRobots[11,8]: - 1.0000131302091408e+06 v[11,8]
   - service_time[11] + start_service[8] - start_service[11] >= -999999
 StartServiceRobots[11,9]: - 1.0000509634543548e+06 v[11,9]
   - service_time[11] + start_service[9] - start_service[11] >= -999999
 StartServiceRobots[11,10]: - 1.0000204820647413e+06 v[11,10]
   - service_time[11] + start_service[10] - start_service[11] >= -999999
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 1.0000265613006055e+06 v[11,12]
   - service_time[11] - start_service[11] + start_service[12] >= -999999
 StartServiceRobots[11,13]: - 1.0000225783656818e+06 v[11,13]
   - service_time[11] - start_service[11] + start_service[13] >= -999999
 StartServiceRobots[11,14]: - 1.0000497453427232e+06 v[11,14]
   - service_time[11] - start_service[11] + start_service[14] >= -999999
 StartServiceRobots[11,15]: - 1.0000185543180438e+06 v[11,15]
   - service_time[11] - start_service[11] + start_service[15] >= -999999
 StartServiceRobots[11,16]: - 1.0000472983184841e+06 v[11,16]
   - service_time[11] - start_service[11] + start_service[16] >= -999999
 StartServiceRobots[11,17]: - 1.0000503303973848e+06 v[11,17]
   - service_time[11] - start_service[11] + start_service[17] >= -999999
 StartServiceRobots[11,18]: - 1.0000131302091408e+06 v[11,18]
   - service_time[11] - start_service[11] + start_service[18] >= -999999
 StartServiceRobots[11,19]: - 1.0000509634543548e+06 v[11,19]
   - service_time[11] - start_service[11] + start_service[19] >= -999999
 StartServiceRobots[11,20]: - 1.0000204820647413e+06 v[11,20]
   - service_time[11] - start_service[11] + start_service[20] >= -999999
 StartServiceRobots[11,21]: - 999999 v[11,21] - service_time[11]
   - start_service[11] + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 999999 v[11,22] - service_time[11]
   - start_service[11] + start_service[22] >= -999999
 StartServiceRobots[11,23]: - 999999 v[11,23] - service_time[11]
   - start_service[11] + start_service[23] >= -999999
 StartServiceRobots[12,1]: - 1.0000401805928103e+06 v[12,1]
   - service_time[12] + start_service[1] - start_service[12] >= -999999
 StartServiceRobots[12,2]: - 999999 v[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 1.0000500807426213e+06 v[12,3]
   - service_time[12] + start_service[3] - start_service[12] >= -999999
 StartServiceRobots[12,4]: - 1.000030792514505e+06 v[12,4]
   - service_time[12] + start_service[4] - start_service[12] >= -999999
 StartServiceRobots[12,5]: - 1.0000071143237508e+06 v[12,5]
   - service_time[12] + start_service[5] - start_service[12] >= -999999
 StartServiceRobots[12,6]: - 1.0000533055907747e+06 v[12,6]
   - service_time[12] + start_service[6] - start_service[12] >= -999999
 StartServiceRobots[12,7]: - 1.0000778020430846e+06 v[12,7]
   - service_time[12] + start_service[7] - start_service[12] >= -999999
 StartServiceRobots[12,8]: - 1.0000332531050219e+06 v[12,8]
   - service_time[12] + start_service[8] - start_service[12] >= -999999
 StartServiceRobots[12,9]: - 1.0000784142247903e+06 v[12,9]
   - service_time[12] + start_service[9] - start_service[12] >= -999999
 StartServiceRobots[12,10]: - 1.0000253782266368e+06 v[12,10]
   - service_time[12] + start_service[10] - start_service[12] >= -999999
 StartServiceRobots[12,11]: - 1.0000265613006055e+06 v[12,11]
   - service_time[12] + start_service[11] - start_service[12] >= -999999
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 1.0000500807426213e+06 v[12,13]
   - service_time[12] - start_service[12] + start_service[13] >= -999999
 StartServiceRobots[12,14]: - 1.000030792514505e+06 v[12,14]
   - service_time[12] - start_service[12] + start_service[14] >= -999999
 StartServiceRobots[12,15]: - 1.0000071143237508e+06 v[12,15]
   - service_time[12] - start_service[12] + start_service[15] >= -999999
 StartServiceRobots[12,16]: - 1.0000533055907747e+06 v[12,16]
   - service_time[12] - start_service[12] + start_service[16] >= -999999
 StartServiceRobots[12,17]: - 1.0000778020430846e+06 v[12,17]
   - service_time[12] - start_service[12] + start_service[17] >= -999999
 StartServiceRobots[12,18]: - 1.0000332531050219e+06 v[12,18]
   - service_time[12] - start_service[12] + start_service[18] >= -999999
 StartServiceRobots[12,19]: - 1.0000784142247903e+06 v[12,19]
   - service_time[12] - start_service[12] + start_service[19] >= -999999
 StartServiceRobots[12,20]: - 1.0000253782266368e+06 v[12,20]
   - service_time[12] - start_service[12] + start_service[20] >= -999999
 StartServiceRobots[12,21]: - 1.0000265613006055e+06 v[12,21]
   - service_time[12] - start_service[12] + start_service[21] >= -999999
 StartServiceRobots[12,22]: - 999999 v[12,22] - service_time[12]
   - start_service[12] + start_service[22] >= -999999
 StartServiceRobots[12,23]: - 999999 v[12,23] - service_time[12]
   - start_service[12] + start_service[23] >= -999999
 StartServiceRobots[13,1]: - 1.0000857578554453e+06 v[13,1]
   - service_time[13] + start_service[1] - start_service[13] >= -999999
 StartServiceRobots[13,2]: - 1.0000500807426213e+06 v[13,2]
   - service_time[13] + start_service[2] - start_service[13] >= -999999
 StartServiceRobots[13,3]: - 999999 v[13,3] - service_time[13]
   + start_service[3] - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 1.0000713740287239e+06 v[13,4]
   - service_time[13] + start_service[4] - start_service[13] >= -999999
 StartServiceRobots[13,5]: - 1.000042124286405e+06 v[13,5]
   - service_time[13] + start_service[5] - start_service[13] >= -999999
 StartServiceRobots[13,6]: - 1.0000518389269916e+06 v[13,6]
   - service_time[13] + start_service[6] - start_service[13] >= -999999
 StartServiceRobots[13,7]: - 1.0000267523298946e+06 v[13,7]
   - service_time[13] + start_service[7] - start_service[13] >= -999999
 StartServiceRobots[13,8]: - 1.000021622481183e+06 v[13,8]
   - service_time[13] + start_service[8] - start_service[13] >= -999999
 StartServiceRobots[13,9]: - 1.0000273896253681e+06 v[13,9]
   - service_time[13] + start_service[9] - start_service[13] >= -999999
 StartServiceRobots[13,10]: - 1.0000361135195905e+06 v[13,10]
   - service_time[13] + start_service[10] - start_service[13] >= -999999
 StartServiceRobots[13,11]: - 1.0000225783656818e+06 v[13,11]
   - service_time[13] + start_service[11] - start_service[13] >= -999999
 StartServiceRobots[13,12]: - 1.0000500807426213e+06 v[13,12]
   - service_time[13] + start_service[12] - start_service[13] >= -999999
 StartServiceRobots[13,13]: - 999999 v[13,13] - service_time[13]
   >= -999999
 StartServiceRobots[13,14]: - 1.0000713740287239e+06 v[13,14]
   - service_time[13] - start_service[13] + start_service[14] >= -999999
 StartServiceRobots[13,15]: - 1.000042124286405e+06 v[13,15]
   - service_time[13] - start_service[13] + start_service[15] >= -999999
 StartServiceRobots[13,16]: - 1.0000518389269916e+06 v[13,16]
   - service_time[13] - start_service[13] + start_service[16] >= -999999
 StartServiceRobots[13,17]: - 1.0000267523298946e+06 v[13,17]
   - service_time[13] - start_service[13] + start_service[17] >= -999999
 StartServiceRobots[13,18]: - 1.000021622481183e+06 v[13,18]
   - service_time[13] - start_service[13] + start_service[18] >= -999999
 StartServiceRobots[13,19]: - 1.0000273896253681e+06 v[13,19]
   - service_time[13] - start_service[13] + start_service[19] >= -999999
 StartServiceRobots[13,20]: - 1.0000361135195905e+06 v[13,20]
   - service_time[13] - start_service[13] + start_service[20] >= -999999
 StartServiceRobots[13,21]: - 1.0000225783656818e+06 v[13,21]
   - service_time[13] - start_service[13] + start_service[21] >= -999999
 StartServiceRobots[13,22]: - 999999 v[13,22] - service_time[13]
   - start_service[13] + start_service[22] >= -999999
 StartServiceRobots[13,23]: - 999999 v[13,23] - service_time[13]
   - start_service[13] + start_service[23] >= -999999
 StartServiceRobots[14,1]: - 1.0000147366179901e+06 v[14,1]
   - service_time[14] + start_service[1] - start_service[14] >= -999999
 StartServiceRobots[14,2]: - 1.000030792514505e+06 v[14,2]
   - service_time[14] + start_service[2] - start_service[14] >= -999999
 StartServiceRobots[14,3]: - 1.0000713740287239e+06 v[14,3]
   - service_time[14] + start_service[3] - start_service[14] >= -999999
 StartServiceRobots[14,4]: - 999999 v[14,4] - service_time[14]
   + start_service[4] - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 1.0000343374844047e+06 v[14,5]
   - service_time[14] + start_service[5] - start_service[14] >= -999999
 StartServiceRobots[14,6]: - 1.0000850981022327e+06 v[14,6]
   - service_time[14] + start_service[6] - start_service[14] >= -999999
 StartServiceRobots[14,7]: - 1.0000980247615908e+06 v[14,7]
   - service_time[14] + start_service[7] - start_service[14] >= -999999
 StartServiceRobots[14,8]: - 1.0000612054747479e+06 v[14,8]
   - service_time[14] + start_service[8] - start_service[14] >= -999999
 StartServiceRobots[14,9]: - 1.0000987723651387e+06 v[14,9]
   - service_time[14] + start_service[9] - start_service[14] >= -999999
 StartServiceRobots[14,10]: - 1.0000566241376635e+06 v[14,10]
   - service_time[14] + start_service[10] - start_service[14] >= -999999
 StartServiceRobots[14,11]: - 1.0000497453427232e+06 v[14,11]
   - service_time[14] + start_service[11] - start_service[14] >= -999999
 StartServiceRobots[14,12]: - 1.000030792514505e+06 v[14,12]
   - service_time[14] + start_service[12] - start_service[14] >= -999999
 StartServiceRobots[14,13]: - 1.0000713740287239e+06 v[14,13]
   - service_time[14] + start_service[13] - start_service[14] >= -999999
 StartServiceRobots[14,14]: - 999999 v[14,14] - service_time[14]
   >= -999999
 StartServiceRobots[14,15]: - 1.0000343374844047e+06 v[14,15]
   - service_time[14] - start_service[14] + start_service[15] >= -999999
 StartServiceRobots[14,16]: - 1.0000850981022327e+06 v[14,16]
   - service_time[14] - start_service[14] + start_service[16] >= -999999
 StartServiceRobots[14,17]: - 1.0000980247615908e+06 v[14,17]
   - service_time[14] - start_service[14] + start_service[17] >= -999999
 StartServiceRobots[14,18]: - 1.0000612054747479e+06 v[14,18]
   - service_time[14] - start_service[14] + start_service[18] >= -999999
 StartServiceRobots[14,19]: - 1.0000987723651387e+06 v[14,19]
   - service_time[14] - start_service[14] + start_service[19] >= -999999
 StartServiceRobots[14,20]: - 1.0000566241376635e+06 v[14,20]
   - service_time[14] - start_service[14] + start_service[20] >= -999999
 StartServiceRobots[14,21]: - 1.0000497453427232e+06 v[14,21]
   - service_time[14] - start_service[14] + start_service[21] >= -999999
 StartServiceRobots[14,22]: - 999999 v[14,22] - service_time[14]
   - start_service[14] + start_service[22] >= -999999
 StartServiceRobots[14,23]: - 999999 v[14,23] - service_time[14]
   - start_service[14] + start_service[23] >= -999999
 StartServiceRobots[15,1]: - 1.0000457716821638e+06 v[15,1]
   - service_time[15] + start_service[1] - start_service[15] >= -999999
 StartServiceRobots[15,2]: - 1.0000071143237508e+06 v[15,2]
   - service_time[15] + start_service[2] - start_service[15] >= -999999
 StartServiceRobots[15,3]: - 1.000042124286405e+06 v[15,3]
   - service_time[15] + start_service[3] - start_service[15] >= -999999
 StartServiceRobots[15,4]: - 1.0000343374844047e+06 v[15,4]
   - service_time[15] + start_service[4] - start_service[15] >= -999999
 StartServiceRobots[15,5]: - 999999 v[15,5] - service_time[15]
   + start_service[5] - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 1.0000511580216251e+06 v[15,6]
   - service_time[15] + start_service[6] - start_service[15] >= -999999
 StartServiceRobots[15,7]: - 1.0000698714195783e+06 v[15,7]
   - service_time[15] + start_service[7] - start_service[15] >= -999999
 StartServiceRobots[15,8]: - 1.0000266352600046e+06 v[15,8]
   - service_time[15] + start_service[8] - start_service[15] >= -999999
 StartServiceRobots[15,9]: - 1.0000704969536402e+06 v[15,9]
   - service_time[15] + start_service[9] - start_service[15] >= -999999
 StartServiceRobots[15,10]: - 1.0000216805964843e+06 v[15,10]
   - service_time[15] + start_service[10] - start_service[15] >= -999999
 StartServiceRobots[15,11]: - 1.0000185543180438e+06 v[15,11]
   - service_time[15] + start_service[11] - start_service[15] >= -999999
 StartServiceRobots[15,12]: - 1.0000071143237508e+06 v[15,12]
   - service_time[15] + start_service[12] - start_service[15] >= -999999
 StartServiceRobots[15,13]: - 1.000042124286405e+06 v[15,13]
   - service_time[15] + start_service[13] - start_service[15] >= -999999
 StartServiceRobots[15,14]: - 1.0000343374844047e+06 v[15,14]
   - service_time[15] + start_service[14] - start_service[15] >= -999999
 StartServiceRobots[15,15]: - 999999 v[15,15] - service_time[15]
   >= -999999
 StartServiceRobots[15,16]: - 1.0000511580216251e+06 v[15,16]
   - service_time[15] - start_service[15] + start_service[16] >= -999999
 StartServiceRobots[15,17]: - 1.0000698714195783e+06 v[15,17]
   - service_time[15] - start_service[15] + start_service[17] >= -999999
 StartServiceRobots[15,18]: - 1.0000266352600046e+06 v[15,18]
   - service_time[15] - start_service[15] + start_service[18] >= -999999
 StartServiceRobots[15,19]: - 1.0000704969536402e+06 v[15,19]
   - service_time[15] - start_service[15] + start_service[19] >= -999999
 StartServiceRobots[15,20]: - 1.0000216805964843e+06 v[15,20]
   - service_time[15] - start_service[15] + start_service[20] >= -999999
 StartServiceRobots[15,21]: - 1.0000185543180438e+06 v[15,21]
   - service_time[15] - start_service[15] + start_service[21] >= -999999
 StartServiceRobots[15,22]: - 999999 v[15,22] - service_time[15]
   - start_service[15] + start_service[22] >= -999999
 StartServiceRobots[15,23]: - 999999 v[15,23] - service_time[15]
   - start_service[15] + start_service[23] >= -999999
 StartServiceRobots[16,1]: - 1.0000930531941031e+06 v[16,1]
   - service_time[16] + start_service[1] - start_service[16] >= -999999
 StartServiceRobots[16,2]: - 1.0000533055907747e+06 v[16,2]
   - service_time[16] + start_service[2] - start_service[16] >= -999999
 StartServiceRobots[16,3]: - 1.0000518389269916e+06 v[16,3]
   - service_time[16] + start_service[3] - start_service[16] >= -999999
 StartServiceRobots[16,4]: - 1.0000850981022327e+06 v[16,4]
   - service_time[16] + start_service[4] - start_service[16] >= -999999
 StartServiceRobots[16,5]: - 1.0000511580216251e+06 v[16,5]
   - service_time[16] + start_service[5] - start_service[16] >= -999999
 StartServiceRobots[16,6]: - 999999 v[16,6] - service_time[16]
   + start_service[6] - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 1.0000678379998124e+06 v[16,7]
   - service_time[16] + start_service[7] - start_service[16] >= -999999
 StartServiceRobots[16,8]: - 1.0000342428720902e+06 v[16,8]
   - service_time[16] + start_service[8] - start_service[16] >= -999999
 StartServiceRobots[16,9]: - 1.0000678991589429e+06 v[16,9]
   - service_time[16] + start_service[9] - start_service[16] >= -999999
 StartServiceRobots[16,10]: - 1.0000287929886794e+06 v[16,10]
   - service_time[16] + start_service[10] - start_service[16] >= -999999
 StartServiceRobots[16,11]: - 1.0000472983184841e+06 v[16,11]
   - service_time[16] + start_service[11] - start_service[16] >= -999999
 StartServiceRobots[16,12]: - 1.0000533055907747e+06 v[16,12]
   - service_time[16] + start_service[12] - start_service[16] >= -999999
 StartServiceRobots[16,13]: - 1.0000518389269916e+06 v[16,13]
   - service_time[16] + start_service[13] - start_service[16] >= -999999
 StartServiceRobots[16,14]: - 1.0000850981022327e+06 v[16,14]
   - service_time[16] + start_service[14] - start_service[16] >= -999999
 StartServiceRobots[16,15]: - 1.0000511580216251e+06 v[16,15]
   - service_time[16] + start_service[15] - start_service[16] >= -999999
 StartServiceRobots[16,16]: - 999999 v[16,16] - service_time[16]
   >= -999999
 StartServiceRobots[16,17]: - 1.0000678379998124e+06 v[16,17]
   - service_time[16] - start_service[16] + start_service[17] >= -999999
 StartServiceRobots[16,18]: - 1.0000342428720902e+06 v[16,18]
   - service_time[16] - start_service[16] + start_service[18] >= -999999
 StartServiceRobots[16,19]: - 1.0000678991589429e+06 v[16,19]
   - service_time[16] - start_service[16] + start_service[19] >= -999999
 StartServiceRobots[16,20]: - 1.0000287929886794e+06 v[16,20]
   - service_time[16] - start_service[16] + start_service[20] >= -999999
 StartServiceRobots[16,21]: - 1.0000472983184841e+06 v[16,21]
   - service_time[16] - start_service[16] + start_service[21] >= -999999
 StartServiceRobots[16,22]: - 999999 v[16,22] - service_time[16]
   - start_service[16] + start_service[22] >= -999999
 StartServiceRobots[16,23]: - 999999 v[16,23] - service_time[16]
   - start_service[16] + start_service[23] >= -999999
 StartServiceRobots[17,1]: - 1.0001128471096832e+06 v[17,1]
   - service_time[17] + start_service[1] - start_service[17] >= -999999
 StartServiceRobots[17,2]: - 1.0000778020430846e+06 v[17,2]
   - service_time[17] + start_service[2] - start_service[17] >= -999999
 StartServiceRobots[17,3]: - 1.0000267523298946e+06 v[17,3]
   - service_time[17] + start_service[3] - start_service[17] >= -999999
 StartServiceRobots[17,4]: - 1.0000980247615908e+06 v[17,4]
   - service_time[17] + start_service[4] - start_service[17] >= -999999
 StartServiceRobots[17,5]: - 1.0000698714195783e+06 v[17,5]
   - service_time[17] + start_service[5] - start_service[17] >= -999999
 StartServiceRobots[17,6]: - 1.0000678379998124e+06 v[17,6]
   - service_time[17] + start_service[6] - start_service[17] >= -999999
 StartServiceRobots[17,7]: - 999999 v[17,7] - service_time[17]
   + start_service[7] - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 1.0000469444748068e+06 v[17,8]
   - service_time[17] + start_service[8] - start_service[17] >= -999999
 StartServiceRobots[17,9]: - 999999.8437675878 v[17,9] - service_time[17]
   + start_service[9] - start_service[17] >= -999999
 StartServiceRobots[17,10]: - 1.00006122839988e+06 v[17,10]
   - service_time[17] + start_service[10] - start_service[17] >= -999999
 StartServiceRobots[17,11]: - 1.0000503303973848e+06 v[17,11]
   - service_time[17] + start_service[11] - start_service[17] >= -999999
 StartServiceRobots[17,12]: - 1.0000778020430846e+06 v[17,12]
   - service_time[17] + start_service[12] - start_service[17] >= -999999
 StartServiceRobots[17,13]: - 1.0000267523298946e+06 v[17,13]
   - service_time[17] + start_service[13] - start_service[17] >= -999999
 StartServiceRobots[17,14]: - 1.0000980247615908e+06 v[17,14]
   - service_time[17] + start_service[14] - start_service[17] >= -999999
 StartServiceRobots[17,15]: - 1.0000698714195783e+06 v[17,15]
   - service_time[17] + start_service[15] - start_service[17] >= -999999
 StartServiceRobots[17,16]: - 1.0000678379998124e+06 v[17,16]
   - service_time[17] + start_service[16] - start_service[17] >= -999999
 StartServiceRobots[17,17]: - 999999 v[17,17] - service_time[17]
   >= -999999
 StartServiceRobots[17,18]: - 1.0000469444748068e+06 v[17,18]
   - service_time[17] - start_service[17] + start_service[18] >= -999999
 StartServiceRobots[17,19]: - 999999.8437675878 v[17,19] - service_time[17]
   - start_service[17] + start_service[19] >= -999999
 StartServiceRobots[17,20]: - 1.00006122839988e+06 v[17,20]
   - service_time[17] - start_service[17] + start_service[20] >= -999999
 StartServiceRobots[17,21]: - 1.0000503303973848e+06 v[17,21]
   - service_time[17] - start_service[17] + start_service[21] >= -999999
 StartServiceRobots[17,22]: - 999999 v[17,22] - service_time[17]
   - start_service[17] + start_service[22] >= -999999
 StartServiceRobots[17,23]: - 999999 v[17,23] - service_time[17]
   - start_service[17] + start_service[23] >= -999999
 StartServiceRobots[18,1]: - 1.0000733867898309e+06 v[18,1]
   - service_time[18] + start_service[1] - start_service[18] >= -999999
 StartServiceRobots[18,2]: - 1.0000332531050219e+06 v[18,2]
   - service_time[18] + start_service[2] - start_service[18] >= -999999
 StartServiceRobots[18,3]: - 1.000021622481183e+06 v[18,3]
   - service_time[18] + start_service[3] - start_service[18] >= -999999
 StartServiceRobots[18,4]: - 1.0000612054747479e+06 v[18,4]
   - service_time[18] + start_service[4] - start_service[18] >= -999999
 StartServiceRobots[18,5]: - 1.0000266352600046e+06 v[18,5]
   - service_time[18] + start_service[5] - start_service[18] >= -999999
 StartServiceRobots[18,6]: - 1.0000342428720902e+06 v[18,6]
   - service_time[18] + start_service[6] - start_service[18] >= -999999
 StartServiceRobots[18,7]: - 1.0000469444748068e+06 v[18,7]
   - service_time[18] + start_service[7] - start_service[18] >= -999999
 StartServiceRobots[18,8]: - 999999 v[18,8] - service_time[18]
   + start_service[8] - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 1.0000474019358882e+06 v[18,9]
   - service_time[18] + start_service[9] - start_service[18] >= -999999
 StartServiceRobots[18,10]: - 1.0000135594043691e+06 v[18,10]
   - service_time[18] + start_service[10] - start_service[18] >= -999999
 StartServiceRobots[18,11]: - 1.0000131302091408e+06 v[18,11]
   - service_time[18] + start_service[11] - start_service[18] >= -999999
 StartServiceRobots[18,12]: - 1.0000332531050219e+06 v[18,12]
   - service_time[18] + start_service[12] - start_service[18] >= -999999
 StartServiceRobots[18,13]: - 1.000021622481183e+06 v[18,13]
   - service_time[18] + start_service[13] - start_service[18] >= -999999
 StartServiceRobots[18,14]: - 1.0000612054747479e+06 v[18,14]
   - service_time[18] + start_service[14] - start_service[18] >= -999999
 StartServiceRobots[18,15]: - 1.0000266352600046e+06 v[18,15]
   - service_time[18] + start_service[15] - start_service[18] >= -999999
 StartServiceRobots[18,16]: - 1.0000342428720902e+06 v[18,16]
   - service_time[18] + start_service[16] - start_service[18] >= -999999
 StartServiceRobots[18,17]: - 1.0000469444748068e+06 v[18,17]
   - service_time[18] + start_service[17] - start_service[18] >= -999999
 StartServiceRobots[18,18]: - 999999 v[18,18] - service_time[18]
   >= -999999
 StartServiceRobots[18,19]: - 1.0000474019358882e+06 v[18,19]
   - service_time[18] - start_service[18] + start_service[19] >= -999999
 StartServiceRobots[18,20]: - 1.0000135594043691e+06 v[18,20]
   - service_time[18] - start_service[18] + start_service[20] >= -999999
 StartServiceRobots[18,21]: - 1.0000131302091408e+06 v[18,21]
   - service_time[18] - start_service[18] + start_service[21] >= -999999
 StartServiceRobots[18,22]: - 999999 v[18,22] - service_time[18]
   - start_service[18] + start_service[22] >= -999999
 StartServiceRobots[18,23]: - 999999 v[18,23] - service_time[18]
   - start_service[18] + start_service[23] >= -999999
 StartServiceRobots[19,1]: - 1.0001135742778145e+06 v[19,1]
   - service_time[19] + start_service[1] - start_service[19] >= -999999
 StartServiceRobots[19,2]: - 1.0000784142247903e+06 v[19,2]
   - service_time[19] + start_service[2] - start_service[19] >= -999999
 StartServiceRobots[19,3]: - 1.0000273896253681e+06 v[19,3]
   - service_time[19] + start_service[3] - start_service[19] >= -999999
 StartServiceRobots[19,4]: - 1.0000987723651387e+06 v[19,4]
   - service_time[19] + start_service[4] - start_service[19] >= -999999
 StartServiceRobots[19,5]: - 1.0000704969536402e+06 v[19,5]
   - service_time[19] + start_service[5] - start_service[19] >= -999999
 StartServiceRobots[19,6]: - 1.0000678991589429e+06 v[19,6]
   - service_time[19] + start_service[6] - start_service[19] >= -999999
 StartServiceRobots[19,7]: - 999999.8437675878 v[19,7] - service_time[19]
   + start_service[7] - start_service[19] >= -999999
 StartServiceRobots[19,8]: - 1.0000474019358882e+06 v[19,8]
   - service_time[19] + start_service[8] - start_service[19] >= -999999
 StartServiceRobots[19,9]: - 999999 v[19,9] - service_time[19]
   + start_service[9] - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 1.0000616475755909e+06 v[19,10]
   - service_time[19] + start_service[10] - start_service[19] >= -999999
 StartServiceRobots[19,11]: - 1.0000509634543548e+06 v[19,11]
   - service_time[19] + start_service[11] - start_service[19] >= -999999
 StartServiceRobots[19,12]: - 1.0000784142247903e+06 v[19,12]
   - service_time[19] + start_service[12] - start_service[19] >= -999999
 StartServiceRobots[19,13]: - 1.0000273896253681e+06 v[19,13]
   - service_time[19] + start_service[13] - start_service[19] >= -999999
 StartServiceRobots[19,14]: - 1.0000987723651387e+06 v[19,14]
   - service_time[19] + start_service[14] - start_service[19] >= -999999
 StartServiceRobots[19,15]: - 1.0000704969536402e+06 v[19,15]
   - service_time[19] + start_service[15] - start_service[19] >= -999999
 StartServiceRobots[19,16]: - 1.0000678991589429e+06 v[19,16]
   - service_time[19] + start_service[16] - start_service[19] >= -999999
 StartServiceRobots[19,17]: - 999999.8437675878 v[19,17] - service_time[19]
   + start_service[17] - start_service[19] >= -999999
 StartServiceRobots[19,18]: - 1.0000474019358882e+06 v[19,18]
   - service_time[19] + start_service[18] - start_service[19] >= -999999
 StartServiceRobots[19,19]: - 999999 v[19,19] - service_time[19]
   >= -999999
 StartServiceRobots[19,20]: - 1.0000616475755909e+06 v[19,20]
   - service_time[19] - start_service[19] + start_service[20] >= -999999
 StartServiceRobots[19,21]: - 1.0000509634543548e+06 v[19,21]
   - service_time[19] - start_service[19] + start_service[21] >= -999999
 StartServiceRobots[19,22]: - 999999 v[19,22] - service_time[19]
   - start_service[19] + start_service[22] >= -999999
 StartServiceRobots[19,23]: - 999999 v[19,23] - service_time[19]
   - start_service[19] + start_service[23] >= -999999
 StartServiceRobots[20,1]: - 1.0000665133082913e+06 v[20,1]
   - service_time[20] + start_service[1] - start_service[20] >= -999999
 StartServiceRobots[20,2]: - 1.0000253782266368e+06 v[20,2]
   - service_time[20] + start_service[2] - start_service[20] >= -999999
 StartServiceRobots[20,3]: - 1.0000361135195905e+06 v[20,3]
   - service_time[20] + start_service[3] - start_service[20] >= -999999
 StartServiceRobots[20,4]: - 1.0000566241376635e+06 v[20,4]
   - service_time[20] + start_service[4] - start_service[20] >= -999999
 StartServiceRobots[20,5]: - 1.0000216805964843e+06 v[20,5]
   - service_time[20] + start_service[5] - start_service[20] >= -999999
 StartServiceRobots[20,6]: - 1.0000287929886794e+06 v[20,6]
   - service_time[20] + start_service[6] - start_service[20] >= -999999
 StartServiceRobots[20,7]: - 1.00006122839988e+06 v[20,7]
   - service_time[20] + start_service[7] - start_service[20] >= -999999
 StartServiceRobots[20,8]: - 1.0000135594043691e+06 v[20,8]
   - service_time[20] + start_service[8] - start_service[20] >= -999999
 StartServiceRobots[20,9]: - 1.0000616475755909e+06 v[20,9]
   - service_time[20] + start_service[9] - start_service[20] >= -999999
 StartServiceRobots[20,10]: - 999999 v[20,10] - service_time[20]
   + start_service[10] - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 1.0000204820647413e+06 v[20,11]
   - service_time[20] + start_service[11] - start_service[20] >= -999999
 StartServiceRobots[20,12]: - 1.0000253782266368e+06 v[20,12]
   - service_time[20] + start_service[12] - start_service[20] >= -999999
 StartServiceRobots[20,13]: - 1.0000361135195905e+06 v[20,13]
   - service_time[20] + start_service[13] - start_service[20] >= -999999
 StartServiceRobots[20,14]: - 1.0000566241376635e+06 v[20,14]
   - service_time[20] + start_service[14] - start_service[20] >= -999999
 StartServiceRobots[20,15]: - 1.0000216805964843e+06 v[20,15]
   - service_time[20] + start_service[15] - start_service[20] >= -999999
 StartServiceRobots[20,16]: - 1.0000287929886794e+06 v[20,16]
   - service_time[20] + start_service[16] - start_service[20] >= -999999
 StartServiceRobots[20,17]: - 1.00006122839988e+06 v[20,17]
   - service_time[20] + start_service[17] - start_service[20] >= -999999
 StartServiceRobots[20,18]: - 1.0000135594043691e+06 v[20,18]
   - service_time[20] + start_service[18] - start_service[20] >= -999999
 StartServiceRobots[20,19]: - 1.0000616475755909e+06 v[20,19]
   - service_time[20] + start_service[19] - start_service[20] >= -999999
 StartServiceRobots[20,20]: - 999999 v[20,20] - service_time[20]
   >= -999999
 StartServiceRobots[20,21]: - 1.0000204820647413e+06 v[20,21]
   - service_time[20] - start_service[20] + start_service[21] >= -999999
 StartServiceRobots[20,22]: - 999999 v[20,22] - service_time[20]
   - start_service[20] + start_service[22] >= -999999
 StartServiceRobots[20,23]: - 999999 v[20,23] - service_time[20]
   - start_service[20] + start_service[23] >= -999999
 StartServiceRobots[21,1]: - 1.0000632444353482e+06 v[21,1]
   - service_time[21] + start_service[1] - start_service[21] >= -999999
 StartServiceRobots[21,2]: - 1.0000265613006055e+06 v[21,2]
   - service_time[21] + start_service[2] - start_service[21] >= -999999
 StartServiceRobots[21,3]: - 1.0000225783656818e+06 v[21,3]
   - service_time[21] + start_service[3] - start_service[21] >= -999999
 StartServiceRobots[21,4]: - 1.0000497453427232e+06 v[21,4]
   - service_time[21] + start_service[4] - start_service[21] >= -999999
 StartServiceRobots[21,5]: - 1.0000185543180438e+06 v[21,5]
   - service_time[21] + start_service[5] - start_service[21] >= -999999
 StartServiceRobots[21,6]: - 1.0000472983184841e+06 v[21,6]
   - service_time[21] + start_service[6] - start_service[21] >= -999999
 StartServiceRobots[21,7]: - 1.0000503303973848e+06 v[21,7]
   - service_time[21] + start_service[7] - start_service[21] >= -999999
 StartServiceRobots[21,8]: - 1.0000131302091408e+06 v[21,8]
   - service_time[21] + start_service[8] - start_service[21] >= -999999
 StartServiceRobots[21,9]: - 1.0000509634543548e+06 v[21,9]
   - service_time[21] + start_service[9] - start_service[21] >= -999999
 StartServiceRobots[21,10]: - 1.0000204820647413e+06 v[21,10]
   - service_time[21] + start_service[10] - start_service[21] >= -999999
 StartServiceRobots[21,11]: - 999999 v[21,11] - service_time[21]
   + start_service[11] - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 1.0000265613006055e+06 v[21,12]
   - service_time[21] + start_service[12] - start_service[21] >= -999999
 StartServiceRobots[21,13]: - 1.0000225783656818e+06 v[21,13]
   - service_time[21] + start_service[13] - start_service[21] >= -999999
 StartServiceRobots[21,14]: - 1.0000497453427232e+06 v[21,14]
   - service_time[21] + start_service[14] - start_service[21] >= -999999
 StartServiceRobots[21,15]: - 1.0000185543180438e+06 v[21,15]
   - service_time[21] + start_service[15] - start_service[21] >= -999999
 StartServiceRobots[21,16]: - 1.0000472983184841e+06 v[21,16]
   - service_time[21] + start_service[16] - start_service[21] >= -999999
 StartServiceRobots[21,17]: - 1.0000503303973848e+06 v[21,17]
   - service_time[21] + start_service[17] - start_service[21] >= -999999
 StartServiceRobots[21,18]: - 1.0000131302091408e+06 v[21,18]
   - service_time[21] + start_service[18] - start_service[21] >= -999999
 StartServiceRobots[21,19]: - 1.0000509634543548e+06 v[21,19]
   - service_time[21] + start_service[19] - start_service[21] >= -999999
 StartServiceRobots[21,20]: - 1.0000204820647413e+06 v[21,20]
   - service_time[21] + start_service[20] - start_service[21] >= -999999
 StartServiceRobots[21,21]: - 999999 v[21,21] - service_time[21]
   >= -999999
 StartServiceRobots[21,22]: - 999999 v[21,22] - service_time[21]
   - start_service[21] + start_service[22] >= -999999
 StartServiceRobots[21,23]: - 999999 v[21,23] - service_time[21]
   - start_service[21] + start_service[23] >= -999999
 StartServiceRobots[22,1]: - 999999 v[22,1] - service_time[22]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 1.0000401805928103e+06 v[22,2]
   - service_time[22] + start_service[2] - start_service[22] >= -999999
 StartServiceRobots[22,3]: - 1.0000857578554453e+06 v[22,3]
   - service_time[22] + start_service[3] - start_service[22] >= -999999
 StartServiceRobots[22,4]: - 1.0000147366179901e+06 v[22,4]
   - service_time[22] + start_service[4] - start_service[22] >= -999999
 StartServiceRobots[22,5]: - 1.0000457716821638e+06 v[22,5]
   - service_time[22] + start_service[5] - start_service[22] >= -999999
 StartServiceRobots[22,6]: - 1.0000930531941031e+06 v[22,6]
   - service_time[22] + start_service[6] - start_service[22] >= -999999
 StartServiceRobots[22,7]: - 1.0001128471096832e+06 v[22,7]
   - service_time[22] + start_service[7] - start_service[22] >= -999999
 StartServiceRobots[22,8]: - 1.0000733867898309e+06 v[22,8]
   - service_time[22] + start_service[8] - start_service[22] >= -999999
 StartServiceRobots[22,9]: - 1.0001135742778145e+06 v[22,9]
   - service_time[22] + start_service[9] - start_service[22] >= -999999
 StartServiceRobots[22,10]: - 1.0000665133082913e+06 v[22,10]
   - service_time[22] + start_service[10] - start_service[22] >= -999999
 StartServiceRobots[22,11]: - 1.0000632444353482e+06 v[22,11]
   - service_time[22] + start_service[11] - start_service[22] >= -999999
 StartServiceRobots[22,12]: - 1.0000401805928103e+06 v[22,12]
   - service_time[22] + start_service[12] - start_service[22] >= -999999
 StartServiceRobots[22,13]: - 1.0000857578554453e+06 v[22,13]
   - service_time[22] + start_service[13] - start_service[22] >= -999999
 StartServiceRobots[22,14]: - 1.0000147366179901e+06 v[22,14]
   - service_time[22] + start_service[14] - start_service[22] >= -999999
 StartServiceRobots[22,15]: - 1.0000457716821638e+06 v[22,15]
   - service_time[22] + start_service[15] - start_service[22] >= -999999
 StartServiceRobots[22,16]: - 1.0000930531941031e+06 v[22,16]
   - service_time[22] + start_service[16] - start_service[22] >= -999999
 StartServiceRobots[22,17]: - 1.0001128471096832e+06 v[22,17]
   - service_time[22] + start_service[17] - start_service[22] >= -999999
 StartServiceRobots[22,18]: - 1.0000733867898309e+06 v[22,18]
   - service_time[22] + start_service[18] - start_service[22] >= -999999
 StartServiceRobots[22,19]: - 1.0001135742778145e+06 v[22,19]
   - service_time[22] + start_service[19] - start_service[22] >= -999999
 StartServiceRobots[22,20]: - 1.0000665133082913e+06 v[22,20]
   - service_time[22] + start_service[20] - start_service[22] >= -999999
 StartServiceRobots[22,21]: - 1.0000632444353482e+06 v[22,21]
   - service_time[22] + start_service[21] - start_service[22] >= -999999
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
 SettingMode[2]: - w[1,2] - w[2,2] - w[3,2] - w[4,2] - w[5,2] - w[6,2]
   - w[7,2] - w[8,2] - w[9,2] - w[10,2] - w[11,2] - w[12,2] - w[13,2]
   - w[14,2] - w[15,2] - w[16,2] - w[17,2] - w[18,2] - w[19,2] - w[20,2]
   - w[21,2] - w[22,2] + y[2,1] + 2 y[2,2] = 0
 SettingMode[3]: - w[1,3] - w[2,3] - w[3,3] - w[4,3] - w[5,3] - w[6,3]
   - w[7,3] - w[8,3] - w[9,3] - w[10,3] - w[11,3] - w[12,3] - w[13,3]
   - w[14,3] - w[15,3] - w[16,3] - w[17,3] - w[18,3] - w[19,3] - w[20,3]
   - w[21,3] - w[22,3] + y[3,1] + 2 y[3,2] = 0
 SettingMode[4]: - w[1,4] - w[2,4] - w[3,4] - w[4,4] - w[5,4] - w[6,4]
   - w[7,4] - w[8,4] - w[9,4] - w[10,4] - w[11,4] - w[12,4] - w[13,4]
   - w[14,4] - w[15,4] - w[16,4] - w[17,4] - w[18,4] - w[19,4] - w[20,4]
   - w[21,4] - w[22,4] + y[4,1] + 2 y[4,2] = 0
 SettingMode[5]: - w[1,5] - w[2,5] - w[3,5] - w[4,5] - w[5,5] - w[6,5]
   - w[7,5] - w[8,5] - w[9,5] - w[10,5] - w[11,5] - w[12,5] - w[13,5]
   - w[14,5] - w[15,5] - w[16,5] - w[17,5] - w[18,5] - w[19,5] - w[20,5]
   - w[21,5] - w[22,5] + y[5,1] + 2 y[5,2] = 0
 SettingMode[6]: - w[1,6] - w[2,6] - w[3,6] - w[4,6] - w[5,6] - w[6,6]
   - w[7,6] - w[8,6] - w[9,6] - w[10,6] - w[11,6] - w[12,6] - w[13,6]
   - w[14,6] - w[15,6] - w[16,6] - w[17,6] - w[18,6] - w[19,6] - w[20,6]
   - w[21,6] - w[22,6] + y[6,1] + 2 y[6,2] = 0
 SettingMode[7]: - w[1,7] - w[2,7] - w[3,7] - w[4,7] - w[5,7] - w[6,7]
   - w[7,7] - w[8,7] - w[9,7] - w[10,7] - w[11,7] - w[12,7] - w[13,7]
   - w[14,7] - w[15,7] - w[16,7] - w[17,7] - w[18,7] - w[19,7] - w[20,7]
   - w[21,7] - w[22,7] + y[7,1] + 2 y[7,2] = 0
 SettingMode[8]: - w[1,8] - w[2,8] - w[3,8] - w[4,8] - w[5,8] - w[6,8]
   - w[7,8] - w[8,8] - w[9,8] - w[10,8] - w[11,8] - w[12,8] - w[13,8]
   - w[14,8] - w[15,8] - w[16,8] - w[17,8] - w[18,8] - w[19,8] - w[20,8]
   - w[21,8] - w[22,8] + y[8,1] + 2 y[8,2] = 0
 SettingMode[9]: - w[1,9] - w[2,9] - w[3,9] - w[4,9] - w[5,9] - w[6,9]
   - w[7,9] - w[8,9] - w[9,9] - w[10,9] - w[11,9] - w[12,9] - w[13,9]
   - w[14,9] - w[15,9] - w[16,9] - w[17,9] - w[18,9] - w[19,9] - w[20,9]
   - w[21,9] - w[22,9] + y[9,1] + 2 y[9,2] = 0
 SettingMode[10]: - w[1,10] - w[2,10] - w[3,10] - w[4,10] - w[5,10]
   - w[6,10] - w[7,10] - w[8,10] - w[9,10] - w[10,10] - w[11,10] - w[12,10]
   - w[13,10] - w[14,10] - w[15,10] - w[16,10] - w[17,10] - w[18,10]
   - w[19,10] - w[20,10] - w[21,10] - w[22,10] + y[10,1] + 2 y[10,2] = 0
 SettingMode[11]: - w[1,11] - w[2,11] - w[3,11] - w[4,11] - w[5,11]
   - w[6,11] - w[7,11] - w[8,11] - w[9,11] - w[10,11] - w[11,11] - w[12,11]
   - w[13,11] - w[14,11] - w[15,11] - w[16,11] - w[17,11] - w[18,11]
   - w[19,11] - w[20,11] - w[21,11] - w[22,11] + y[11,1] + 2 y[11,2] = 0
 SettingServiceTime[2]: - 3100 y[2,1] - 1550 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 2800 y[3,1] - 1400 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 2800 y[4,1] - 1400 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 4600 y[5,1] - 2300 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 2800 y[6,1] - 1400 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 4100 y[7,1] - 2050 y[7,2] + service_time[7] = 0
 SettingServiceTime[8]: - 4600 y[8,1] - 2300 y[8,2] + service_time[8] = 0
 SettingServiceTime[9]: - 2000 y[9,1] - 1000 y[9,2] + service_time[9] = 0
 SettingServiceTime[10]: - 3000 y[10,1] - 1500 y[10,2] + service_time[10]
   = 0
 SettingServiceTime[11]: - 4800 y[11,1] - 2400 y[11,2] + service_time[11]
   = 0
 AverageTimeRobots: - 5.882941830038524 w[0,2] - 12.3939793493244 w[0,3]
   - 2.248088284291125 w[0,4] - 6.681668880551247 w[0,5]
   - 13.43617058615687 w[0,6] - 16.26387281188105 w[0,7]
   - 10.62668426155347 w[0,8] - 16.36775397350477 w[0,9]
   - 9.6447583273267 w[0,10] - 9.177776478310765 w[0,11]
   - 5.882941830038524 w[0,12] - 12.3939793493244 w[0,13]
   - 2.248088284291125 w[0,14] - 6.681668880551247 w[0,15]
   - 13.43617058615687 w[0,16] - 16.26387281188105 w[0,17]
   - 10.62668426155347 w[0,18] - 16.36775397350477 w[0,19]
   - 9.6447583273267 w[0,20] - 9.177776478310765 w[0,21]
   - 5.882941830038524 w[1,2] - 12.3939793493244 w[1,3]
   - 2.248088284291125 w[1,4] - 6.681668880551247 w[1,5]
   - 13.43617058615687 w[1,6] - 16.26387281188105 w[1,7]
   - 10.62668426155347 w[1,8] - 16.36775397350477 w[1,9]
   - 9.6447583273267 w[1,10] - 9.177776478310765 w[1,11]
   - 5.882941830038524 w[1,12] - 12.3939793493244 w[1,13]
   - 2.248088284291125 w[1,14] - 6.681668880551247 w[1,15]
   - 13.43617058615687 w[1,16] - 16.26387281188105 w[1,17]
   - 10.62668426155347 w[1,18] - 16.36775397350477 w[1,19]
   - 9.6447583273267 w[1,20] - 9.177776478310765 w[1,21]
   - 5.882941830038524 w[2,1] - 7.297248945906314 w[2,3]
   - 4.541787786430247 w[2,4] - 1.15918910725489 w[2,5]
   - 7.757941539250877 w[2,6] - 11.25743472637459 w[2,7]
   - 4.893300717405878 w[2,8] - 11.34488925575209 w[2,9]
   - 3.768318090962095 w[2,10] - 3.937328657930143 w[2,11]
   - 7.297248945906314 w[2,13] - 4.541787786430247 w[2,14]
   - 1.15918910725489 w[2,15] - 7.757941539250877 w[2,16]
   - 11.25743472637459 w[2,17] - 4.893300717405878 w[2,18]
   - 11.34488925575209 w[2,19] - 3.768318090962095 w[2,20]
   - 3.937328657930143 w[2,21] - 12.3939793493244 w[3,1]
   - 7.297248945906314 w[3,2] - 10.33914696055559 w[3,4]
   - 6.160612343581414 w[3,5] - 7.54841814164813 w[3,6]
   - 3.964618556366802 w[3,7] - 3.231783026150763 w[3,8]
   - 4.055660766857629 w[3,9] - 5.301931370062465 w[3,10]
   - 3.368337954549703 w[3,11] - 7.297248945906314 w[3,12]
   - 10.33914696055559 w[3,14] - 6.160612343581414 w[3,15]
   - 7.54841814164813 w[3,16] - 3.964618556366802 w[3,17]
   - 3.231783026150763 w[3,18] - 4.055660766857629 w[3,19]
   - 5.301931370062465 w[3,20] - 3.368337954549703 w[3,21]
   - 2.248088284291125 w[4,1] - 4.541787786430247 w[4,2]
   - 10.33914696055559 w[4,3] - 5.048212057815362 w[4,5]
   - 12.29972889038682 w[4,6] - 14.14639451295712 w[4,7]
   - 8.886496392560238 w[4,8] - 14.25319501982308 w[4,9]
   - 8.232019666208187 w[4,10] - 7.249334674738049 w[4,11]
   - 4.541787786430247 w[4,12] - 10.33914696055559 w[4,13]
   - 5.048212057815362 w[4,15] - 12.29972889038682 w[4,16]
   - 14.14639451295712 w[4,17] - 8.886496392560238 w[4,18]
   - 14.25319501982308 w[4,19] - 8.232019666208187 w[4,20]
   - 7.249334674738049 w[4,21] - 6.681668880551247 w[5,1]
   - 1.15918910725489 w[5,2] - 6.160612343581414 w[5,3]
   - 5.048212057815362 w[5,4] - 7.45114594644753 w[5,6]
   - 10.1244885111843 w[5,7] - 3.947894286368979 w[5,8]
   - 10.21385052004081 w[5,9] - 3.240085212048028 w[5,10]
   - 2.793474006256249 w[5,11] - 1.15918910725489 w[5,12]
   - 6.160612343581414 w[5,13] - 5.048212057815362 w[5,14]
   - 7.45114594644753 w[5,16] - 10.1244885111843 w[5,17]
   - 3.947894286368979 w[5,18] - 10.21385052004081 w[5,19]
   - 3.240085212048028 w[5,20] - 2.793474006256249 w[5,21]
   - 13.43617058615687 w[6,1] - 7.757941539250877 w[6,2]
   - 7.54841814164813 w[6,3] - 12.29972889038682 w[6,4]
   - 7.45114594644753 w[6,5] - 9.833999973208464 w[6,7]
   - 5.034696012888882 w[6,8] - 9.842736991834967 w[6,9]
   - 4.25614123990918 w[6,10] - 6.899759783444865 w[6,11]
   - 7.757941539250877 w[6,12] - 7.54841814164813 w[6,13]
   - 12.29972889038682 w[6,14] - 7.45114594644753 w[6,15]
   - 9.833999973208464 w[6,17] - 5.034696012888882 w[6,18]
   - 9.842736991834967 w[6,19] - 4.25614123990918 w[6,20]
   - 6.899759783444865 w[6,21] - 16.26387281188105 w[7,1]
   - 11.25743472637459 w[7,2] - 3.964618556366802 w[7,3]
   - 14.14639451295712 w[7,4] - 10.1244885111843 w[7,5]
   - 9.833999973208464 w[7,6] - 6.849210686688249 w[7,8]
   - 0.1205382268208547 w[7,9] - 8.889771411420645 w[7,10]
   - 7.332913912121832 w[7,11] - 11.25743472637459 w[7,12]
   - 3.964618556366802 w[7,13] - 14.14639451295712 w[7,14]
   - 10.1244885111843 w[7,15] - 9.833999973208464 w[7,16]
   - 6.849210686688249 w[7,18] - 0.1205382268208547 w[7,19]
   - 8.889771411420645 w[7,20] - 7.332913912121832 w[7,21]
   - 10.62668426155347 w[8,1] - 4.893300717405878 w[8,2]
   - 3.231783026150763 w[8,3] - 8.886496392560238 w[8,4]
   - 3.947894286368979 w[8,5] - 5.034696012888882 w[8,6]
   - 6.849210686688249 w[8,7] - 6.914562269744982 w[8,9]
   - 2.079914909863166 w[8,10] - 2.018601305828331 w[8,11]
   - 4.893300717405878 w[8,12] - 3.231783026150763 w[8,13]
   - 8.886496392560238 w[8,14] - 3.947894286368979 w[8,15]
   - 5.034696012888882 w[8,16] - 6.849210686688249 w[8,17]
   - 6.914562269744982 w[8,19] - 2.079914909863166 w[8,20]
   - 2.018601305828331 w[8,21] - 16.36775397350477 w[9,1]
   - 11.34488925575209 w[9,2] - 4.055660766857629 w[9,3]
   - 14.25319501982308 w[9,4] - 10.21385052004081 w[9,5]
   - 9.842736991834967 w[9,6] - 0.1205382268208547 w[9,7]
   - 6.914562269744982 w[9,8] - 8.949653655839008 w[9,10]
   - 7.423350622105616 w[9,11] - 11.34488925575209 w[9,12]
   - 4.055660766857629 w[9,13] - 14.25319501982308 w[9,14]
   - 10.21385052004081 w[9,15] - 9.842736991834967 w[9,16]
   - 0.1205382268208547 w[9,17] - 6.914562269744982 w[9,18]
   - 8.949653655839008 w[9,20] - 7.423350622105616 w[9,21]
   - 9.6447583273267 w[10,1] - 3.768318090962095 w[10,2]
   - 5.301931370062465 w[10,3] - 8.232019666208187 w[10,4]
   - 3.240085212048028 w[10,5] - 4.25614123990918 w[10,6]
   - 8.889771411420645 w[10,7] - 2.079914909863166 w[10,8]
   - 8.949653655839008 w[10,9] - 3.068866391623932 w[10,11]
   - 3.768318090962095 w[10,12] - 5.301931370062465 w[10,13]
   - 8.232019666208187 w[10,14] - 3.240085212048028 w[10,15]
   - 4.25614123990918 w[10,16] - 8.889771411420645 w[10,17]
   - 2.079914909863166 w[10,18] - 8.949653655839008 w[10,19]
   - 3.068866391623932 w[10,21] - 9.177776478310765 w[11,1]
   - 3.937328657930143 w[11,2] - 3.368337954549703 w[11,3]
   - 7.249334674738049 w[11,4] - 2.793474006256249 w[11,5]
   - 6.899759783444865 w[11,6] - 7.332913912121832 w[11,7]
   - 2.018601305828331 w[11,8] - 7.423350622105616 w[11,9]
   - 3.068866391623932 w[11,10] - 3.937328657930143 w[11,12]
   - 3.368337954549703 w[11,13] - 7.249334674738049 w[11,14]
   - 2.793474006256249 w[11,15] - 6.899759783444865 w[11,16]
   - 7.332913912121832 w[11,17] - 2.018601305828331 w[11,18]
   - 7.423350622105616 w[11,19] - 3.068866391623932 w[11,20]
   - 5.882941830038524 w[12,1] - 7.297248945906314 w[12,3]
   - 4.541787786430247 w[12,4] - 1.15918910725489 w[12,5]
   - 7.757941539250877 w[12,6] - 11.25743472637459 w[12,7]
   - 4.893300717405878 w[12,8] - 11.34488925575209 w[12,9]
   - 3.768318090962095 w[12,10] - 3.937328657930143 w[12,11]
   - 7.297248945906314 w[12,13] - 4.541787786430247 w[12,14]
   - 1.15918910725489 w[12,15] - 7.757941539250877 w[12,16]
   - 11.25743472637459 w[12,17] - 4.893300717405878 w[12,18]
   - 11.34488925575209 w[12,19] - 3.768318090962095 w[12,20]
   - 3.937328657930143 w[12,21] - 12.3939793493244 w[13,1]
   - 7.297248945906314 w[13,2] - 10.33914696055559 w[13,4]
   - 6.160612343581414 w[13,5] - 7.54841814164813 w[13,6]
   - 3.964618556366802 w[13,7] - 3.231783026150763 w[13,8]
   - 4.055660766857629 w[13,9] - 5.301931370062465 w[13,10]
   - 3.368337954549703 w[13,11] - 7.297248945906314 w[13,12]
   - 10.33914696055559 w[13,14] - 6.160612343581414 w[13,15]
   - 7.54841814164813 w[13,16] - 3.964618556366802 w[13,17]
   - 3.231783026150763 w[13,18] - 4.055660766857629 w[13,19]
   - 5.301931370062465 w[13,20] - 3.368337954549703 w[13,21]
   - 2.248088284291125 w[14,1] - 4.541787786430247 w[14,2]
   - 10.33914696055559 w[14,3] - 5.048212057815362 w[14,5]
   - 12.29972889038682 w[14,6] - 14.14639451295712 w[14,7]
   - 8.886496392560238 w[14,8] - 14.25319501982308 w[14,9]
   - 8.232019666208187 w[14,10] - 7.249334674738049 w[14,11]
   - 4.541787786430247 w[14,12] - 10.33914696055559 w[14,13]
   - 5.048212057815362 w[14,15] - 12.29972889038682 w[14,16]
   - 14.14639451295712 w[14,17] - 8.886496392560238 w[14,18]
   - 14.25319501982308 w[14,19] - 8.232019666208187 w[14,20]
   - 7.249334674738049 w[14,21] - 6.681668880551247 w[15,1]
   - 1.15918910725489 w[15,2] - 6.160612343581414 w[15,3]
   - 5.048212057815362 w[15,4] - 7.45114594644753 w[15,6]
   - 10.1244885111843 w[15,7] - 3.947894286368979 w[15,8]
   - 10.21385052004081 w[15,9] - 3.240085212048028 w[15,10]
   - 2.793474006256249 w[15,11] - 1.15918910725489 w[15,12]
   - 6.160612343581414 w[15,13] - 5.048212057815362 w[15,14]
   - 7.45114594644753 w[15,16] - 10.1244885111843 w[15,17]
   - 3.947894286368979 w[15,18] - 10.21385052004081 w[15,19]
   - 3.240085212048028 w[15,20] - 2.793474006256249 w[15,21]
   - 13.43617058615687 w[16,1] - 7.757941539250877 w[16,2]
   - 7.54841814164813 w[16,3] - 12.29972889038682 w[16,4]
   - 7.45114594644753 w[16,5] - 9.833999973208464 w[16,7]
   - 5.034696012888882 w[16,8] - 9.842736991834967 w[16,9]
   - 4.25614123990918 w[16,10] - 6.899759783444865 w[16,11]
   - 7.757941539250877 w[16,12] - 7.54841814164813 w[16,13]
   - 12.29972889038682 w[16,14] - 7.45114594644753 w[16,15]
   - 9.833999973208464 w[16,17] - 5.034696012888882 w[16,18]
   - 9.842736991834967 w[16,19] - 4.25614123990918 w[16,20]
   - 6.899759783444865 w[16,21] - 16.26387281188105 w[17,1]
   - 11.25743472637459 w[17,2] - 3.964618556366802 w[17,3]
   - 14.14639451295712 w[17,4] - 10.1244885111843 w[17,5]
   - 9.833999973208464 w[17,6] - 6.849210686688249 w[17,8]
   - 0.1205382268208547 w[17,9] - 8.889771411420645 w[17,10]
   - 7.332913912121832 w[17,11] - 11.25743472637459 w[17,12]
   - 3.964618556366802 w[17,13] - 14.14639451295712 w[17,14]
   - 10.1244885111843 w[17,15] - 9.833999973208464 w[17,16]
   - 6.849210686688249 w[17,18] - 0.1205382268208547 w[17,19]
   - 8.889771411420645 w[17,20] - 7.332913912121832 w[17,21]
   - 10.62668426155347 w[18,1] - 4.893300717405878 w[18,2]
   - 3.231783026150763 w[18,3] - 8.886496392560238 w[18,4]
   - 3.947894286368979 w[18,5] - 5.034696012888882 w[18,6]
   - 6.849210686688249 w[18,7] - 6.914562269744982 w[18,9]
   - 2.079914909863166 w[18,10] - 2.018601305828331 w[18,11]
   - 4.893300717405878 w[18,12] - 3.231783026150763 w[18,13]
   - 8.886496392560238 w[18,14] - 3.947894286368979 w[18,15]
   - 5.034696012888882 w[18,16] - 6.849210686688249 w[18,17]
   - 6.914562269744982 w[18,19] - 2.079914909863166 w[18,20]
   - 2.018601305828331 w[18,21] - 16.36775397350477 w[19,1]
   - 11.34488925575209 w[19,2] - 4.055660766857629 w[19,3]
   - 14.25319501982308 w[19,4] - 10.21385052004081 w[19,5]
   - 9.842736991834967 w[19,6] - 0.1205382268208547 w[19,7]
   - 6.914562269744982 w[19,8] - 8.949653655839008 w[19,10]
   - 7.423350622105616 w[19,11] - 11.34488925575209 w[19,12]
   - 4.055660766857629 w[19,13] - 14.25319501982308 w[19,14]
   - 10.21385052004081 w[19,15] - 9.842736991834967 w[19,16]
   - 0.1205382268208547 w[19,17] - 6.914562269744982 w[19,18]
   - 8.949653655839008 w[19,20] - 7.423350622105616 w[19,21]
   - 9.6447583273267 w[20,1] - 3.768318090962095 w[20,2]
   - 5.301931370062465 w[20,3] - 8.232019666208187 w[20,4]
   - 3.240085212048028 w[20,5] - 4.25614123990918 w[20,6]
   - 8.889771411420645 w[20,7] - 2.079914909863166 w[20,8]
   - 8.949653655839008 w[20,9] - 3.068866391623932 w[20,11]
   - 3.768318090962095 w[20,12] - 5.301931370062465 w[20,13]
   - 8.232019666208187 w[20,14] - 3.240085212048028 w[20,15]
   - 4.25614123990918 w[20,16] - 8.889771411420645 w[20,17]
   - 2.079914909863166 w[20,18] - 8.949653655839008 w[20,19]
   - 3.068866391623932 w[20,21] - 9.177776478310765 w[21,1]
   - 3.937328657930143 w[21,2] - 3.368337954549703 w[21,3]
   - 7.249334674738049 w[21,4] - 2.793474006256249 w[21,5]
   - 6.899759783444865 w[21,6] - 7.332913912121832 w[21,7]
   - 2.018601305828331 w[21,8] - 7.423350622105616 w[21,9]
   - 3.068866391623932 w[21,10] - 3.937328657930143 w[21,12]
   - 3.368337954549703 w[21,13] - 7.249334674738049 w[21,14]
   - 2.793474006256249 w[21,15] - 6.899759783444865 w[21,16]
   - 7.332913912121832 w[21,17] - 2.018601305828331 w[21,18]
   - 7.423350622105616 w[21,19] - 3.068866391623932 w[21,20]
   - 5.882941830038524 w[22,2] - 12.3939793493244 w[22,3]
   - 2.248088284291125 w[22,4] - 6.681668880551247 w[22,5]
   - 13.43617058615687 w[22,6] - 16.26387281188105 w[22,7]
   - 10.62668426155347 w[22,8] - 16.36775397350477 w[22,9]
   - 9.6447583273267 w[22,10] - 9.177776478310765 w[22,11]
   - 5.882941830038524 w[22,12] - 12.3939793493244 w[22,13]
   - 2.248088284291125 w[22,14] - 6.681668880551247 w[22,15]
   - 13.43617058615687 w[22,16] - 16.26387281188105 w[22,17]
   - 10.62668426155347 w[22,18] - 16.36775397350477 w[22,19]
   - 9.6447583273267 w[22,20] - 9.177776478310765 w[22,21]
   - 442.8571428571428 y[2,1] - 885.7142857142857 y[2,2] - 400 y[3,1]
   - 800 y[3,2] - 400 y[4,1] - 800 y[4,2] - 657.1428571428571 y[5,1]
   - 1314.285714285714 y[5,2] - 400 y[6,1] - 800 y[6,2]
   - 585.7142857142857 y[7,1] - 1171.428571428571 y[7,2]
   - 657.1428571428571 y[8,1] - 1314.285714285714 y[8,2]
   - 285.7142857142857 y[9,1] - 571.4285714285714 y[9,2]
   - 428.5714285714286 y[10,1] - 857.1428571428571 y[10,2]
   - 685.7142857142857 y[11,1] - 1371.428571428571 y[11,2]
   + start_service[23] >= 0
 AverageTimeVehices: - 13.72686427008989 x[0,2] - 28.91928514842361 x[0,3]
   - 5.245539330012624 x[0,4] - 15.59056072128624 x[0,5]
   - 31.3510647010327 x[0,6] - 37.94903656105578 x[0,7]
   - 24.79559661029143 x[0,8] - 38.19142593817779 x[0,9]
   - 22.50443609709563 x[0,10] - 21.41481178272512 x[0,11]
   - 13.72686427008989 x[0,12] - 28.91928514842361 x[0,13]
   - 5.245539330012624 x[0,14] - 15.59056072128624 x[0,15]
   - 31.3510647010327 x[0,16] - 37.94903656105578 x[0,17]
   - 24.79559661029143 x[0,18] - 38.19142593817779 x[0,19]
   - 22.50443609709563 x[0,20] - 21.41481178272512 x[0,21]
   - 13.72686427008989 x[1,2] - 28.91928514842361 x[1,3]
   - 5.245539330012624 x[1,4] - 15.59056072128624 x[1,5]
   - 31.3510647010327 x[1,6] - 37.94903656105578 x[1,7]
   - 24.79559661029143 x[1,8] - 38.19142593817779 x[1,9]
   - 22.50443609709563 x[1,10] - 21.41481178272512 x[1,11]
   - 13.72686427008989 x[1,12] - 28.91928514842361 x[1,13]
   - 5.245539330012624 x[1,14] - 15.59056072128624 x[1,15]
   - 31.3510647010327 x[1,16] - 37.94903656105578 x[1,17]
   - 24.79559661029143 x[1,18] - 38.19142593817779 x[1,19]
   - 22.50443609709563 x[1,20] - 21.41481178272512 x[1,21]
   - 13.72686427008989 x[2,1] - 17.02691420711473 x[2,3]
   - 10.59750483500391 x[2,4] - 2.704774583594743 x[2,5]
   - 18.10186359158538 x[2,6] - 26.26734769487405 x[2,7]
   - 11.41770167394705 x[2,8] - 26.47140826342154 x[2,9]
   - 8.792742212244889 x[2,10] - 9.187100201837 x[2,11]
   - 17.02691420711473 x[2,13] - 10.59750483500391 x[2,14]
   - 2.704774583594743 x[2,15] - 18.10186359158538 x[2,16]
   - 26.26734769487405 x[2,17] - 11.41770167394705 x[2,18]
   - 26.47140826342154 x[2,19] - 8.792742212244889 x[2,20]
   - 9.187100201837 x[2,21] - 28.91928514842361 x[3,1]
   - 17.02691420711473 x[3,2] - 24.12467624129638 x[3,4]
   - 14.3747621350233 x[3,5] - 17.61297566384563 x[3,6]
   - 9.250776631522537 x[3,7] - 7.540827061018448 x[3,8]
   - 9.463208456001135 x[3,9] - 12.37117319681242 x[3,10]
   - 7.859455227282641 x[3,11] - 17.02691420711473 x[3,12]
   - 24.12467624129638 x[3,14] - 14.3747621350233 x[3,15]
   - 17.61297566384563 x[3,16] - 9.250776631522537 x[3,17]
   - 7.540827061018448 x[3,18] - 9.463208456001135 x[3,19]
   - 12.37117319681242 x[3,20] - 7.859455227282641 x[3,21]
   - 5.245539330012624 x[4,1] - 10.59750483500391 x[4,2]
   - 24.12467624129638 x[4,3] - 11.77916146823584 x[4,5]
   - 28.69936741090257 x[4,6] - 33.00825386356662 x[4,7]
   - 20.73515824930722 x[4,8] - 33.25745504625385 x[4,9]
   - 19.2080458878191 x[4,10] - 16.91511424105545 x[4,11]
   - 10.59750483500391 x[4,12] - 24.12467624129638 x[4,13]
   - 11.77916146823584 x[4,15] - 28.69936741090257 x[4,16]
   - 33.00825386356662 x[4,17] - 20.73515824930722 x[4,18]
   - 33.25745504625385 x[4,19] - 19.2080458878191 x[4,20]
   - 16.91511424105545 x[4,21] - 15.59056072128624 x[5,1]
   - 2.704774583594743 x[5,2] - 14.3747621350233 x[5,3]
   - 11.77916146823584 x[5,4] - 17.38600720837757 x[5,6]
   - 23.6238065260967 x[5,7] - 9.211753334860951 x[5,8]
   - 23.83231788009521 x[5,9] - 7.560198828112066 x[5,10]
   - 6.518106014597915 x[5,11] - 2.704774583594743 x[5,12]
   - 14.3747621350233 x[5,13] - 11.77916146823584 x[5,14]
   - 17.38600720837757 x[5,16] - 23.6238065260967 x[5,17]
   - 9.211753334860951 x[5,18] - 23.83231788009521 x[5,19]
   - 7.560198828112066 x[5,20] - 6.518106014597915 x[5,21]
   - 31.3510647010327 x[6,1] - 18.10186359158538 x[6,2]
   - 17.61297566384563 x[6,3] - 28.69936741090257 x[6,4]
   - 17.38600720837757 x[6,5] - 22.94599993748641 x[6,7]
   - 11.74762403007406 x[6,8] - 22.96638631428159 x[6,9]
   - 9.930996226454752 x[6,10] - 16.09943949470468 x[6,11]
   - 18.10186359158538 x[6,12] - 17.61297566384563 x[6,13]
   - 28.69936741090257 x[6,14] - 17.38600720837757 x[6,15]
   - 22.94599993748641 x[6,17] - 11.74762403007406 x[6,18]
   - 22.96638631428159 x[6,19] - 9.930996226454752 x[6,20]
   - 16.09943949470468 x[6,21] - 37.94903656105578 x[7,1]
   - 26.26734769487405 x[7,2] - 9.250776631522537 x[7,3]
   - 33.00825386356662 x[7,4] - 23.6238065260967 x[7,5]
   - 22.94599993748641 x[7,6] - 15.98149160227258 x[7,8]
   - 0.2812558625819943 x[7,9] - 20.7427999599815 x[7,10]
   - 17.11013246161761 x[7,11] - 26.26734769487405 x[7,12]
   - 9.250776631522537 x[7,13] - 33.00825386356662 x[7,14]
   - 23.6238065260967 x[7,15] - 22.94599993748641 x[7,16]
   - 15.98149160227258 x[7,18] - 0.2812558625819943 x[7,19]
   - 20.7427999599815 x[7,20] - 17.11013246161761 x[7,21]
   - 24.79559661029143 x[8,1] - 11.41770167394705 x[8,2]
   - 7.540827061018448 x[8,3] - 20.73515824930722 x[8,4]
   - 9.211753334860951 x[8,5] - 11.74762403007406 x[8,6]
   - 15.98149160227258 x[8,7] - 16.13397862940496 x[8,9]
   - 4.853134789680722 x[8,10] - 4.710069713599438 x[8,11]
   - 11.41770167394705 x[8,12] - 7.540827061018448 x[8,13]
   - 20.73515824930722 x[8,14] - 9.211753334860951 x[8,15]
   - 11.74762403007406 x[8,16] - 15.98149160227258 x[8,17]
   - 16.13397862940496 x[8,19] - 4.853134789680722 x[8,20]
   - 4.710069713599438 x[8,21] - 38.19142593817779 x[9,1]
   - 26.47140826342154 x[9,2] - 9.463208456001135 x[9,3]
   - 33.25745504625385 x[9,4] - 23.83231788009521 x[9,5]
   - 22.96638631428159 x[9,6] - 0.2812558625819943 x[9,7]
   - 16.13397862940496 x[9,8] - 20.88252519695769 x[9,10]
   - 17.32115145157977 x[9,11] - 26.47140826342154 x[9,12]
   - 9.463208456001135 x[9,13] - 33.25745504625385 x[9,14]
   - 23.83231788009521 x[9,15] - 22.96638631428159 x[9,16]
   - 0.2812558625819943 x[9,17] - 16.13397862940496 x[9,18]
   - 20.88252519695769 x[9,20] - 17.32115145157977 x[9,21]
   - 22.50443609709563 x[10,1] - 8.792742212244889 x[10,2]
   - 12.37117319681242 x[10,3] - 19.2080458878191 x[10,4]
   - 7.560198828112066 x[10,5] - 9.930996226454752 x[10,6]
   - 20.7427999599815 x[10,7] - 4.853134789680722 x[10,8]
   - 20.88252519695769 x[10,9] - 7.160688247122508 x[10,11]
   - 8.792742212244889 x[10,12] - 12.37117319681242 x[10,13]
   - 19.2080458878191 x[10,14] - 7.560198828112066 x[10,15]
   - 9.930996226454752 x[10,16] - 20.7427999599815 x[10,17]
   - 4.853134789680722 x[10,18] - 20.88252519695769 x[10,19]
   - 7.160688247122508 x[10,21] - 21.41481178272512 x[11,1]
   - 9.187100201837 x[11,2] - 7.859455227282641 x[11,3]
   - 16.91511424105545 x[11,4] - 6.518106014597915 x[11,5]
   - 16.09943949470468 x[11,6] - 17.11013246161761 x[11,7]
   - 4.710069713599438 x[11,8] - 17.32115145157977 x[11,9]
   - 7.160688247122508 x[11,10] - 9.187100201837 x[11,12]
   - 7.859455227282641 x[11,13] - 16.91511424105545 x[11,14]
   - 6.518106014597915 x[11,15] - 16.09943949470468 x[11,16]
   - 17.11013246161761 x[11,17] - 4.710069713599438 x[11,18]
   - 17.32115145157977 x[11,19] - 7.160688247122508 x[11,20]
   - 13.72686427008989 x[12,1] - 17.02691420711473 x[12,3]
   - 10.59750483500391 x[12,4] - 2.704774583594743 x[12,5]
   - 18.10186359158538 x[12,6] - 26.26734769487405 x[12,7]
   - 11.41770167394705 x[12,8] - 26.47140826342154 x[12,9]
   - 8.792742212244889 x[12,10] - 9.187100201837 x[12,11]
   - 17.02691420711473 x[12,13] - 10.59750483500391 x[12,14]
   - 2.704774583594743 x[12,15] - 18.10186359158538 x[12,16]
   - 26.26734769487405 x[12,17] - 11.41770167394705 x[12,18]
   - 26.47140826342154 x[12,19] - 8.792742212244889 x[12,20]
   - 9.187100201837 x[12,21] - 28.91928514842361 x[13,1]
   - 17.02691420711473 x[13,2] - 24.12467624129638 x[13,4]
   - 14.3747621350233 x[13,5] - 17.61297566384563 x[13,6]
   - 9.250776631522537 x[13,7] - 7.540827061018448 x[13,8]
   - 9.463208456001135 x[13,9] - 12.37117319681242 x[13,10]
   - 7.859455227282641 x[13,11] - 17.02691420711473 x[13,12]
   - 24.12467624129638 x[13,14] - 14.3747621350233 x[13,15]
   - 17.61297566384563 x[13,16] - 9.250776631522537 x[13,17]
   - 7.540827061018448 x[13,18] - 9.463208456001135 x[13,19]
   - 12.37117319681242 x[13,20] - 7.859455227282641 x[13,21]
   - 5.245539330012624 x[14,1] - 10.59750483500391 x[14,2]
   - 24.12467624129638 x[14,3] - 11.77916146823584 x[14,5]
   - 28.69936741090257 x[14,6] - 33.00825386356662 x[14,7]
   - 20.73515824930722 x[14,8] - 33.25745504625385 x[14,9]
   - 19.2080458878191 x[14,10] - 16.91511424105545 x[14,11]
   - 10.59750483500391 x[14,12] - 24.12467624129638 x[14,13]
   - 11.77916146823584 x[14,15] - 28.69936741090257 x[14,16]
   - 33.00825386356662 x[14,17] - 20.73515824930722 x[14,18]
   - 33.25745504625385 x[14,19] - 19.2080458878191 x[14,20]
   - 16.91511424105545 x[14,21] - 15.59056072128624 x[15,1]
   - 2.704774583594743 x[15,2] - 14.3747621350233 x[15,3]
   - 11.77916146823584 x[15,4] - 17.38600720837757 x[15,6]
   - 23.6238065260967 x[15,7] - 9.211753334860951 x[15,8]
   - 23.83231788009521 x[15,9] - 7.560198828112066 x[15,10]
   - 6.518106014597915 x[15,11] - 2.704774583594743 x[15,12]
   - 14.3747621350233 x[15,13] - 11.77916146823584 x[15,14]
   - 17.38600720837757 x[15,16] - 23.6238065260967 x[15,17]
   - 9.211753334860951 x[15,18] - 23.83231788009521 x[15,19]
   - 7.560198828112066 x[15,20] - 6.518106014597915 x[15,21]
   - 31.3510647010327 x[16,1] - 18.10186359158538 x[16,2]
   - 17.61297566384563 x[16,3] - 28.69936741090257 x[16,4]
   - 17.38600720837757 x[16,5] - 22.94599993748641 x[16,7]
   - 11.74762403007406 x[16,8] - 22.96638631428159 x[16,9]
   - 9.930996226454752 x[16,10] - 16.09943949470468 x[16,11]
   - 18.10186359158538 x[16,12] - 17.61297566384563 x[16,13]
   - 28.69936741090257 x[16,14] - 17.38600720837757 x[16,15]
   - 22.94599993748641 x[16,17] - 11.74762403007406 x[16,18]
   - 22.96638631428159 x[16,19] - 9.930996226454752 x[16,20]
   - 16.09943949470468 x[16,21] - 37.94903656105578 x[17,1]
   - 26.26734769487405 x[17,2] - 9.250776631522537 x[17,3]
   - 33.00825386356662 x[17,4] - 23.6238065260967 x[17,5]
   - 22.94599993748641 x[17,6] - 15.98149160227258 x[17,8]
   - 0.2812558625819943 x[17,9] - 20.7427999599815 x[17,10]
   - 17.11013246161761 x[17,11] - 26.26734769487405 x[17,12]
   - 9.250776631522537 x[17,13] - 33.00825386356662 x[17,14]
   - 23.6238065260967 x[17,15] - 22.94599993748641 x[17,16]
   - 15.98149160227258 x[17,18] - 0.2812558625819943 x[17,19]
   - 20.7427999599815 x[17,20] - 17.11013246161761 x[17,21]
   - 24.79559661029143 x[18,1] - 11.41770167394705 x[18,2]
   - 7.540827061018448 x[18,3] - 20.73515824930722 x[18,4]
   - 9.211753334860951 x[18,5] - 11.74762403007406 x[18,6]
   - 15.98149160227258 x[18,7] - 16.13397862940496 x[18,9]
   - 4.853134789680722 x[18,10] - 4.710069713599438 x[18,11]
   - 11.41770167394705 x[18,12] - 7.540827061018448 x[18,13]
   - 20.73515824930722 x[18,14] - 9.211753334860951 x[18,15]
   - 11.74762403007406 x[18,16] - 15.98149160227258 x[18,17]
   - 16.13397862940496 x[18,19] - 4.853134789680722 x[18,20]
   - 4.710069713599438 x[18,21] - 38.19142593817779 x[19,1]
   - 26.47140826342154 x[19,2] - 9.463208456001135 x[19,3]
   - 33.25745504625385 x[19,4] - 23.83231788009521 x[19,5]
   - 22.96638631428159 x[19,6] - 0.2812558625819943 x[19,7]
   - 16.13397862940496 x[19,8] - 20.88252519695769 x[19,10]
   - 17.32115145157977 x[19,11] - 26.47140826342154 x[19,12]
   - 9.463208456001135 x[19,13] - 33.25745504625385 x[19,14]
   - 23.83231788009521 x[19,15] - 22.96638631428159 x[19,16]
   - 0.2812558625819943 x[19,17] - 16.13397862940496 x[19,18]
   - 20.88252519695769 x[19,20] - 17.32115145157977 x[19,21]
   - 22.50443609709563 x[20,1] - 8.792742212244889 x[20,2]
   - 12.37117319681242 x[20,3] - 19.2080458878191 x[20,4]
   - 7.560198828112066 x[20,5] - 9.930996226454752 x[20,6]
   - 20.7427999599815 x[20,7] - 4.853134789680722 x[20,8]
   - 20.88252519695769 x[20,9] - 7.160688247122508 x[20,11]
   - 8.792742212244889 x[20,12] - 12.37117319681242 x[20,13]
   - 19.2080458878191 x[20,14] - 7.560198828112066 x[20,15]
   - 9.930996226454752 x[20,16] - 20.7427999599815 x[20,17]
   - 4.853134789680722 x[20,18] - 20.88252519695769 x[20,19]
   - 7.160688247122508 x[20,21] - 21.41481178272512 x[21,1]
   - 9.187100201837 x[21,2] - 7.859455227282641 x[21,3]
   - 16.91511424105545 x[21,4] - 6.518106014597915 x[21,5]
   - 16.09943949470468 x[21,6] - 17.11013246161761 x[21,7]
   - 4.710069713599438 x[21,8] - 17.32115145157977 x[21,9]
   - 7.160688247122508 x[21,10] - 9.187100201837 x[21,12]
   - 7.859455227282641 x[21,13] - 16.91511424105545 x[21,14]
   - 6.518106014597915 x[21,15] - 16.09943949470468 x[21,16]
   - 17.11013246161761 x[21,17] - 4.710069713599438 x[21,18]
   - 17.32115145157977 x[21,19] - 7.160688247122508 x[21,20]
   - 13.72686427008989 x[22,2] - 28.91928514842361 x[22,3]
   - 5.245539330012624 x[22,4] - 15.59056072128624 x[22,5]
   - 31.3510647010327 x[22,6] - 37.94903656105578 x[22,7]
   - 24.79559661029143 x[22,8] - 38.19142593817779 x[22,9]
   - 22.50443609709563 x[22,10] - 21.41481178272512 x[22,11]
   - 13.72686427008989 x[22,12] - 28.91928514842361 x[22,13]
   - 5.245539330012624 x[22,14] - 15.59056072128624 x[22,15]
   - 31.3510647010327 x[22,16] - 37.94903656105578 x[22,17]
   - 24.79559661029143 x[22,18] - 38.19142593817779 x[22,19]
   - 22.50443609709563 x[22,20] - 21.41481178272512 x[22,21]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[0]: - 41.18059281026967 x[0,2]
   - 86.75785544527083 x[0,3] - 15.73661799003787 x[0,4]
   - 46.77168216385873 x[0,5] - 94.05319410309809 x[0,6]
   - 113.8471096831673 x[0,7] - 74.38678983087431 x[0,8]
   - 114.5742778145334 x[0,9] - 67.5133082912869 x[0,10]
   - 64.24443534817536 x[0,11] - 41.18059281026967 x[0,12]
   - 86.75785544527083 x[0,13] - 15.73661799003787 x[0,14]
   - 46.77168216385873 x[0,15] - 94.05319410309809 x[0,16]
   - 113.8471096831673 x[0,17] - 74.38678983087431 x[0,18]
   - 114.5742778145334 x[0,19] - 67.5133082912869 x[0,20]
   - 64.24443534817536 x[0,21] - service_time[0] - start_service[0]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[1]: - 41.18059281026967 x[1,2]
   - 86.75785544527083 x[1,3] - 15.73661799003787 x[1,4]
   - 46.77168216385873 x[1,5] - 94.05319410309809 x[1,6]
   - 113.8471096831673 x[1,7] - 74.38678983087431 x[1,8]
   - 114.5742778145334 x[1,9] - 67.5133082912869 x[1,10]
   - 64.24443534817536 x[1,11] - 41.18059281026967 x[1,12]
   - 86.75785544527083 x[1,13] - 15.73661799003787 x[1,14]
   - 46.77168216385873 x[1,15] - 94.05319410309809 x[1,16]
   - 113.8471096831673 x[1,17] - 74.38678983087431 x[1,18]
   - 114.5742778145334 x[1,19] - 67.5133082912869 x[1,20]
   - 64.24443534817536 x[1,21] - service_time[1] - start_service[1]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[2]: - 41.18059281026967 x[2,1]
   - 51.0807426213442 x[2,3] - 31.79251450501173 x[2,4]
   - 8.114323750784228 x[2,5] - 54.30559077475615 x[2,6]
   - 78.80204308462216 x[2,7] - 34.25310502184114 x[2,8]
   - 79.41422479026464 x[2,9] - 26.37822663673467 x[2,10]
   - 27.561300605511 x[2,11] - 51.0807426213442 x[2,13]
   - 31.79251450501173 x[2,14] - 8.114323750784228 x[2,15]
   - 54.30559077475615 x[2,16] - 78.80204308462216 x[2,17]
   - 34.25310502184114 x[2,18] - 79.41422479026464 x[2,19]
   - 26.37822663673467 x[2,20] - 27.561300605511 x[2,21] - service_time[2]
   - start_service[2] + start_service[23] >= 0
 LowerBoundRoutingVehicle[3]: - 86.75785544527083 x[3,1]
   - 51.0807426213442 x[3,2] - 72.37402872388915 x[3,4]
   - 43.1242864050699 x[3,5] - 52.83892699153691 x[3,6]
   - 27.75232989456761 x[3,7] - 22.62248118305535 x[3,8]
   - 28.38962536800341 x[3,9] - 37.11351959043726 x[3,10]
   - 23.57836568184792 x[3,11] - 51.0807426213442 x[3,12]
   - 72.37402872388915 x[3,14] - 43.1242864050699 x[3,15]
   - 52.83892699153691 x[3,16] - 27.75232989456761 x[3,17]
   - 22.62248118305535 x[3,18] - 28.38962536800341 x[3,19]
   - 37.11351959043726 x[3,20] - 23.57836568184792 x[3,21]
   - service_time[3] - start_service[3] + start_service[23] >= 0
 LowerBoundRoutingVehicle[4]: - 15.73661799003787 x[4,1]
   - 31.79251450501173 x[4,2] - 72.37402872388915 x[4,3]
   - 35.33748440470753 x[4,5] - 86.09810223270773 x[4,6]
   - 99.02476159069988 x[4,7] - 62.20547474792166 x[4,8]
   - 99.77236513876157 x[4,9] - 57.6241376634573 x[4,10]
   - 50.74534272316635 x[4,11] - 31.79251450501173 x[4,12]
   - 72.37402872388915 x[4,13] - 35.33748440470753 x[4,15]
   - 86.09810223270773 x[4,16] - 99.02476159069988 x[4,17]
   - 62.20547474792166 x[4,18] - 99.77236513876157 x[4,19]
   - 57.6241376634573 x[4,20] - 50.74534272316635 x[4,21] - service_time[4]
   - start_service[4] + start_service[23] >= 0
 LowerBoundRoutingVehicle[5]: - 46.77168216385873 x[5,1]
   - 8.114323750784228 x[5,2] - 43.1242864050699 x[5,3]
   - 35.33748440470753 x[5,4] - 52.15802162513271 x[5,6]
   - 70.8714195782901 x[5,7] - 27.63526000458286 x[5,8]
   - 71.49695364028564 x[5,9] - 22.6805964843362 x[5,10]
   - 19.55431804379375 x[5,11] - 8.114323750784228 x[5,12]
   - 43.1242864050699 x[5,13] - 35.33748440470753 x[5,14]
   - 52.15802162513271 x[5,16] - 70.8714195782901 x[5,17]
   - 27.63526000458286 x[5,18] - 71.49695364028564 x[5,19]
   - 22.6805964843362 x[5,20] - 19.55431804379375 x[5,21] - service_time[5]
   - start_service[5] + start_service[23] >= 0
 LowerBoundRoutingVehicle[6]: - 94.05319410309809 x[6,1]
   - 54.30559077475615 x[6,2] - 52.83892699153691 x[6,3]
   - 86.09810223270773 x[6,4] - 52.15802162513271 x[6,5]
   - 68.83799981245924 x[6,7] - 35.24287209022218 x[6,8]
   - 68.89915894284476 x[6,9] - 29.79298867936426 x[6,10]
   - 48.29831848411406 x[6,11] - 54.30559077475615 x[6,12]
   - 52.83892699153691 x[6,13] - 86.09810223270773 x[6,14]
   - 52.15802162513271 x[6,15] - 68.83799981245924 x[6,17]
   - 35.24287209022218 x[6,18] - 68.89915894284476 x[6,19]
   - 29.79298867936426 x[6,20] - 48.29831848411406 x[6,21]
   - service_time[6] - start_service[6] + start_service[23] >= 0
 LowerBoundRoutingVehicle[7]: - 113.8471096831673 x[7,1]
   - 78.80204308462216 x[7,2] - 27.75232989456761 x[7,3]
   - 99.02476159069988 x[7,4] - 70.8714195782901 x[7,5]
   - 68.83799981245924 x[7,6] - 47.94447480681774 x[7,8]
   - 0.8437675877459829 x[7,9] - 62.22839987994452 x[7,10]
   - 51.33039738485282 x[7,11] - 78.80204308462216 x[7,12]
   - 27.75232989456761 x[7,13] - 99.02476159069988 x[7,14]
   - 70.8714195782901 x[7,15] - 68.83799981245924 x[7,16]
   - 47.94447480681774 x[7,18] - 0.8437675877459829 x[7,19]
   - 62.22839987994452 x[7,20] - 51.33039738485282 x[7,21]
   - service_time[7] - start_service[7] + start_service[23] >= 0
 LowerBoundRoutingVehicle[8]: - 74.38678983087431 x[8,1]
   - 34.25310502184114 x[8,2] - 22.62248118305535 x[8,3]
   - 62.20547474792166 x[8,4] - 27.63526000458286 x[8,5]
   - 35.24287209022218 x[8,6] - 47.94447480681774 x[8,7]
   - 48.40193588821488 x[8,9] - 14.55940436904216 x[8,10]
   - 14.13020914079832 x[8,11] - 34.25310502184114 x[8,12]
   - 22.62248118305535 x[8,13] - 62.20547474792166 x[8,14]
   - 27.63526000458286 x[8,15] - 35.24287209022218 x[8,16]
   - 47.94447480681774 x[8,17] - 48.40193588821488 x[8,19]
   - 14.55940436904216 x[8,20] - 14.13020914079832 x[8,21]
   - service_time[8] - start_service[8] + start_service[23] >= 0
 LowerBoundRoutingVehicle[9]: - 114.5742778145334 x[9,1]
   - 79.41422479026464 x[9,2] - 28.38962536800341 x[9,3]
   - 99.77236513876157 x[9,4] - 71.49695364028564 x[9,5]
   - 68.89915894284476 x[9,6] - 0.8437675877459829 x[9,7]
   - 48.40193588821488 x[9,8] - 62.64757559087306 x[9,10]
   - 51.96345435473931 x[9,11] - 79.41422479026464 x[9,12]
   - 28.38962536800341 x[9,13] - 99.77236513876157 x[9,14]
   - 71.49695364028564 x[9,15] - 68.89915894284476 x[9,16]
   - 0.8437675877459829 x[9,17] - 48.40193588821488 x[9,18]
   - 62.64757559087306 x[9,20] - 51.96345435473931 x[9,21]
   - service_time[9] - start_service[9] + start_service[23] >= 0
 LowerBoundRoutingVehicle[10]: - 67.5133082912869 x[10,1]
   - 26.37822663673467 x[10,2] - 37.11351959043726 x[10,3]
   - 57.6241376634573 x[10,4] - 22.6805964843362 x[10,5]
   - 29.79298867936426 x[10,6] - 62.22839987994452 x[10,7]
   - 14.55940436904216 x[10,8] - 62.64757559087306 x[10,9]
   - 21.48206474136752 x[10,11] - 26.37822663673467 x[10,12]
   - 37.11351959043726 x[10,13] - 57.6241376634573 x[10,14]
   - 22.6805964843362 x[10,15] - 29.79298867936426 x[10,16]
   - 62.22839987994452 x[10,17] - 14.55940436904216 x[10,18]
   - 62.64757559087306 x[10,19] - 21.48206474136752 x[10,21]
   - service_time[10] - start_service[10] + start_service[23] >= 0
 LowerBoundRoutingVehicle[11]: - 64.24443534817536 x[11,1]
   - 27.561300605511 x[11,2] - 23.57836568184792 x[11,3]
   - 50.74534272316635 x[11,4] - 19.55431804379375 x[11,5]
   - 48.29831848411406 x[11,6] - 51.33039738485282 x[11,7]
   - 14.13020914079832 x[11,8] - 51.96345435473931 x[11,9]
   - 21.48206474136752 x[11,10] - 27.561300605511 x[11,12]
   - 23.57836568184792 x[11,13] - 50.74534272316635 x[11,14]
   - 19.55431804379375 x[11,15] - 48.29831848411406 x[11,16]
   - 51.33039738485282 x[11,17] - 14.13020914079832 x[11,18]
   - 51.96345435473931 x[11,19] - 21.48206474136752 x[11,20]
   - service_time[11] - start_service[11] + start_service[23] >= 0
 LowerBoundRoutingVehicle[12]: - 41.18059281026967 x[12,1]
   - 51.0807426213442 x[12,3] - 31.79251450501173 x[12,4]
   - 8.114323750784228 x[12,5] - 54.30559077475615 x[12,6]
   - 78.80204308462216 x[12,7] - 34.25310502184114 x[12,8]
   - 79.41422479026464 x[12,9] - 26.37822663673467 x[12,10]
   - 27.561300605511 x[12,11] - 51.0807426213442 x[12,13]
   - 31.79251450501173 x[12,14] - 8.114323750784228 x[12,15]
   - 54.30559077475615 x[12,16] - 78.80204308462216 x[12,17]
   - 34.25310502184114 x[12,18] - 79.41422479026464 x[12,19]
   - 26.37822663673467 x[12,20] - 27.561300605511 x[12,21]
   - service_time[12] - start_service[12] + start_service[23] >= 0
 LowerBoundRoutingVehicle[13]: - 86.75785544527083 x[13,1]
   - 51.0807426213442 x[13,2] - 72.37402872388915 x[13,4]
   - 43.1242864050699 x[13,5] - 52.83892699153691 x[13,6]
   - 27.75232989456761 x[13,7] - 22.62248118305535 x[13,8]
   - 28.38962536800341 x[13,9] - 37.11351959043726 x[13,10]
   - 23.57836568184792 x[13,11] - 51.0807426213442 x[13,12]
   - 72.37402872388915 x[13,14] - 43.1242864050699 x[13,15]
   - 52.83892699153691 x[13,16] - 27.75232989456761 x[13,17]
   - 22.62248118305535 x[13,18] - 28.38962536800341 x[13,19]
   - 37.11351959043726 x[13,20] - 23.57836568184792 x[13,21]
   - service_time[13] - start_service[13] + start_service[23] >= 0
 LowerBoundRoutingVehicle[14]: - 15.73661799003787 x[14,1]
   - 31.79251450501173 x[14,2] - 72.37402872388915 x[14,3]
   - 35.33748440470753 x[14,5] - 86.09810223270773 x[14,6]
   - 99.02476159069988 x[14,7] - 62.20547474792166 x[14,8]
   - 99.77236513876157 x[14,9] - 57.6241376634573 x[14,10]
   - 50.74534272316635 x[14,11] - 31.79251450501173 x[14,12]
   - 72.37402872388915 x[14,13] - 35.33748440470753 x[14,15]
   - 86.09810223270773 x[14,16] - 99.02476159069988 x[14,17]
   - 62.20547474792166 x[14,18] - 99.77236513876157 x[14,19]
   - 57.6241376634573 x[14,20] - 50.74534272316635 x[14,21]
   - service_time[14] - start_service[14] + start_service[23] >= 0
 LowerBoundRoutingVehicle[15]: - 46.77168216385873 x[15,1]
   - 8.114323750784228 x[15,2] - 43.1242864050699 x[15,3]
   - 35.33748440470753 x[15,4] - 52.15802162513271 x[15,6]
   - 70.8714195782901 x[15,7] - 27.63526000458286 x[15,8]
   - 71.49695364028564 x[15,9] - 22.6805964843362 x[15,10]
   - 19.55431804379375 x[15,11] - 8.114323750784228 x[15,12]
   - 43.1242864050699 x[15,13] - 35.33748440470753 x[15,14]
   - 52.15802162513271 x[15,16] - 70.8714195782901 x[15,17]
   - 27.63526000458286 x[15,18] - 71.49695364028564 x[15,19]
   - 22.6805964843362 x[15,20] - 19.55431804379375 x[15,21]
   - service_time[15] - start_service[15] + start_service[23] >= 0
 LowerBoundRoutingVehicle[16]: - 94.05319410309809 x[16,1]
   - 54.30559077475615 x[16,2] - 52.83892699153691 x[16,3]
   - 86.09810223270773 x[16,4] - 52.15802162513271 x[16,5]
   - 68.83799981245924 x[16,7] - 35.24287209022218 x[16,8]
   - 68.89915894284476 x[16,9] - 29.79298867936426 x[16,10]
   - 48.29831848411406 x[16,11] - 54.30559077475615 x[16,12]
   - 52.83892699153691 x[16,13] - 86.09810223270773 x[16,14]
   - 52.15802162513271 x[16,15] - 68.83799981245924 x[16,17]
   - 35.24287209022218 x[16,18] - 68.89915894284476 x[16,19]
   - 29.79298867936426 x[16,20] - 48.29831848411406 x[16,21]
   - service_time[16] - start_service[16] + start_service[23] >= 0
 LowerBoundRoutingVehicle[17]: - 113.8471096831673 x[17,1]
   - 78.80204308462216 x[17,2] - 27.75232989456761 x[17,3]
   - 99.02476159069988 x[17,4] - 70.8714195782901 x[17,5]
   - 68.83799981245924 x[17,6] - 47.94447480681774 x[17,8]
   - 0.8437675877459829 x[17,9] - 62.22839987994452 x[17,10]
   - 51.33039738485282 x[17,11] - 78.80204308462216 x[17,12]
   - 27.75232989456761 x[17,13] - 99.02476159069988 x[17,14]
   - 70.8714195782901 x[17,15] - 68.83799981245924 x[17,16]
   - 47.94447480681774 x[17,18] - 0.8437675877459829 x[17,19]
   - 62.22839987994452 x[17,20] - 51.33039738485282 x[17,21]
   - service_time[17] - start_service[17] + start_service[23] >= 0
 LowerBoundRoutingVehicle[18]: - 74.38678983087431 x[18,1]
   - 34.25310502184114 x[18,2] - 22.62248118305535 x[18,3]
   - 62.20547474792166 x[18,4] - 27.63526000458286 x[18,5]
   - 35.24287209022218 x[18,6] - 47.94447480681774 x[18,7]
   - 48.40193588821488 x[18,9] - 14.55940436904216 x[18,10]
   - 14.13020914079832 x[18,11] - 34.25310502184114 x[18,12]
   - 22.62248118305535 x[18,13] - 62.20547474792166 x[18,14]
   - 27.63526000458286 x[18,15] - 35.24287209022218 x[18,16]
   - 47.94447480681774 x[18,17] - 48.40193588821488 x[18,19]
   - 14.55940436904216 x[18,20] - 14.13020914079832 x[18,21]
   - service_time[18] - start_service[18] + start_service[23] >= 0
 LowerBoundRoutingVehicle[19]: - 114.5742778145334 x[19,1]
   - 79.41422479026464 x[19,2] - 28.38962536800341 x[19,3]
   - 99.77236513876157 x[19,4] - 71.49695364028564 x[19,5]
   - 68.89915894284476 x[19,6] - 0.8437675877459829 x[19,7]
   - 48.40193588821488 x[19,8] - 62.64757559087306 x[19,10]
   - 51.96345435473931 x[19,11] - 79.41422479026464 x[19,12]
   - 28.38962536800341 x[19,13] - 99.77236513876157 x[19,14]
   - 71.49695364028564 x[19,15] - 68.89915894284476 x[19,16]
   - 0.8437675877459829 x[19,17] - 48.40193588821488 x[19,18]
   - 62.64757559087306 x[19,20] - 51.96345435473931 x[19,21]
   - service_time[19] - start_service[19] + start_service[23] >= 0
 LowerBoundRoutingVehicle[20]: - 67.5133082912869 x[20,1]
   - 26.37822663673467 x[20,2] - 37.11351959043726 x[20,3]
   - 57.6241376634573 x[20,4] - 22.6805964843362 x[20,5]
   - 29.79298867936426 x[20,6] - 62.22839987994452 x[20,7]
   - 14.55940436904216 x[20,8] - 62.64757559087306 x[20,9]
   - 21.48206474136752 x[20,11] - 26.37822663673467 x[20,12]
   - 37.11351959043726 x[20,13] - 57.6241376634573 x[20,14]
   - 22.6805964843362 x[20,15] - 29.79298867936426 x[20,16]
   - 62.22839987994452 x[20,17] - 14.55940436904216 x[20,18]
   - 62.64757559087306 x[20,19] - 21.48206474136752 x[20,21]
   - service_time[20] - start_service[20] + start_service[23] >= 0
 LowerBoundRoutingVehicle[21]: - 64.24443534817536 x[21,1]
   - 27.561300605511 x[21,2] - 23.57836568184792 x[21,3]
   - 50.74534272316635 x[21,4] - 19.55431804379375 x[21,5]
   - 48.29831848411406 x[21,6] - 51.33039738485282 x[21,7]
   - 14.13020914079832 x[21,8] - 51.96345435473931 x[21,9]
   - 21.48206474136752 x[21,10] - 27.561300605511 x[21,12]
   - 23.57836568184792 x[21,13] - 50.74534272316635 x[21,14]
   - 19.55431804379375 x[21,15] - 48.29831848411406 x[21,16]
   - 51.33039738485282 x[21,17] - 14.13020914079832 x[21,18]
   - 51.96345435473931 x[21,19] - 21.48206474136752 x[21,20]
   - service_time[21] - start_service[21] + start_service[23] >= 0
 LowerBoundRoutingVehicle[22]: - 41.18059281026967 x[22,2]
   - 86.75785544527083 x[22,3] - 15.73661799003787 x[22,4]
   - 46.77168216385873 x[22,5] - 94.05319410309809 x[22,6]
   - 113.8471096831673 x[22,7] - 74.38678983087431 x[22,8]
   - 114.5742778145334 x[22,9] - 67.5133082912869 x[22,10]
   - 64.24443534817536 x[22,11] - 41.18059281026967 x[22,12]
   - 86.75785544527083 x[22,13] - 15.73661799003787 x[22,14]
   - 46.77168216385873 x[22,15] - 94.05319410309809 x[22,16]
   - 113.8471096831673 x[22,17] - 74.38678983087431 x[22,18]
   - 114.5742778145334 x[22,19] - 67.5133082912869 x[22,20]
   - 64.24443534817536 x[22,21] - service_time[22] - start_service[22]
   + start_service[23] >= 0
 LowerBoundArrivalVehicle[1]: - 82.36118562053935 x[2,1]
   - 173.5157108905417 x[3,1] - 31.47323598007575 x[4,1]
   - 93.54336432771747 x[5,1] - 188.1063882061962 x[6,1]
   - 227.6942193663347 x[7,1] - 148.7735796617486 x[8,1]
   - 229.1485556290668 x[9,1] - 135.0266165825738 x[10,1]
   - 128.4888706963507 x[11,1] - 82.36118562053935 x[12,1]
   - 173.5157108905417 x[13,1] - 31.47323598007575 x[14,1]
   - 93.54336432771747 x[15,1] - 188.1063882061962 x[16,1]
   - 227.6942193663347 x[17,1] - 148.7735796617486 x[18,1]
   - 229.1485556290668 x[19,1] - 135.0266165825738 x[20,1]
   - 128.4888706963507 x[21,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 41.18059281026967 x[0,2]
   - 41.18059281026967 x[1,2] - 41.18059281026967 x[2,2]
   - 137.838598066615 x[3,2] - 47.52913249504961 x[4,2]
   - 54.88600591464296 x[5,2] - 148.3587848778542 x[6,2]
   - 192.6491527677895 x[7,2] - 108.6398948527155 x[8,2]
   - 193.988502604798 x[9,2] - 93.89153492802157 x[10,2]
   - 91.80573595368637 x[11,2] - 41.18059281026967 x[12,2]
   - 137.838598066615 x[13,2] - 47.52913249504961 x[14,2]
   - 54.88600591464296 x[15,2] - 148.3587848778542 x[16,2]
   - 192.6491527677895 x[17,2] - 108.6398948527155 x[18,2]
   - 193.988502604798 x[19,2] - 93.89153492802157 x[20,2]
   - 91.80573595368637 x[21,2] - 41.18059281026967 x[22,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 86.75785544527083 x[0,3]
   - 86.75785544527083 x[1,3] - 92.26133543161387 x[2,3]
   - 86.75785544527083 x[3,3] - 88.11064671392703 x[4,3]
   - 89.89596856892864 x[5,3] - 146.892121094635 x[6,3]
   - 141.599439577735 x[7,3] - 97.00927101392965 x[8,3]
   - 142.9639031825368 x[9,3] - 104.6268278817242 x[10,3]
   - 87.82280103002329 x[11,3] - 92.26133543161387 x[12,3]
   - 86.75785544527083 x[13,3] - 88.11064671392703 x[14,3]
   - 89.89596856892864 x[15,3] - 146.892121094635 x[16,3]
   - 141.599439577735 x[17,3] - 97.00927101392965 x[18,3]
   - 142.9639031825368 x[19,3] - 104.6268278817242 x[20,3]
   - 87.82280103002329 x[21,3] - 86.75785544527083 x[22,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 15.73661799003787 x[0,4]
   - 15.73661799003787 x[1,4] - 72.9731073152814 x[2,4]
   - 159.13188416916 x[3,4] - 15.73661799003787 x[4,4]
   - 82.10916656856627 x[5,4] - 180.1512963358058 x[6,4]
   - 212.8718712738672 x[7,4] - 136.592264578796 x[8,4]
   - 214.346642953295 x[9,4] - 125.1374459547442 x[10,4]
   - 114.9897780713417 x[11,4] - 72.9731073152814 x[12,4]
   - 159.13188416916 x[13,4] - 15.73661799003787 x[14,4]
   - 82.10916656856627 x[15,4] - 180.1512963358058 x[16,4]
   - 212.8718712738672 x[17,4] - 136.592264578796 x[18,4]
   - 214.346642953295 x[19,4] - 125.1374459547442 x[20,4]
   - 114.9897780713417 x[21,4] - 15.73661799003787 x[22,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 46.77168216385873 x[0,5]
   - 46.77168216385873 x[1,5] - 49.2949165610539 x[2,5]
   - 129.8821418503407 x[3,5] - 51.07410239474541 x[4,5]
   - 46.77168216385873 x[5,5] - 146.2112157282308 x[6,5]
   - 184.7185292614574 x[7,5] - 102.0220498354572 x[8,5]
   - 186.071231454819 x[9,5] - 90.19390477562311 x[10,5]
   - 83.7987533919691 x[11,5] - 49.2949165610539 x[12,5]
   - 129.8821418503407 x[13,5] - 51.07410239474541 x[14,5]
   - 46.77168216385873 x[15,5] - 146.2112157282308 x[16,5]
   - 184.7185292614574 x[17,5] - 102.0220498354572 x[18,5]
   - 186.071231454819 x[19,5] - 90.19390477562311 x[20,5]
   - 83.7987533919691 x[21,5] - 46.77168216385873 x[22,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 94.05319410309809 x[0,6]
   - 94.05319410309809 x[1,6] - 95.48618358502583 x[2,6]
   - 139.5967824368078 x[3,6] - 101.8347202227456 x[4,6]
   - 98.92970378899145 x[5,6] - 94.05319410309809 x[6,6]
   - 182.6851094956266 x[7,6] - 109.6296619210965 x[8,6]
   - 183.4734367573782 x[9,6] - 97.30629697065116 x[10,6]
   - 112.5427538322894 x[11,6] - 95.48618358502583 x[12,6]
   - 139.5967824368078 x[13,6] - 101.8347202227456 x[14,6]
   - 98.92970378899145 x[15,6] - 94.05319410309809 x[16,6]
   - 182.6851094956266 x[17,6] - 109.6296619210965 x[18,6]
   - 183.4734367573782 x[19,6] - 97.30629697065116 x[20,6]
   - 112.5427538322894 x[21,6] - 94.05319410309809 x[22,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 113.8471096831673 x[0,7]
   - 113.8471096831673 x[1,7] - 119.9826358948918 x[2,7]
   - 114.5101853398384 x[3,7] - 114.7613795807378 x[4,7]
   - 117.6431017421488 x[5,7] - 162.8911939155573 x[6,7]
   - 113.8471096831673 x[7,7] - 122.331264637692 x[8,7]
   - 115.4180454022794 x[9,7] - 129.7417081712314 x[10,7]
   - 115.5748327330282 x[11,7] - 119.9826358948918 x[12,7]
   - 114.5101853398384 x[13,7] - 114.7613795807378 x[14,7]
   - 117.6431017421488 x[15,7] - 162.8911939155573 x[16,7]
   - 113.8471096831673 x[17,7] - 122.331264637692 x[18,7]
   - 115.4180454022794 x[19,7] - 129.7417081712314 x[20,7]
   - 115.5748327330282 x[21,7] - 113.8471096831673 x[22,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 74.38678983087431 x[0,8]
   - 74.38678983087431 x[1,8] - 75.43369783211082 x[2,8]
   - 109.3803366283262 x[3,8] - 77.94209273795954 x[4,8]
   - 74.40694216844159 x[5,8] - 129.2960661933203 x[6,8]
   - 161.7915844899851 x[7,8] - 74.38678983087431 x[8,8]
   - 162.9762137027483 x[9,8] - 82.07271266032907 x[10,8]
   - 78.37464448897367 x[11,8] - 75.43369783211082 x[12,8]
   - 109.3803366283262 x[13,8] - 77.94209273795954 x[14,8]
   - 74.40694216844159 x[15,8] - 129.2960661933203 x[16,8]
   - 161.7915844899851 x[17,8] - 74.38678983087431 x[18,8]
   - 162.9762137027483 x[19,8] - 82.07271266032907 x[20,8]
   - 78.37464448897367 x[21,8] - 74.38678983087431 x[22,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 114.5742778145334 x[0,9]
   - 114.5742778145334 x[1,9] - 120.5948176005343 x[2,9]
   - 115.1474808132742 x[3,9] - 115.5089831287994 x[4,9]
   - 118.2686358041444 x[5,9] - 162.9523530459429 x[6,9]
   - 114.6908772709133 x[7,9] - 122.7887257190892 x[8,9]
   - 114.5742778145334 x[9,9] - 130.16088388216 x[10,9]
   - 116.2078897029147 x[11,9] - 120.5948176005343 x[12,9]
   - 115.1474808132742 x[13,9] - 115.5089831287994 x[14,9]
   - 118.2686358041444 x[15,9] - 162.9523530459429 x[16,9]
   - 114.6908772709133 x[17,9] - 122.7887257190892 x[18,9]
   - 114.5742778145334 x[19,9] - 130.16088388216 x[20,9]
   - 116.2078897029147 x[21,9] - 114.5742778145334 x[22,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 67.5133082912869 x[0,10]
   - 67.5133082912869 x[1,10] - 67.55881944700434 x[2,10]
   - 123.8713750357081 x[3,10] - 73.36075565349518 x[4,10]
   - 69.45227864819493 x[5,10] - 123.8461827824623 x[6,10]
   - 176.0755095631118 x[7,10] - 88.94619419991648 x[8,10]
   - 177.2218534054064 x[9,10] - 67.5133082912869 x[10,10]
   - 85.72650008954288 x[11,10] - 67.55881944700434 x[12,10]
   - 123.8713750357081 x[13,10] - 73.36075565349518 x[14,10]
   - 69.45227864819493 x[15,10] - 123.8461827824623 x[16,10]
   - 176.0755095631118 x[17,10] - 88.94619419991648 x[18,10]
   - 177.2218534054064 x[19,10] - 67.5133082912869 x[20,10]
   - 85.72650008954288 x[21,10] - 67.5133082912869 x[22,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 64.24443534817536 x[0,11]
   - 64.24443534817536 x[1,11] - 68.74189341578068 x[2,11]
   - 110.3362211271188 x[3,11] - 66.48196071320422 x[4,11]
   - 66.32600020765248 x[5,11] - 142.3515125872121 x[6,11]
   - 165.1775070680202 x[7,11] - 88.51699897167262 x[8,11]
   - 166.5377321692727 x[9,11] - 88.99537303265443 x[10,11]
   - 64.24443534817536 x[11,11] - 68.74189341578068 x[12,11]
   - 110.3362211271188 x[13,11] - 66.48196071320422 x[14,11]
   - 66.32600020765248 x[15,11] - 142.3515125872121 x[16,11]
   - 165.1775070680202 x[17,11] - 88.51699897167262 x[18,11]
   - 166.5377321692727 x[19,11] - 88.99537303265443 x[20,11]
   - 64.24443534817536 x[21,11] - 64.24443534817536 x[22,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 41.18059281026967 x[0,12]
   - 41.18059281026967 x[1,12] - 41.18059281026967 x[2,12]
   - 137.838598066615 x[3,12] - 47.52913249504961 x[4,12]
   - 54.88600591464296 x[5,12] - 148.3587848778542 x[6,12]
   - 192.6491527677895 x[7,12] - 108.6398948527155 x[8,12]
   - 193.988502604798 x[9,12] - 93.89153492802157 x[10,12]
   - 91.80573595368637 x[11,12] - 41.18059281026967 x[12,12]
   - 137.838598066615 x[13,12] - 47.52913249504961 x[14,12]
   - 54.88600591464296 x[15,12] - 148.3587848778542 x[16,12]
   - 192.6491527677895 x[17,12] - 108.6398948527155 x[18,12]
   - 193.988502604798 x[19,12] - 93.89153492802157 x[20,12]
   - 91.80573595368637 x[21,12] - 41.18059281026967 x[22,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 86.75785544527083 x[0,13]
   - 86.75785544527083 x[1,13] - 92.26133543161387 x[2,13]
   - 86.75785544527083 x[3,13] - 88.11064671392703 x[4,13]
   - 89.89596856892864 x[5,13] - 146.892121094635 x[6,13]
   - 141.599439577735 x[7,13] - 97.00927101392965 x[8,13]
   - 142.9639031825368 x[9,13] - 104.6268278817242 x[10,13]
   - 87.82280103002329 x[11,13] - 92.26133543161387 x[12,13]
   - 86.75785544527083 x[13,13] - 88.11064671392703 x[14,13]
   - 89.89596856892864 x[15,13] - 146.892121094635 x[16,13]
   - 141.599439577735 x[17,13] - 97.00927101392965 x[18,13]
   - 142.9639031825368 x[19,13] - 104.6268278817242 x[20,13]
   - 87.82280103002329 x[21,13] - 86.75785544527083 x[22,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 15.73661799003787 x[0,14]
   - 15.73661799003787 x[1,14] - 72.9731073152814 x[2,14]
   - 159.13188416916 x[3,14] - 15.73661799003787 x[4,14]
   - 82.10916656856627 x[5,14] - 180.1512963358058 x[6,14]
   - 212.8718712738672 x[7,14] - 136.592264578796 x[8,14]
   - 214.346642953295 x[9,14] - 125.1374459547442 x[10,14]
   - 114.9897780713417 x[11,14] - 72.9731073152814 x[12,14]
   - 159.13188416916 x[13,14] - 15.73661799003787 x[14,14]
   - 82.10916656856627 x[15,14] - 180.1512963358058 x[16,14]
   - 212.8718712738672 x[17,14] - 136.592264578796 x[18,14]
   - 214.346642953295 x[19,14] - 125.1374459547442 x[20,14]
   - 114.9897780713417 x[21,14] - 15.73661799003787 x[22,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 46.77168216385873 x[0,15]
   - 46.77168216385873 x[1,15] - 49.2949165610539 x[2,15]
   - 129.8821418503407 x[3,15] - 51.07410239474541 x[4,15]
   - 46.77168216385873 x[5,15] - 146.2112157282308 x[6,15]
   - 184.7185292614574 x[7,15] - 102.0220498354572 x[8,15]
   - 186.071231454819 x[9,15] - 90.19390477562311 x[10,15]
   - 83.7987533919691 x[11,15] - 49.2949165610539 x[12,15]
   - 129.8821418503407 x[13,15] - 51.07410239474541 x[14,15]
   - 46.77168216385873 x[15,15] - 146.2112157282308 x[16,15]
   - 184.7185292614574 x[17,15] - 102.0220498354572 x[18,15]
   - 186.071231454819 x[19,15] - 90.19390477562311 x[20,15]
   - 83.7987533919691 x[21,15] - 46.77168216385873 x[22,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 94.05319410309809 x[0,16]
   - 94.05319410309809 x[1,16] - 95.48618358502583 x[2,16]
   - 139.5967824368078 x[3,16] - 101.8347202227456 x[4,16]
   - 98.92970378899145 x[5,16] - 94.05319410309809 x[6,16]
   - 182.6851094956266 x[7,16] - 109.6296619210965 x[8,16]
   - 183.4734367573782 x[9,16] - 97.30629697065116 x[10,16]
   - 112.5427538322894 x[11,16] - 95.48618358502583 x[12,16]
   - 139.5967824368078 x[13,16] - 101.8347202227456 x[14,16]
   - 98.92970378899145 x[15,16] - 94.05319410309809 x[16,16]
   - 182.6851094956266 x[17,16] - 109.6296619210965 x[18,16]
   - 183.4734367573782 x[19,16] - 97.30629697065116 x[20,16]
   - 112.5427538322894 x[21,16] - 94.05319410309809 x[22,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 113.8471096831673 x[0,17]
   - 113.8471096831673 x[1,17] - 119.9826358948918 x[2,17]
   - 114.5101853398384 x[3,17] - 114.7613795807378 x[4,17]
   - 117.6431017421488 x[5,17] - 162.8911939155573 x[6,17]
   - 113.8471096831673 x[7,17] - 122.331264637692 x[8,17]
   - 115.4180454022794 x[9,17] - 129.7417081712314 x[10,17]
   - 115.5748327330282 x[11,17] - 119.9826358948918 x[12,17]
   - 114.5101853398384 x[13,17] - 114.7613795807378 x[14,17]
   - 117.6431017421488 x[15,17] - 162.8911939155573 x[16,17]
   - 113.8471096831673 x[17,17] - 122.331264637692 x[18,17]
   - 115.4180454022794 x[19,17] - 129.7417081712314 x[20,17]
   - 115.5748327330282 x[21,17] - 113.8471096831673 x[22,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 74.38678983087431 x[0,18]
   - 74.38678983087431 x[1,18] - 75.43369783211082 x[2,18]
   - 109.3803366283262 x[3,18] - 77.94209273795954 x[4,18]
   - 74.40694216844159 x[5,18] - 129.2960661933203 x[6,18]
   - 161.7915844899851 x[7,18] - 74.38678983087431 x[8,18]
   - 162.9762137027483 x[9,18] - 82.07271266032907 x[10,18]
   - 78.37464448897367 x[11,18] - 75.43369783211082 x[12,18]
   - 109.3803366283262 x[13,18] - 77.94209273795954 x[14,18]
   - 74.40694216844159 x[15,18] - 129.2960661933203 x[16,18]
   - 161.7915844899851 x[17,18] - 74.38678983087431 x[18,18]
   - 162.9762137027483 x[19,18] - 82.07271266032907 x[20,18]
   - 78.37464448897367 x[21,18] - 74.38678983087431 x[22,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 114.5742778145334 x[0,19]
   - 114.5742778145334 x[1,19] - 120.5948176005343 x[2,19]
   - 115.1474808132742 x[3,19] - 115.5089831287994 x[4,19]
   - 118.2686358041444 x[5,19] - 162.9523530459429 x[6,19]
   - 114.6908772709133 x[7,19] - 122.7887257190892 x[8,19]
   - 114.5742778145334 x[9,19] - 130.16088388216 x[10,19]
   - 116.2078897029147 x[11,19] - 120.5948176005343 x[12,19]
   - 115.1474808132742 x[13,19] - 115.5089831287994 x[14,19]
   - 118.2686358041444 x[15,19] - 162.9523530459429 x[16,19]
   - 114.6908772709133 x[17,19] - 122.7887257190892 x[18,19]
   - 114.5742778145334 x[19,19] - 130.16088388216 x[20,19]
   - 116.2078897029147 x[21,19] - 114.5742778145334 x[22,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 67.5133082912869 x[0,20]
   - 67.5133082912869 x[1,20] - 67.55881944700434 x[2,20]
   - 123.8713750357081 x[3,20] - 73.36075565349518 x[4,20]
   - 69.45227864819493 x[5,20] - 123.8461827824623 x[6,20]
   - 176.0755095631118 x[7,20] - 88.94619419991648 x[8,20]
   - 177.2218534054064 x[9,20] - 67.5133082912869 x[10,20]
   - 85.72650008954288 x[11,20] - 67.55881944700434 x[12,20]
   - 123.8713750357081 x[13,20] - 73.36075565349518 x[14,20]
   - 69.45227864819493 x[15,20] - 123.8461827824623 x[16,20]
   - 176.0755095631118 x[17,20] - 88.94619419991648 x[18,20]
   - 177.2218534054064 x[19,20] - 67.5133082912869 x[20,20]
   - 85.72650008954288 x[21,20] - 67.5133082912869 x[22,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 64.24443534817536 x[0,21]
   - 64.24443534817536 x[1,21] - 68.74189341578068 x[2,21]
   - 110.3362211271188 x[3,21] - 66.48196071320422 x[4,21]
   - 66.32600020765248 x[5,21] - 142.3515125872121 x[6,21]
   - 165.1775070680202 x[7,21] - 88.51699897167262 x[8,21]
   - 166.5377321692727 x[9,21] - 88.99537303265443 x[10,21]
   - 64.24443534817536 x[11,21] - 68.74189341578068 x[12,21]
   - 110.3362211271188 x[13,21] - 66.48196071320422 x[14,21]
   - 66.32600020765248 x[15,21] - 142.3515125872121 x[16,21]
   - 165.1775070680202 x[17,21] - 88.51699897167262 x[18,21]
   - 166.5377321692727 x[19,21] - 88.99537303265443 x[20,21]
   - 64.24443534817536 x[21,21] - 64.24443534817536 x[22,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 41.18059281026967 x[2,22]
   - 86.75785544527083 x[3,22] - 15.73661799003787 x[4,22]
   - 46.77168216385873 x[5,22] - 94.05319410309809 x[6,22]
   - 113.8471096831673 x[7,22] - 74.38678983087431 x[8,22]
   - 114.5742778145334 x[9,22] - 67.5133082912869 x[10,22]
   - 64.24443534817536 x[11,22] - 41.18059281026967 x[12,22]
   - 86.75785544527083 x[13,22] - 15.73661799003787 x[14,22]
   - 46.77168216385873 x[15,22] - 94.05319410309809 x[16,22]
   - 113.8471096831673 x[17,22] - 74.38678983087431 x[18,22]
   - 114.5742778145334 x[19,22] - 67.5133082912869 x[20,22]
   - 64.24443534817536 x[21,22] - start_service[0] + start_service[22]
   >= 0
 LowerBoundArrivalVehicle[23]: - 41.18059281026967 x[2,23]
   - 86.75785544527083 x[3,23] - 15.73661799003787 x[4,23]
   - 46.77168216385873 x[5,23] - 94.05319410309809 x[6,23]
   - 113.8471096831673 x[7,23] - 74.38678983087431 x[8,23]
   - 114.5742778145334 x[9,23] - 67.5133082912869 x[10,23]
   - 64.24443534817536 x[11,23] - 41.18059281026967 x[12,23]
   - 86.75785544527083 x[13,23] - 15.73661799003787 x[14,23]
   - 46.77168216385873 x[15,23] - 94.05319410309809 x[16,23]
   - 113.8471096831673 x[17,23] - 74.38678983087431 x[18,23]
   - 114.5742778145334 x[19,23] - 67.5133082912869 x[20,23]
   - 64.24443534817536 x[21,23] - start_service[0] + start_service[23]
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
