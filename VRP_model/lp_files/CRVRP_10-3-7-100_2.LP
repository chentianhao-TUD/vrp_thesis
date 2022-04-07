\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[23,0] + 0 x[23,1] + 0 x[23,2] + 0 x[23,3] + 0 x[23,4]
   + 0 x[23,5] + 0 x[23,6] + 0 x[23,7] + 0 x[23,8] + 0 x[23,9] + 0 x[23,10]
   + 0 x[23,11] + 0 x[23,12] + 0 x[23,13] + 0 x[23,14] + 0 x[23,15]
   + 0 x[23,16] + 0 x[23,17] + 0 x[23,18] + 0 x[23,19] + 0 x[23,20]
   + 0 x[23,21] + 0 x[23,22] + 0 x[23,23] + 0 w[0,0] + 0 w[23,0]
   + 0 w[23,1] + 0 w[23,22] + 0 w[23,23] + 0 v[0,0] + 0 v[23,0] + 0 v[23,1]
   + 0 v[23,2] + 0 v[23,3] + 0 v[23,4] + 0 v[23,5] + 0 v[23,6] + 0 v[23,7]
   + 0 v[23,8] + 0 v[23,9] + 0 v[23,10] + 0 v[23,11] + 0 v[23,12]
   + 0 v[23,13] + 0 v[23,14] + 0 v[23,15] + 0 v[23,16] + 0 v[23,17]
   + 0 v[23,18] + 0 v[23,19] + 0 v[23,20] + 0 v[23,21] + 0 v[23,22]
   + 0 v[23,23] + 0 service_time[23] + start_service[23]
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
   + start_service[2] >= -999970.4355137392
 NodeArrivalVehicles[0,3]: - 999999 x[0,3] - start_service[0]
   + start_service[3] >= -999977.0287494746
 NodeArrivalVehicles[0,4]: - 999999 x[0,4] - start_service[0]
   + start_service[4] >= -999994.052853037
 NodeArrivalVehicles[0,5]: - 999999 x[0,5] - start_service[0]
   + start_service[5] >= -999930.4872109075
 NodeArrivalVehicles[0,6]: - 999999 x[0,6] - start_service[0]
   + start_service[6] >= -999967.9589406843
 NodeArrivalVehicles[0,7]: - 999999 x[0,7] - start_service[0]
   + start_service[7] >= -999951.4555340747
 NodeArrivalVehicles[0,8]: - 999999 x[0,8] - start_service[0]
   + start_service[8] >= -999990.1581436468
 NodeArrivalVehicles[0,9]: - 999999 x[0,9] - start_service[0]
   + start_service[9] >= -999960.6560937124
 NodeArrivalVehicles[0,10]: - 999999 x[0,10] - start_service[0]
   + start_service[10] >= -999973.4984481733
 NodeArrivalVehicles[0,11]: - 999999 x[0,11] - start_service[0]
   + start_service[11] >= -999922.9870655254
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999970.4355137392
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999977.0287494746
 NodeArrivalVehicles[0,14]: - 999999 x[0,14] - start_service[0]
   + start_service[14] >= -999994.052853037
 NodeArrivalVehicles[0,15]: - 999999 x[0,15] - start_service[0]
   + start_service[15] >= -999930.4872109075
 NodeArrivalVehicles[0,16]: - 999999 x[0,16] - start_service[0]
   + start_service[16] >= -999967.9589406843
 NodeArrivalVehicles[0,17]: - 999999 x[0,17] - start_service[0]
   + start_service[17] >= -999951.4555340747
 NodeArrivalVehicles[0,18]: - 999999 x[0,18] - start_service[0]
   + start_service[18] >= -999990.1581436468
 NodeArrivalVehicles[0,19]: - 999999 x[0,19] - start_service[0]
   + start_service[19] >= -999960.6560937124
 NodeArrivalVehicles[0,20]: - 999999 x[0,20] - start_service[0]
   + start_service[20] >= -999973.4984481733
 NodeArrivalVehicles[0,21]: - 999999 x[0,21] - start_service[0]
   + start_service[21] >= -999922.9870655254
 NodeArrivalVehicles[0,22]: - 999999 x[0,22] - start_service[0]
   + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999 x[0,23] - start_service[0]
   + start_service[23] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
   + start_service[2] >= -999970.4355137392
 NodeArrivalVehicles[1,3]: - 999999 x[1,3] - start_service[1]
   + start_service[3] >= -999977.0287494746
 NodeArrivalVehicles[1,4]: - 999999 x[1,4] - start_service[1]
   + start_service[4] >= -999994.052853037
 NodeArrivalVehicles[1,5]: - 999999 x[1,5] - start_service[1]
   + start_service[5] >= -999930.4872109075
 NodeArrivalVehicles[1,6]: - 999999 x[1,6] - start_service[1]
   + start_service[6] >= -999967.9589406843
 NodeArrivalVehicles[1,7]: - 999999 x[1,7] - start_service[1]
   + start_service[7] >= -999951.4555340747
 NodeArrivalVehicles[1,8]: - 999999 x[1,8] - start_service[1]
   + start_service[8] >= -999990.1581436468
 NodeArrivalVehicles[1,9]: - 999999 x[1,9] - start_service[1]
   + start_service[9] >= -999960.6560937124
 NodeArrivalVehicles[1,10]: - 999999 x[1,10] - start_service[1]
   + start_service[10] >= -999973.4984481733
 NodeArrivalVehicles[1,11]: - 999999 x[1,11] - start_service[1]
   + start_service[11] >= -999922.9870655254
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999970.4355137392
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999977.0287494746
 NodeArrivalVehicles[1,14]: - 999999 x[1,14] - start_service[1]
   + start_service[14] >= -999994.052853037
 NodeArrivalVehicles[1,15]: - 999999 x[1,15] - start_service[1]
   + start_service[15] >= -999930.4872109075
 NodeArrivalVehicles[1,16]: - 999999 x[1,16] - start_service[1]
   + start_service[16] >= -999967.9589406843
 NodeArrivalVehicles[1,17]: - 999999 x[1,17] - start_service[1]
   + start_service[17] >= -999951.4555340747
 NodeArrivalVehicles[1,18]: - 999999 x[1,18] - start_service[1]
   + start_service[18] >= -999990.1581436468
 NodeArrivalVehicles[1,19]: - 999999 x[1,19] - start_service[1]
   + start_service[19] >= -999960.6560937124
 NodeArrivalVehicles[1,20]: - 999999 x[1,20] - start_service[1]
   + start_service[20] >= -999973.4984481733
 NodeArrivalVehicles[1,21]: - 999999 x[1,21] - start_service[1]
   + start_service[21] >= -999922.9870655254
 NodeArrivalVehicles[1,22]: - 999999 x[1,22] - start_service[1]
   + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999 x[1,23] - start_service[1]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999970.4355137392
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 999999 x[2,3] - start_service[2]
   + start_service[3] >= -999960.7530661469
 NodeArrivalVehicles[2,4]: - 999999 x[2,4] - start_service[2]
   + start_service[4] >= -999973.8738751572
 NodeArrivalVehicles[2,5]: - 999999 x[2,5] - start_service[2]
   + start_service[5] >= -999922.0828731196
 NodeArrivalVehicles[2,6]: - 999999 x[2,6] - start_service[2]
   + start_service[6] >= -999963.6131301826
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999962.1348413215
 NodeArrivalVehicles[2,8]: - 999999 x[2,8] - start_service[2]
   + start_service[8] >= -999979.0699702846
 NodeArrivalVehicles[2,9]: - 999999 x[2,9] - start_service[2]
   + start_service[9] >= -999986.4581145474
 NodeArrivalVehicles[2,10]: - 999999 x[2,10] - start_service[2]
   + start_service[10] >= -999957.3214814937
 NodeArrivalVehicles[2,11]: - 999999 x[2,11] - start_service[2]
   + start_service[11] >= -999924.0650949096
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - start_service[2]
   + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999960.7530661469
 NodeArrivalVehicles[2,14]: - 999999 x[2,14] - start_service[2]
   + start_service[14] >= -999973.8738751572
 NodeArrivalVehicles[2,15]: - 999999 x[2,15] - start_service[2]
   + start_service[15] >= -999922.0828731196
 NodeArrivalVehicles[2,16]: - 999999 x[2,16] - start_service[2]
   + start_service[16] >= -999963.6131301826
 NodeArrivalVehicles[2,17]: - 999999 x[2,17] - start_service[2]
   + start_service[17] >= -999962.1348413215
 NodeArrivalVehicles[2,18]: - 999999 x[2,18] - start_service[2]
   + start_service[18] >= -999979.0699702846
 NodeArrivalVehicles[2,19]: - 999999 x[2,19] - start_service[2]
   + start_service[19] >= -999986.4581145474
 NodeArrivalVehicles[2,20]: - 999999 x[2,20] - start_service[2]
   + start_service[20] >= -999957.3214814937
 NodeArrivalVehicles[2,21]: - 999999 x[2,21] - start_service[2]
   + start_service[21] >= -999924.0650949096
 NodeArrivalVehicles[2,22]: - 999999 x[2,22] - start_service[2]
   + start_service[22] >= -999970.4355137392
 NodeArrivalVehicles[2,23]: - 999999 x[2,23] - start_service[2]
   + start_service[23] >= -999970.4355137392
 NodeArrivalVehicles[3,1]: - 999999 x[3,1] + start_service[1]
   - start_service[3] >= -999977.0287494746
 NodeArrivalVehicles[3,2]: - 999999 x[3,2] + start_service[2]
   - start_service[3] >= -999960.7530661469
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 999999 x[3,4] - start_service[3]
   + start_service[4] >= -999979.4237314209
 NodeArrivalVehicles[3,5]: - 999999 x[3,5] - start_service[3]
   + start_service[5] >= -999952.4465287814
 NodeArrivalVehicles[3,6]: - 999999 x[3,6] - start_service[3]
   + start_service[6] >= -999984.5718729242
 NodeArrivalVehicles[3,7]: - 999999 x[3,7] - start_service[3]
   + start_service[7] >= -999962.7885731502
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999972.9207620689
 NodeArrivalVehicles[3,9]: - 999999 x[3,9] - start_service[3]
   + start_service[9] >= -999956.8858705129
 NodeArrivalVehicles[3,10]: - 999999 x[3,10] - start_service[3]
   + start_service[10] >= -999995.2305121262
 NodeArrivalVehicles[3,11]: - 999999 x[3,11] - start_service[3]
   + start_service[11] >= -999943.0282623119
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - start_service[3]
   + start_service[12] >= -999960.7530661469
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 999999 x[3,14] - start_service[3]
   + start_service[14] >= -999979.4237314209
 NodeArrivalVehicles[3,15]: - 999999 x[3,15] - start_service[3]
   + start_service[15] >= -999952.4465287814
 NodeArrivalVehicles[3,16]: - 999999 x[3,16] - start_service[3]
   + start_service[16] >= -999984.5718729242
 NodeArrivalVehicles[3,17]: - 999999 x[3,17] - start_service[3]
   + start_service[17] >= -999962.7885731502
 NodeArrivalVehicles[3,18]: - 999999 x[3,18] - start_service[3]
   + start_service[18] >= -999972.9207620689
 NodeArrivalVehicles[3,19]: - 999999 x[3,19] - start_service[3]
   + start_service[19] >= -999956.8858705129
 NodeArrivalVehicles[3,20]: - 999999 x[3,20] - start_service[3]
   + start_service[20] >= -999995.2305121262
 NodeArrivalVehicles[3,21]: - 999999 x[3,21] - start_service[3]
   + start_service[21] >= -999943.0282623119
 NodeArrivalVehicles[3,22]: - 999999 x[3,22] - start_service[3]
   + start_service[22] >= -999977.0287494746
 NodeArrivalVehicles[3,23]: - 999999 x[3,23] - start_service[3]
   + start_service[23] >= -999977.0287494746
 NodeArrivalVehicles[4,1]: - 999999 x[4,1] + start_service[1]
   - start_service[4] >= -999994.052853037
 NodeArrivalVehicles[4,2]: - 999999 x[4,2] + start_service[2]
   - start_service[4] >= -999973.8738751572
 NodeArrivalVehicles[4,3]: - 999999 x[4,3] + start_service[3]
   - start_service[4] >= -999979.4237314209
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 999999 x[4,5] - start_service[4]
   + start_service[5] >= -999933.296967469
 NodeArrivalVehicles[4,6]: - 999999 x[4,6] - start_service[4]
   + start_service[6] >= -999972.10830705
 NodeArrivalVehicles[4,7]: - 999999 x[4,7] - start_service[4]
   + start_service[7] >= -999956.3421461377
 NodeArrivalVehicles[4,8]: - 999999 x[4,8] - start_service[4]
   + start_service[8] >= -999991.9597178316
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999964.9144336868
 NodeArrivalVehicles[4,10]: - 999999 x[4,10] - start_service[4]
   + start_service[10] >= -999975.7042393816
 NodeArrivalVehicles[4,11]: - 999999 x[4,11] - start_service[4]
   + start_service[11] >= -999926.9998154708
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - start_service[4]
   + start_service[12] >= -999973.8738751572
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - start_service[4]
   + start_service[13] >= -999979.4237314209
 NodeArrivalVehicles[4,14]: - 999999 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 999999 x[4,15] - start_service[4]
   + start_service[15] >= -999933.296967469
 NodeArrivalVehicles[4,16]: - 999999 x[4,16] - start_service[4]
   + start_service[16] >= -999972.10830705
 NodeArrivalVehicles[4,17]: - 999999 x[4,17] - start_service[4]
   + start_service[17] >= -999956.3421461377
 NodeArrivalVehicles[4,18]: - 999999 x[4,18] - start_service[4]
   + start_service[18] >= -999991.9597178316
 NodeArrivalVehicles[4,19]: - 999999 x[4,19] - start_service[4]
   + start_service[19] >= -999964.9144336868
 NodeArrivalVehicles[4,20]: - 999999 x[4,20] - start_service[4]
   + start_service[20] >= -999975.7042393816
 NodeArrivalVehicles[4,21]: - 999999 x[4,21] - start_service[4]
   + start_service[21] >= -999926.9998154708
 NodeArrivalVehicles[4,22]: - 999999 x[4,22] - start_service[4]
   + start_service[22] >= -999994.052853037
 NodeArrivalVehicles[4,23]: - 999999 x[4,23] - start_service[4]
   + start_service[23] >= -999994.052853037
 NodeArrivalVehicles[5,1]: - 999999 x[5,1] + start_service[1]
   - start_service[5] >= -999930.4872109075
 NodeArrivalVehicles[5,2]: - 999999 x[5,2] + start_service[2]
   - start_service[5] >= -999922.0828731196
 NodeArrivalVehicles[5,3]: - 999999 x[5,3] + start_service[3]
   - start_service[5] >= -999952.4465287814
 NodeArrivalVehicles[5,4]: - 999999 x[5,4] + start_service[4]
   - start_service[5] >= -999933.296967469
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 999999 x[5,6] - start_service[5]
   + start_service[6] >= -999957.1706496663
 NodeArrivalVehicles[5,7]: - 999999 x[5,7] - start_service[5]
   + start_service[7] >= -999950.3991247974
 NodeArrivalVehicles[5,8]: - 999999 x[5,8] - start_service[5]
   + start_service[8] >= -999927.4985292086
 NodeArrivalVehicles[5,9]: - 999999 x[5,9] - start_service[5]
   + start_service[9] >= -999925.0867816372
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999955.8491601927
 NodeArrivalVehicles[5,11]: - 999999 x[5,11] - start_service[5]
   + start_service[11] >= -999974.0460374815
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - start_service[5]
   + start_service[12] >= -999922.0828731196
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - start_service[5]
   + start_service[13] >= -999952.4465287814
 NodeArrivalVehicles[5,14]: - 999999 x[5,14] - start_service[5]
   + start_service[14] >= -999933.296967469
 NodeArrivalVehicles[5,15]: - 999999 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 999999 x[5,16] - start_service[5]
   + start_service[16] >= -999957.1706496663
 NodeArrivalVehicles[5,17]: - 999999 x[5,17] - start_service[5]
   + start_service[17] >= -999950.3991247974
 NodeArrivalVehicles[5,18]: - 999999 x[5,18] - start_service[5]
   + start_service[18] >= -999927.4985292086
 NodeArrivalVehicles[5,19]: - 999999 x[5,19] - start_service[5]
   + start_service[19] >= -999925.0867816372
 NodeArrivalVehicles[5,20]: - 999999 x[5,20] - start_service[5]
   + start_service[20] >= -999955.8491601927
 NodeArrivalVehicles[5,21]: - 999999 x[5,21] - start_service[5]
   + start_service[21] >= -999974.0460374815
 NodeArrivalVehicles[5,22]: - 999999 x[5,22] - start_service[5]
   + start_service[22] >= -999930.4872109075
 NodeArrivalVehicles[5,23]: - 999999 x[5,23] - start_service[5]
   + start_service[23] >= -999930.4872109075
 NodeArrivalVehicles[6,1]: - 999999 x[6,1] + start_service[1]
   - start_service[6] >= -999967.9589406843
 NodeArrivalVehicles[6,2]: - 999999 x[6,2] + start_service[2]
   - start_service[6] >= -999963.6131301826
 NodeArrivalVehicles[6,3]: - 999999 x[6,3] + start_service[3]
   - start_service[6] >= -999984.5718729242
 NodeArrivalVehicles[6,4]: - 999999 x[6,4] + start_service[4]
   - start_service[6] >= -999972.10830705
 NodeArrivalVehicles[6,5]: - 999999 x[6,5] + start_service[5]
   - start_service[6] >= -999957.1706496663
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 999999 x[6,7] - start_service[6]
   + start_service[7] >= -999976.9595480452
 NodeArrivalVehicles[6,8]: - 999999 x[6,8] - start_service[6]
   + start_service[8] >= -999967.7691443588
 NodeArrivalVehicles[6,9]: - 999999 x[6,9] - start_service[6]
   + start_service[9] >= -999964.2077640658
 NodeArrivalVehicles[6,10]: - 999999 x[6,10] - start_service[6]
   + start_service[10] >= -999984.2983734549
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999953.8825736255
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - start_service[6]
   + start_service[12] >= -999963.6131301826
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - start_service[6]
   + start_service[13] >= -999984.5718729242
 NodeArrivalVehicles[6,14]: - 999999 x[6,14] - start_service[6]
   + start_service[14] >= -999972.10830705
 NodeArrivalVehicles[6,15]: - 999999 x[6,15] - start_service[6]
   + start_service[15] >= -999957.1706496663
 NodeArrivalVehicles[6,16]: - 999999 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 999999 x[6,17] - start_service[6]
   + start_service[17] >= -999976.9595480452
 NodeArrivalVehicles[6,18]: - 999999 x[6,18] - start_service[6]
   + start_service[18] >= -999967.7691443588
 NodeArrivalVehicles[6,19]: - 999999 x[6,19] - start_service[6]
   + start_service[19] >= -999964.2077640658
 NodeArrivalVehicles[6,20]: - 999999 x[6,20] - start_service[6]
   + start_service[20] >= -999984.2983734549
 NodeArrivalVehicles[6,21]: - 999999 x[6,21] - start_service[6]
   + start_service[21] >= -999953.8825736255
 NodeArrivalVehicles[6,22]: - 999999 x[6,22] - start_service[6]
   + start_service[22] >= -999967.9589406843
 NodeArrivalVehicles[6,23]: - 999999 x[6,23] - start_service[6]
   + start_service[23] >= -999967.9589406843
 NodeArrivalVehicles[7,1]: - 999999 x[7,1] + start_service[1]
   - start_service[7] >= -999951.4555340747
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999962.1348413215
 NodeArrivalVehicles[7,3]: - 999999 x[7,3] + start_service[3]
   - start_service[7] >= -999962.7885731502
 NodeArrivalVehicles[7,4]: - 999999 x[7,4] + start_service[4]
   - start_service[7] >= -999956.3421461377
 NodeArrivalVehicles[7,5]: - 999999 x[7,5] + start_service[5]
   - start_service[7] >= -999950.3991247974
 NodeArrivalVehicles[7,6]: - 999999 x[7,6] + start_service[6]
   - start_service[7] >= -999976.9595480452
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 999999 x[7,8] - start_service[7]
   + start_service[8] >= -999955.0366450849
 NodeArrivalVehicles[7,9]: - 999999 x[7,9] - start_service[7]
   + start_service[9] >= -999970.2808806106
 NodeArrivalVehicles[7,10]: - 999999 x[7,10] - start_service[7]
   + start_service[10] >= -999962.2590264571
 NodeArrivalVehicles[7,11]: - 999999 x[7,11] - start_service[7]
   + start_service[11] >= -999959.5953207669
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999962.1348413215
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - start_service[7]
   + start_service[13] >= -999962.7885731502
 NodeArrivalVehicles[7,14]: - 999999 x[7,14] - start_service[7]
   + start_service[14] >= -999956.3421461377
 NodeArrivalVehicles[7,15]: - 999999 x[7,15] - start_service[7]
   + start_service[15] >= -999950.3991247974
 NodeArrivalVehicles[7,16]: - 999999 x[7,16] - start_service[7]
   + start_service[16] >= -999976.9595480452
 NodeArrivalVehicles[7,17]: - 999999 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 999999 x[7,18] - start_service[7]
   + start_service[18] >= -999955.0366450849
 NodeArrivalVehicles[7,19]: - 999999 x[7,19] - start_service[7]
   + start_service[19] >= -999970.2808806106
 NodeArrivalVehicles[7,20]: - 999999 x[7,20] - start_service[7]
   + start_service[20] >= -999962.2590264571
 NodeArrivalVehicles[7,21]: - 999999 x[7,21] - start_service[7]
   + start_service[21] >= -999959.5953207669
 NodeArrivalVehicles[7,22]: - 999999 x[7,22] - start_service[7]
   + start_service[22] >= -999951.4555340747
 NodeArrivalVehicles[7,23]: - 999999 x[7,23] - start_service[7]
   + start_service[23] >= -999951.4555340747
 NodeArrivalVehicles[8,1]: - 999999 x[8,1] + start_service[1]
   - start_service[8] >= -999990.1581436468
 NodeArrivalVehicles[8,2]: - 999999 x[8,2] + start_service[2]
   - start_service[8] >= -999979.0699702846
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999972.9207620689
 NodeArrivalVehicles[8,4]: - 999999 x[8,4] + start_service[4]
   - start_service[8] >= -999991.9597178316
 NodeArrivalVehicles[8,5]: - 999999 x[8,5] + start_service[5]
   - start_service[8] >= -999927.4985292086
 NodeArrivalVehicles[8,6]: - 999999 x[8,6] + start_service[6]
   - start_service[8] >= -999967.7691443588
 NodeArrivalVehicles[8,7]: - 999999 x[8,7] + start_service[7]
   - start_service[8] >= -999955.0366450849
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 999999 x[8,9] - start_service[8]
   + start_service[9] >= -999968.5222110867
 NodeArrivalVehicles[8,10]: - 999999 x[8,10] - start_service[8]
   + start_service[10] >= -999969.1569164672
 NodeArrivalVehicles[8,11]: - 999999 x[8,11] - start_service[8]
   + start_service[11] >= -999922.8849691104
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - start_service[8]
   + start_service[12] >= -999979.0699702846
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - start_service[8]
   + start_service[13] >= -999972.9207620689
 NodeArrivalVehicles[8,14]: - 999999 x[8,14] - start_service[8]
   + start_service[14] >= -999991.9597178316
 NodeArrivalVehicles[8,15]: - 999999 x[8,15] - start_service[8]
   + start_service[15] >= -999927.4985292086
 NodeArrivalVehicles[8,16]: - 999999 x[8,16] - start_service[8]
   + start_service[16] >= -999967.7691443588
 NodeArrivalVehicles[8,17]: - 999999 x[8,17] - start_service[8]
   + start_service[17] >= -999955.0366450849
 NodeArrivalVehicles[8,18]: - 999999 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 999999 x[8,19] - start_service[8]
   + start_service[19] >= -999968.5222110867
 NodeArrivalVehicles[8,20]: - 999999 x[8,20] - start_service[8]
   + start_service[20] >= -999969.1569164672
 NodeArrivalVehicles[8,21]: - 999999 x[8,21] - start_service[8]
   + start_service[21] >= -999922.8849691104
 NodeArrivalVehicles[8,22]: - 999999 x[8,22] - start_service[8]
   + start_service[22] >= -999990.1581436468
 NodeArrivalVehicles[8,23]: - 999999 x[8,23] - start_service[8]
   + start_service[23] >= -999990.1581436468
 NodeArrivalVehicles[9,1]: - 999999 x[9,1] + start_service[1]
   - start_service[9] >= -999960.6560937124
 NodeArrivalVehicles[9,2]: - 999999 x[9,2] + start_service[2]
   - start_service[9] >= -999986.4581145474
 NodeArrivalVehicles[9,3]: - 999999 x[9,3] + start_service[3]
   - start_service[9] >= -999956.8858705129
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999964.9144336868
 NodeArrivalVehicles[9,5]: - 999999 x[9,5] + start_service[5]
   - start_service[9] >= -999925.0867816372
 NodeArrivalVehicles[9,6]: - 999999 x[9,6] + start_service[6]
   - start_service[9] >= -999964.2077640658
 NodeArrivalVehicles[9,7]: - 999999 x[9,7] + start_service[7]
   - start_service[9] >= -999970.2808806106
 NodeArrivalVehicles[9,8]: - 999999 x[9,8] + start_service[8]
   - start_service[9] >= -999968.5222110867
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 999999 x[9,10] - start_service[9]
   + start_service[10] >= -999954.0422286508
 NodeArrivalVehicles[9,11]: - 999999 x[9,11] - start_service[9]
   + start_service[11] >= -999930.9304394979
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - start_service[9]
   + start_service[12] >= -999986.4581145474
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - start_service[9]
   + start_service[13] >= -999956.8858705129
 NodeArrivalVehicles[9,14]: - 999999 x[9,14] - start_service[9]
   + start_service[14] >= -999964.9144336868
 NodeArrivalVehicles[9,15]: - 999999 x[9,15] - start_service[9]
   + start_service[15] >= -999925.0867816372
 NodeArrivalVehicles[9,16]: - 999999 x[9,16] - start_service[9]
   + start_service[16] >= -999964.2077640658
 NodeArrivalVehicles[9,17]: - 999999 x[9,17] - start_service[9]
   + start_service[17] >= -999970.2808806106
 NodeArrivalVehicles[9,18]: - 999999 x[9,18] - start_service[9]
   + start_service[18] >= -999968.5222110867
 NodeArrivalVehicles[9,19]: - 999999 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 999999 x[9,20] - start_service[9]
   + start_service[20] >= -999954.0422286508
 NodeArrivalVehicles[9,21]: - 999999 x[9,21] - start_service[9]
   + start_service[21] >= -999930.9304394979
 NodeArrivalVehicles[9,22]: - 999999 x[9,22] - start_service[9]
   + start_service[22] >= -999960.6560937124
 NodeArrivalVehicles[9,23]: - 999999 x[9,23] - start_service[9]
   + start_service[23] >= -999960.6560937124
 NodeArrivalVehicles[10,1]: - 999999 x[10,1] + start_service[1]
   - start_service[10] >= -999973.4984481733
 NodeArrivalVehicles[10,2]: - 999999 x[10,2] + start_service[2]
   - start_service[10] >= -999957.3214814937
 NodeArrivalVehicles[10,3]: - 999999 x[10,3] + start_service[3]
   - start_service[10] >= -999995.2305121262
 NodeArrivalVehicles[10,4]: - 999999 x[10,4] + start_service[4]
   - start_service[10] >= -999975.7042393816
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999955.8491601927
 NodeArrivalVehicles[10,6]: - 999999 x[10,6] + start_service[6]
   - start_service[10] >= -999984.2983734549
 NodeArrivalVehicles[10,7]: - 999999 x[10,7] + start_service[7]
   - start_service[10] >= -999962.2590264571
 NodeArrivalVehicles[10,8]: - 999999 x[10,8] + start_service[8]
   - start_service[10] >= -999969.1569164672
 NodeArrivalVehicles[10,9]: - 999999 x[10,9] + start_service[9]
   - start_service[10] >= -999954.0422286508
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 999999 x[10,11] - start_service[10]
   + start_service[11] >= -999945.3398959702
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - start_service[10]
   + start_service[12] >= -999957.3214814937
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - start_service[10]
   + start_service[13] >= -999995.2305121262
 NodeArrivalVehicles[10,14]: - 999999 x[10,14] - start_service[10]
   + start_service[14] >= -999975.7042393816
 NodeArrivalVehicles[10,15]: - 999999 x[10,15] - start_service[10]
   + start_service[15] >= -999955.8491601927
 NodeArrivalVehicles[10,16]: - 999999 x[10,16] - start_service[10]
   + start_service[16] >= -999984.2983734549
 NodeArrivalVehicles[10,17]: - 999999 x[10,17] - start_service[10]
   + start_service[17] >= -999962.2590264571
 NodeArrivalVehicles[10,18]: - 999999 x[10,18] - start_service[10]
   + start_service[18] >= -999969.1569164672
 NodeArrivalVehicles[10,19]: - 999999 x[10,19] - start_service[10]
   + start_service[19] >= -999954.0422286508
 NodeArrivalVehicles[10,20]: - 999999 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 999999 x[10,21] - start_service[10]
   + start_service[21] >= -999945.3398959702
 NodeArrivalVehicles[10,22]: - 999999 x[10,22] - start_service[10]
   + start_service[22] >= -999973.4984481733
 NodeArrivalVehicles[10,23]: - 999999 x[10,23] - start_service[10]
   + start_service[23] >= -999973.4984481733
 NodeArrivalVehicles[11,1]: - 999999 x[11,1] + start_service[1]
   - start_service[11] >= -999922.9870655254
 NodeArrivalVehicles[11,2]: - 999999 x[11,2] + start_service[2]
   - start_service[11] >= -999924.0650949096
 NodeArrivalVehicles[11,3]: - 999999 x[11,3] + start_service[3]
   - start_service[11] >= -999943.0282623119
 NodeArrivalVehicles[11,4]: - 999999 x[11,4] + start_service[4]
   - start_service[11] >= -999926.9998154708
 NodeArrivalVehicles[11,5]: - 999999 x[11,5] + start_service[5]
   - start_service[11] >= -999974.0460374815
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999953.8825736255
 NodeArrivalVehicles[11,7]: - 999999 x[11,7] + start_service[7]
   - start_service[11] >= -999959.5953207669
 NodeArrivalVehicles[11,8]: - 999999 x[11,8] + start_service[8]
   - start_service[11] >= -999922.8849691104
 NodeArrivalVehicles[11,9]: - 999999 x[11,9] + start_service[9]
   - start_service[11] >= -999930.9304394979
 NodeArrivalVehicles[11,10]: - 999999 x[11,10] + start_service[10]
   - start_service[11] >= -999945.3398959702
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - start_service[11]
   + start_service[12] >= -999924.0650949096
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - start_service[11]
   + start_service[13] >= -999943.0282623119
 NodeArrivalVehicles[11,14]: - 999999 x[11,14] - start_service[11]
   + start_service[14] >= -999926.9998154708
 NodeArrivalVehicles[11,15]: - 999999 x[11,15] - start_service[11]
   + start_service[15] >= -999974.0460374815
 NodeArrivalVehicles[11,16]: - 999999 x[11,16] - start_service[11]
   + start_service[16] >= -999953.8825736255
 NodeArrivalVehicles[11,17]: - 999999 x[11,17] - start_service[11]
   + start_service[17] >= -999959.5953207669
 NodeArrivalVehicles[11,18]: - 999999 x[11,18] - start_service[11]
   + start_service[18] >= -999922.8849691104
 NodeArrivalVehicles[11,19]: - 999999 x[11,19] - start_service[11]
   + start_service[19] >= -999930.9304394979
 NodeArrivalVehicles[11,20]: - 999999 x[11,20] - start_service[11]
   + start_service[20] >= -999945.3398959702
 NodeArrivalVehicles[11,21]: - 999999 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 999999 x[11,22] - start_service[11]
   + start_service[22] >= -999922.9870655254
 NodeArrivalVehicles[11,23]: - 999999 x[11,23] - start_service[11]
   + start_service[23] >= -999922.9870655254
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999970.4355137392
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] + start_service[2]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 999999 x[12,3] + start_service[3]
   - start_service[12] >= -999960.7530661469
 NodeArrivalVehicles[12,4]: - 999999 x[12,4] + start_service[4]
   - start_service[12] >= -999973.8738751572
 NodeArrivalVehicles[12,5]: - 999999 x[12,5] + start_service[5]
   - start_service[12] >= -999922.0828731196
 NodeArrivalVehicles[12,6]: - 999999 x[12,6] + start_service[6]
   - start_service[12] >= -999963.6131301826
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999962.1348413215
 NodeArrivalVehicles[12,8]: - 999999 x[12,8] + start_service[8]
   - start_service[12] >= -999979.0699702846
 NodeArrivalVehicles[12,9]: - 999999 x[12,9] + start_service[9]
   - start_service[12] >= -999986.4581145474
 NodeArrivalVehicles[12,10]: - 999999 x[12,10] + start_service[10]
   - start_service[12] >= -999957.3214814937
 NodeArrivalVehicles[12,11]: - 999999 x[12,11] + start_service[11]
   - start_service[12] >= -999924.0650949096
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999960.7530661469
 NodeArrivalVehicles[12,14]: - 999999 x[12,14] - start_service[12]
   + start_service[14] >= -999973.8738751572
 NodeArrivalVehicles[12,15]: - 999999 x[12,15] - start_service[12]
   + start_service[15] >= -999922.0828731196
 NodeArrivalVehicles[12,16]: - 999999 x[12,16] - start_service[12]
   + start_service[16] >= -999963.6131301826
 NodeArrivalVehicles[12,17]: - 999999 x[12,17] - start_service[12]
   + start_service[17] >= -999962.1348413215
 NodeArrivalVehicles[12,18]: - 999999 x[12,18] - start_service[12]
   + start_service[18] >= -999979.0699702846
 NodeArrivalVehicles[12,19]: - 999999 x[12,19] - start_service[12]
   + start_service[19] >= -999986.4581145474
 NodeArrivalVehicles[12,20]: - 999999 x[12,20] - start_service[12]
   + start_service[20] >= -999957.3214814937
 NodeArrivalVehicles[12,21]: - 999999 x[12,21] - start_service[12]
   + start_service[21] >= -999924.0650949096
 NodeArrivalVehicles[12,22]: - 999999 x[12,22] - start_service[12]
   + start_service[22] >= -999970.4355137392
 NodeArrivalVehicles[12,23]: - 999999 x[12,23] - start_service[12]
   + start_service[23] >= -999970.4355137392
 NodeArrivalVehicles[13,1]: - 999999 x[13,1] + start_service[1]
   - start_service[13] >= -999977.0287494746
 NodeArrivalVehicles[13,2]: - 999999 x[13,2] + start_service[2]
   - start_service[13] >= -999960.7530661469
 NodeArrivalVehicles[13,3]: - 999999 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 999999 x[13,4] + start_service[4]
   - start_service[13] >= -999979.4237314209
 NodeArrivalVehicles[13,5]: - 999999 x[13,5] + start_service[5]
   - start_service[13] >= -999952.4465287814
 NodeArrivalVehicles[13,6]: - 999999 x[13,6] + start_service[6]
   - start_service[13] >= -999984.5718729242
 NodeArrivalVehicles[13,7]: - 999999 x[13,7] + start_service[7]
   - start_service[13] >= -999962.7885731502
 NodeArrivalVehicles[13,8]: - 999999 x[13,8] + start_service[8]
   - start_service[13] >= -999972.9207620689
 NodeArrivalVehicles[13,9]: - 999999 x[13,9] + start_service[9]
   - start_service[13] >= -999956.8858705129
 NodeArrivalVehicles[13,10]: - 999999 x[13,10] + start_service[10]
   - start_service[13] >= -999995.2305121262
 NodeArrivalVehicles[13,11]: - 999999 x[13,11] + start_service[11]
   - start_service[13] >= -999943.0282623119
 NodeArrivalVehicles[13,12]: - 999999 x[13,12] + start_service[12]
   - start_service[13] >= -999960.7530661469
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 999999 x[13,14] - start_service[13]
   + start_service[14] >= -999979.4237314209
 NodeArrivalVehicles[13,15]: - 999999 x[13,15] - start_service[13]
   + start_service[15] >= -999952.4465287814
 NodeArrivalVehicles[13,16]: - 999999 x[13,16] - start_service[13]
   + start_service[16] >= -999984.5718729242
 NodeArrivalVehicles[13,17]: - 999999 x[13,17] - start_service[13]
   + start_service[17] >= -999962.7885731502
 NodeArrivalVehicles[13,18]: - 999999 x[13,18] - start_service[13]
   + start_service[18] >= -999972.9207620689
 NodeArrivalVehicles[13,19]: - 999999 x[13,19] - start_service[13]
   + start_service[19] >= -999956.8858705129
 NodeArrivalVehicles[13,20]: - 999999 x[13,20] - start_service[13]
   + start_service[20] >= -999995.2305121262
 NodeArrivalVehicles[13,21]: - 999999 x[13,21] - start_service[13]
   + start_service[21] >= -999943.0282623119
 NodeArrivalVehicles[13,22]: - 999999 x[13,22] - start_service[13]
   + start_service[22] >= -999977.0287494746
 NodeArrivalVehicles[13,23]: - 999999 x[13,23] - start_service[13]
   + start_service[23] >= -999977.0287494746
 NodeArrivalVehicles[14,1]: - 999999 x[14,1] + start_service[1]
   - start_service[14] >= -999994.052853037
 NodeArrivalVehicles[14,2]: - 999999 x[14,2] + start_service[2]
   - start_service[14] >= -999973.8738751572
 NodeArrivalVehicles[14,3]: - 999999 x[14,3] + start_service[3]
   - start_service[14] >= -999979.4237314209
 NodeArrivalVehicles[14,4]: - 999999 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 999999 x[14,5] + start_service[5]
   - start_service[14] >= -999933.296967469
 NodeArrivalVehicles[14,6]: - 999999 x[14,6] + start_service[6]
   - start_service[14] >= -999972.10830705
 NodeArrivalVehicles[14,7]: - 999999 x[14,7] + start_service[7]
   - start_service[14] >= -999956.3421461377
 NodeArrivalVehicles[14,8]: - 999999 x[14,8] + start_service[8]
   - start_service[14] >= -999991.9597178316
 NodeArrivalVehicles[14,9]: - 999999 x[14,9] + start_service[9]
   - start_service[14] >= -999964.9144336868
 NodeArrivalVehicles[14,10]: - 999999 x[14,10] + start_service[10]
   - start_service[14] >= -999975.7042393816
 NodeArrivalVehicles[14,11]: - 999999 x[14,11] + start_service[11]
   - start_service[14] >= -999926.9998154708
 NodeArrivalVehicles[14,12]: - 999999 x[14,12] + start_service[12]
   - start_service[14] >= -999973.8738751572
 NodeArrivalVehicles[14,13]: - 999999 x[14,13] + start_service[13]
   - start_service[14] >= -999979.4237314209
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 999999 x[14,15] - start_service[14]
   + start_service[15] >= -999933.296967469
 NodeArrivalVehicles[14,16]: - 999999 x[14,16] - start_service[14]
   + start_service[16] >= -999972.10830705
 NodeArrivalVehicles[14,17]: - 999999 x[14,17] - start_service[14]
   + start_service[17] >= -999956.3421461377
 NodeArrivalVehicles[14,18]: - 999999 x[14,18] - start_service[14]
   + start_service[18] >= -999991.9597178316
 NodeArrivalVehicles[14,19]: - 999999 x[14,19] - start_service[14]
   + start_service[19] >= -999964.9144336868
 NodeArrivalVehicles[14,20]: - 999999 x[14,20] - start_service[14]
   + start_service[20] >= -999975.7042393816
 NodeArrivalVehicles[14,21]: - 999999 x[14,21] - start_service[14]
   + start_service[21] >= -999926.9998154708
 NodeArrivalVehicles[14,22]: - 999999 x[14,22] - start_service[14]
   + start_service[22] >= -999994.052853037
 NodeArrivalVehicles[14,23]: - 999999 x[14,23] - start_service[14]
   + start_service[23] >= -999994.052853037
 NodeArrivalVehicles[15,1]: - 999999 x[15,1] + start_service[1]
   - start_service[15] >= -999930.4872109075
 NodeArrivalVehicles[15,2]: - 999999 x[15,2] + start_service[2]
   - start_service[15] >= -999922.0828731196
 NodeArrivalVehicles[15,3]: - 999999 x[15,3] + start_service[3]
   - start_service[15] >= -999952.4465287814
 NodeArrivalVehicles[15,4]: - 999999 x[15,4] + start_service[4]
   - start_service[15] >= -999933.296967469
 NodeArrivalVehicles[15,5]: - 999999 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 999999 x[15,6] + start_service[6]
   - start_service[15] >= -999957.1706496663
 NodeArrivalVehicles[15,7]: - 999999 x[15,7] + start_service[7]
   - start_service[15] >= -999950.3991247974
 NodeArrivalVehicles[15,8]: - 999999 x[15,8] + start_service[8]
   - start_service[15] >= -999927.4985292086
 NodeArrivalVehicles[15,9]: - 999999 x[15,9] + start_service[9]
   - start_service[15] >= -999925.0867816372
 NodeArrivalVehicles[15,10]: - 999999 x[15,10] + start_service[10]
   - start_service[15] >= -999955.8491601927
 NodeArrivalVehicles[15,11]: - 999999 x[15,11] + start_service[11]
   - start_service[15] >= -999974.0460374815
 NodeArrivalVehicles[15,12]: - 999999 x[15,12] + start_service[12]
   - start_service[15] >= -999922.0828731196
 NodeArrivalVehicles[15,13]: - 999999 x[15,13] + start_service[13]
   - start_service[15] >= -999952.4465287814
 NodeArrivalVehicles[15,14]: - 999999 x[15,14] + start_service[14]
   - start_service[15] >= -999933.296967469
 NodeArrivalVehicles[15,15]: - 999999 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 999999 x[15,16] - start_service[15]
   + start_service[16] >= -999957.1706496663
 NodeArrivalVehicles[15,17]: - 999999 x[15,17] - start_service[15]
   + start_service[17] >= -999950.3991247974
 NodeArrivalVehicles[15,18]: - 999999 x[15,18] - start_service[15]
   + start_service[18] >= -999927.4985292086
 NodeArrivalVehicles[15,19]: - 999999 x[15,19] - start_service[15]
   + start_service[19] >= -999925.0867816372
 NodeArrivalVehicles[15,20]: - 999999 x[15,20] - start_service[15]
   + start_service[20] >= -999955.8491601927
 NodeArrivalVehicles[15,21]: - 999999 x[15,21] - start_service[15]
   + start_service[21] >= -999974.0460374815
 NodeArrivalVehicles[15,22]: - 999999 x[15,22] - start_service[15]
   + start_service[22] >= -999930.4872109075
 NodeArrivalVehicles[15,23]: - 999999 x[15,23] - start_service[15]
   + start_service[23] >= -999930.4872109075
 NodeArrivalVehicles[16,1]: - 999999 x[16,1] + start_service[1]
   - start_service[16] >= -999967.9589406843
 NodeArrivalVehicles[16,2]: - 999999 x[16,2] + start_service[2]
   - start_service[16] >= -999963.6131301826
 NodeArrivalVehicles[16,3]: - 999999 x[16,3] + start_service[3]
   - start_service[16] >= -999984.5718729242
 NodeArrivalVehicles[16,4]: - 999999 x[16,4] + start_service[4]
   - start_service[16] >= -999972.10830705
 NodeArrivalVehicles[16,5]: - 999999 x[16,5] + start_service[5]
   - start_service[16] >= -999957.1706496663
 NodeArrivalVehicles[16,6]: - 999999 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 999999 x[16,7] + start_service[7]
   - start_service[16] >= -999976.9595480452
 NodeArrivalVehicles[16,8]: - 999999 x[16,8] + start_service[8]
   - start_service[16] >= -999967.7691443588
 NodeArrivalVehicles[16,9]: - 999999 x[16,9] + start_service[9]
   - start_service[16] >= -999964.2077640658
 NodeArrivalVehicles[16,10]: - 999999 x[16,10] + start_service[10]
   - start_service[16] >= -999984.2983734549
 NodeArrivalVehicles[16,11]: - 999999 x[16,11] + start_service[11]
   - start_service[16] >= -999953.8825736255
 NodeArrivalVehicles[16,12]: - 999999 x[16,12] + start_service[12]
   - start_service[16] >= -999963.6131301826
 NodeArrivalVehicles[16,13]: - 999999 x[16,13] + start_service[13]
   - start_service[16] >= -999984.5718729242
 NodeArrivalVehicles[16,14]: - 999999 x[16,14] + start_service[14]
   - start_service[16] >= -999972.10830705
 NodeArrivalVehicles[16,15]: - 999999 x[16,15] + start_service[15]
   - start_service[16] >= -999957.1706496663
 NodeArrivalVehicles[16,16]: - 999999 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 999999 x[16,17] - start_service[16]
   + start_service[17] >= -999976.9595480452
 NodeArrivalVehicles[16,18]: - 999999 x[16,18] - start_service[16]
   + start_service[18] >= -999967.7691443588
 NodeArrivalVehicles[16,19]: - 999999 x[16,19] - start_service[16]
   + start_service[19] >= -999964.2077640658
 NodeArrivalVehicles[16,20]: - 999999 x[16,20] - start_service[16]
   + start_service[20] >= -999984.2983734549
 NodeArrivalVehicles[16,21]: - 999999 x[16,21] - start_service[16]
   + start_service[21] >= -999953.8825736255
 NodeArrivalVehicles[16,22]: - 999999 x[16,22] - start_service[16]
   + start_service[22] >= -999967.9589406843
 NodeArrivalVehicles[16,23]: - 999999 x[16,23] - start_service[16]
   + start_service[23] >= -999967.9589406843
 NodeArrivalVehicles[17,1]: - 999999 x[17,1] + start_service[1]
   - start_service[17] >= -999951.4555340747
 NodeArrivalVehicles[17,2]: - 999999 x[17,2] + start_service[2]
   - start_service[17] >= -999962.1348413215
 NodeArrivalVehicles[17,3]: - 999999 x[17,3] + start_service[3]
   - start_service[17] >= -999962.7885731502
 NodeArrivalVehicles[17,4]: - 999999 x[17,4] + start_service[4]
   - start_service[17] >= -999956.3421461377
 NodeArrivalVehicles[17,5]: - 999999 x[17,5] + start_service[5]
   - start_service[17] >= -999950.3991247974
 NodeArrivalVehicles[17,6]: - 999999 x[17,6] + start_service[6]
   - start_service[17] >= -999976.9595480452
 NodeArrivalVehicles[17,7]: - 999999 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 999999 x[17,8] + start_service[8]
   - start_service[17] >= -999955.0366450849
 NodeArrivalVehicles[17,9]: - 999999 x[17,9] + start_service[9]
   - start_service[17] >= -999970.2808806106
 NodeArrivalVehicles[17,10]: - 999999 x[17,10] + start_service[10]
   - start_service[17] >= -999962.2590264571
 NodeArrivalVehicles[17,11]: - 999999 x[17,11] + start_service[11]
   - start_service[17] >= -999959.5953207669
 NodeArrivalVehicles[17,12]: - 999999 x[17,12] + start_service[12]
   - start_service[17] >= -999962.1348413215
 NodeArrivalVehicles[17,13]: - 999999 x[17,13] + start_service[13]
   - start_service[17] >= -999962.7885731502
 NodeArrivalVehicles[17,14]: - 999999 x[17,14] + start_service[14]
   - start_service[17] >= -999956.3421461377
 NodeArrivalVehicles[17,15]: - 999999 x[17,15] + start_service[15]
   - start_service[17] >= -999950.3991247974
 NodeArrivalVehicles[17,16]: - 999999 x[17,16] + start_service[16]
   - start_service[17] >= -999976.9595480452
 NodeArrivalVehicles[17,17]: - 999999 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 999999 x[17,18] - start_service[17]
   + start_service[18] >= -999955.0366450849
 NodeArrivalVehicles[17,19]: - 999999 x[17,19] - start_service[17]
   + start_service[19] >= -999970.2808806106
 NodeArrivalVehicles[17,20]: - 999999 x[17,20] - start_service[17]
   + start_service[20] >= -999962.2590264571
 NodeArrivalVehicles[17,21]: - 999999 x[17,21] - start_service[17]
   + start_service[21] >= -999959.5953207669
 NodeArrivalVehicles[17,22]: - 999999 x[17,22] - start_service[17]
   + start_service[22] >= -999951.4555340747
 NodeArrivalVehicles[17,23]: - 999999 x[17,23] - start_service[17]
   + start_service[23] >= -999951.4555340747
 NodeArrivalVehicles[18,1]: - 999999 x[18,1] + start_service[1]
   - start_service[18] >= -999990.1581436468
 NodeArrivalVehicles[18,2]: - 999999 x[18,2] + start_service[2]
   - start_service[18] >= -999979.0699702846
 NodeArrivalVehicles[18,3]: - 999999 x[18,3] + start_service[3]
   - start_service[18] >= -999972.9207620689
 NodeArrivalVehicles[18,4]: - 999999 x[18,4] + start_service[4]
   - start_service[18] >= -999991.9597178316
 NodeArrivalVehicles[18,5]: - 999999 x[18,5] + start_service[5]
   - start_service[18] >= -999927.4985292086
 NodeArrivalVehicles[18,6]: - 999999 x[18,6] + start_service[6]
   - start_service[18] >= -999967.7691443588
 NodeArrivalVehicles[18,7]: - 999999 x[18,7] + start_service[7]
   - start_service[18] >= -999955.0366450849
 NodeArrivalVehicles[18,8]: - 999999 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 999999 x[18,9] + start_service[9]
   - start_service[18] >= -999968.5222110867
 NodeArrivalVehicles[18,10]: - 999999 x[18,10] + start_service[10]
   - start_service[18] >= -999969.1569164672
 NodeArrivalVehicles[18,11]: - 999999 x[18,11] + start_service[11]
   - start_service[18] >= -999922.8849691104
 NodeArrivalVehicles[18,12]: - 999999 x[18,12] + start_service[12]
   - start_service[18] >= -999979.0699702846
 NodeArrivalVehicles[18,13]: - 999999 x[18,13] + start_service[13]
   - start_service[18] >= -999972.9207620689
 NodeArrivalVehicles[18,14]: - 999999 x[18,14] + start_service[14]
   - start_service[18] >= -999991.9597178316
 NodeArrivalVehicles[18,15]: - 999999 x[18,15] + start_service[15]
   - start_service[18] >= -999927.4985292086
 NodeArrivalVehicles[18,16]: - 999999 x[18,16] + start_service[16]
   - start_service[18] >= -999967.7691443588
 NodeArrivalVehicles[18,17]: - 999999 x[18,17] + start_service[17]
   - start_service[18] >= -999955.0366450849
 NodeArrivalVehicles[18,18]: - 999999 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 999999 x[18,19] - start_service[18]
   + start_service[19] >= -999968.5222110867
 NodeArrivalVehicles[18,20]: - 999999 x[18,20] - start_service[18]
   + start_service[20] >= -999969.1569164672
 NodeArrivalVehicles[18,21]: - 999999 x[18,21] - start_service[18]
   + start_service[21] >= -999922.8849691104
 NodeArrivalVehicles[18,22]: - 999999 x[18,22] - start_service[18]
   + start_service[22] >= -999990.1581436468
 NodeArrivalVehicles[18,23]: - 999999 x[18,23] - start_service[18]
   + start_service[23] >= -999990.1581436468
 NodeArrivalVehicles[19,1]: - 999999 x[19,1] + start_service[1]
   - start_service[19] >= -999960.6560937124
 NodeArrivalVehicles[19,2]: - 999999 x[19,2] + start_service[2]
   - start_service[19] >= -999986.4581145474
 NodeArrivalVehicles[19,3]: - 999999 x[19,3] + start_service[3]
   - start_service[19] >= -999956.8858705129
 NodeArrivalVehicles[19,4]: - 999999 x[19,4] + start_service[4]
   - start_service[19] >= -999964.9144336868
 NodeArrivalVehicles[19,5]: - 999999 x[19,5] + start_service[5]
   - start_service[19] >= -999925.0867816372
 NodeArrivalVehicles[19,6]: - 999999 x[19,6] + start_service[6]
   - start_service[19] >= -999964.2077640658
 NodeArrivalVehicles[19,7]: - 999999 x[19,7] + start_service[7]
   - start_service[19] >= -999970.2808806106
 NodeArrivalVehicles[19,8]: - 999999 x[19,8] + start_service[8]
   - start_service[19] >= -999968.5222110867
 NodeArrivalVehicles[19,9]: - 999999 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 999999 x[19,10] + start_service[10]
   - start_service[19] >= -999954.0422286508
 NodeArrivalVehicles[19,11]: - 999999 x[19,11] + start_service[11]
   - start_service[19] >= -999930.9304394979
 NodeArrivalVehicles[19,12]: - 999999 x[19,12] + start_service[12]
   - start_service[19] >= -999986.4581145474
 NodeArrivalVehicles[19,13]: - 999999 x[19,13] + start_service[13]
   - start_service[19] >= -999956.8858705129
 NodeArrivalVehicles[19,14]: - 999999 x[19,14] + start_service[14]
   - start_service[19] >= -999964.9144336868
 NodeArrivalVehicles[19,15]: - 999999 x[19,15] + start_service[15]
   - start_service[19] >= -999925.0867816372
 NodeArrivalVehicles[19,16]: - 999999 x[19,16] + start_service[16]
   - start_service[19] >= -999964.2077640658
 NodeArrivalVehicles[19,17]: - 999999 x[19,17] + start_service[17]
   - start_service[19] >= -999970.2808806106
 NodeArrivalVehicles[19,18]: - 999999 x[19,18] + start_service[18]
   - start_service[19] >= -999968.5222110867
 NodeArrivalVehicles[19,19]: - 999999 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 999999 x[19,20] - start_service[19]
   + start_service[20] >= -999954.0422286508
 NodeArrivalVehicles[19,21]: - 999999 x[19,21] - start_service[19]
   + start_service[21] >= -999930.9304394979
 NodeArrivalVehicles[19,22]: - 999999 x[19,22] - start_service[19]
   + start_service[22] >= -999960.6560937124
 NodeArrivalVehicles[19,23]: - 999999 x[19,23] - start_service[19]
   + start_service[23] >= -999960.6560937124
 NodeArrivalVehicles[20,1]: - 999999 x[20,1] + start_service[1]
   - start_service[20] >= -999973.4984481733
 NodeArrivalVehicles[20,2]: - 999999 x[20,2] + start_service[2]
   - start_service[20] >= -999957.3214814937
 NodeArrivalVehicles[20,3]: - 999999 x[20,3] + start_service[3]
   - start_service[20] >= -999995.2305121262
 NodeArrivalVehicles[20,4]: - 999999 x[20,4] + start_service[4]
   - start_service[20] >= -999975.7042393816
 NodeArrivalVehicles[20,5]: - 999999 x[20,5] + start_service[5]
   - start_service[20] >= -999955.8491601927
 NodeArrivalVehicles[20,6]: - 999999 x[20,6] + start_service[6]
   - start_service[20] >= -999984.2983734549
 NodeArrivalVehicles[20,7]: - 999999 x[20,7] + start_service[7]
   - start_service[20] >= -999962.2590264571
 NodeArrivalVehicles[20,8]: - 999999 x[20,8] + start_service[8]
   - start_service[20] >= -999969.1569164672
 NodeArrivalVehicles[20,9]: - 999999 x[20,9] + start_service[9]
   - start_service[20] >= -999954.0422286508
 NodeArrivalVehicles[20,10]: - 999999 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 999999 x[20,11] + start_service[11]
   - start_service[20] >= -999945.3398959702
 NodeArrivalVehicles[20,12]: - 999999 x[20,12] + start_service[12]
   - start_service[20] >= -999957.3214814937
 NodeArrivalVehicles[20,13]: - 999999 x[20,13] + start_service[13]
   - start_service[20] >= -999995.2305121262
 NodeArrivalVehicles[20,14]: - 999999 x[20,14] + start_service[14]
   - start_service[20] >= -999975.7042393816
 NodeArrivalVehicles[20,15]: - 999999 x[20,15] + start_service[15]
   - start_service[20] >= -999955.8491601927
 NodeArrivalVehicles[20,16]: - 999999 x[20,16] + start_service[16]
   - start_service[20] >= -999984.2983734549
 NodeArrivalVehicles[20,17]: - 999999 x[20,17] + start_service[17]
   - start_service[20] >= -999962.2590264571
 NodeArrivalVehicles[20,18]: - 999999 x[20,18] + start_service[18]
   - start_service[20] >= -999969.1569164672
 NodeArrivalVehicles[20,19]: - 999999 x[20,19] + start_service[19]
   - start_service[20] >= -999954.0422286508
 NodeArrivalVehicles[20,20]: - 999999 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 999999 x[20,21] - start_service[20]
   + start_service[21] >= -999945.3398959702
 NodeArrivalVehicles[20,22]: - 999999 x[20,22] - start_service[20]
   + start_service[22] >= -999973.4984481733
 NodeArrivalVehicles[20,23]: - 999999 x[20,23] - start_service[20]
   + start_service[23] >= -999973.4984481733
 NodeArrivalVehicles[21,1]: - 999999 x[21,1] + start_service[1]
   - start_service[21] >= -999922.9870655254
 NodeArrivalVehicles[21,2]: - 999999 x[21,2] + start_service[2]
   - start_service[21] >= -999924.0650949096
 NodeArrivalVehicles[21,3]: - 999999 x[21,3] + start_service[3]
   - start_service[21] >= -999943.0282623119
 NodeArrivalVehicles[21,4]: - 999999 x[21,4] + start_service[4]
   - start_service[21] >= -999926.9998154708
 NodeArrivalVehicles[21,5]: - 999999 x[21,5] + start_service[5]
   - start_service[21] >= -999974.0460374815
 NodeArrivalVehicles[21,6]: - 999999 x[21,6] + start_service[6]
   - start_service[21] >= -999953.8825736255
 NodeArrivalVehicles[21,7]: - 999999 x[21,7] + start_service[7]
   - start_service[21] >= -999959.5953207669
 NodeArrivalVehicles[21,8]: - 999999 x[21,8] + start_service[8]
   - start_service[21] >= -999922.8849691104
 NodeArrivalVehicles[21,9]: - 999999 x[21,9] + start_service[9]
   - start_service[21] >= -999930.9304394979
 NodeArrivalVehicles[21,10]: - 999999 x[21,10] + start_service[10]
   - start_service[21] >= -999945.3398959702
 NodeArrivalVehicles[21,11]: - 999999 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 999999 x[21,12] + start_service[12]
   - start_service[21] >= -999924.0650949096
 NodeArrivalVehicles[21,13]: - 999999 x[21,13] + start_service[13]
   - start_service[21] >= -999943.0282623119
 NodeArrivalVehicles[21,14]: - 999999 x[21,14] + start_service[14]
   - start_service[21] >= -999926.9998154708
 NodeArrivalVehicles[21,15]: - 999999 x[21,15] + start_service[15]
   - start_service[21] >= -999974.0460374815
 NodeArrivalVehicles[21,16]: - 999999 x[21,16] + start_service[16]
   - start_service[21] >= -999953.8825736255
 NodeArrivalVehicles[21,17]: - 999999 x[21,17] + start_service[17]
   - start_service[21] >= -999959.5953207669
 NodeArrivalVehicles[21,18]: - 999999 x[21,18] + start_service[18]
   - start_service[21] >= -999922.8849691104
 NodeArrivalVehicles[21,19]: - 999999 x[21,19] + start_service[19]
   - start_service[21] >= -999930.9304394979
 NodeArrivalVehicles[21,20]: - 999999 x[21,20] + start_service[20]
   - start_service[21] >= -999945.3398959702
 NodeArrivalVehicles[21,21]: - 999999 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 999999 x[21,22] - start_service[21]
   + start_service[22] >= -999922.9870655254
 NodeArrivalVehicles[21,23]: - 999999 x[21,23] - start_service[21]
   + start_service[23] >= -999922.9870655254
 NodeArrivalVehicles[22,1]: - 999999 x[22,1] + start_service[1]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 999999 x[22,2] + start_service[2]
   - start_service[22] >= -999970.4355137392
 NodeArrivalVehicles[22,3]: - 999999 x[22,3] + start_service[3]
   - start_service[22] >= -999977.0287494746
 NodeArrivalVehicles[22,4]: - 999999 x[22,4] + start_service[4]
   - start_service[22] >= -999994.052853037
 NodeArrivalVehicles[22,5]: - 999999 x[22,5] + start_service[5]
   - start_service[22] >= -999930.4872109075
 NodeArrivalVehicles[22,6]: - 999999 x[22,6] + start_service[6]
   - start_service[22] >= -999967.9589406843
 NodeArrivalVehicles[22,7]: - 999999 x[22,7] + start_service[7]
   - start_service[22] >= -999951.4555340747
 NodeArrivalVehicles[22,8]: - 999999 x[22,8] + start_service[8]
   - start_service[22] >= -999990.1581436468
 NodeArrivalVehicles[22,9]: - 999999 x[22,9] + start_service[9]
   - start_service[22] >= -999960.6560937124
 NodeArrivalVehicles[22,10]: - 999999 x[22,10] + start_service[10]
   - start_service[22] >= -999973.4984481733
 NodeArrivalVehicles[22,11]: - 999999 x[22,11] + start_service[11]
   - start_service[22] >= -999922.9870655254
 NodeArrivalVehicles[22,12]: - 999999 x[22,12] + start_service[12]
   - start_service[22] >= -999970.4355137392
 NodeArrivalVehicles[22,13]: - 999999 x[22,13] + start_service[13]
   - start_service[22] >= -999977.0287494746
 NodeArrivalVehicles[22,14]: - 999999 x[22,14] + start_service[14]
   - start_service[22] >= -999994.052853037
 NodeArrivalVehicles[22,15]: - 999999 x[22,15] + start_service[15]
   - start_service[22] >= -999930.4872109075
 NodeArrivalVehicles[22,16]: - 999999 x[22,16] + start_service[16]
   - start_service[22] >= -999967.9589406843
 NodeArrivalVehicles[22,17]: - 999999 x[22,17] + start_service[17]
   - start_service[22] >= -999951.4555340747
 NodeArrivalVehicles[22,18]: - 999999 x[22,18] + start_service[18]
   - start_service[22] >= -999990.1581436468
 NodeArrivalVehicles[22,19]: - 999999 x[22,19] + start_service[19]
   - start_service[22] >= -999960.6560937124
 NodeArrivalVehicles[22,20]: - 999999 x[22,20] + start_service[20]
   - start_service[22] >= -999973.4984481733
 NodeArrivalVehicles[22,21]: - 999999 x[22,21] + start_service[21]
   - start_service[22] >= -999922.9870655254
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
 StartServiceRobots[0,1]: - 999999 v[0,1] - start_service[0]
   + start_service[1] >= -999999
 StartServiceRobots[0,2]: - 999999 v[0,2] - start_service[0]
   + start_service[2] >= -999970.4355137392
 StartServiceRobots[0,3]: - 999999 v[0,3] - start_service[0]
   + start_service[3] >= -999977.0287494746
 StartServiceRobots[0,4]: - 999999 v[0,4] - start_service[0]
   + start_service[4] >= -999994.052853037
 StartServiceRobots[0,5]: - 999999 v[0,5] - start_service[0]
   + start_service[5] >= -999930.4872109075
 StartServiceRobots[0,6]: - 999999 v[0,6] - start_service[0]
   + start_service[6] >= -999967.9589406843
 StartServiceRobots[0,7]: - 999999 v[0,7] - start_service[0]
   + start_service[7] >= -999951.4555340747
 StartServiceRobots[0,8]: - 999999 v[0,8] - start_service[0]
   + start_service[8] >= -999990.1581436468
 StartServiceRobots[0,9]: - 999999 v[0,9] - start_service[0]
   + start_service[9] >= -999960.6560937124
 StartServiceRobots[0,10]: - 999999 v[0,10] - start_service[0]
   + start_service[10] >= -999973.4984481733
 StartServiceRobots[0,11]: - 999999 v[0,11] - start_service[0]
   + start_service[11] >= -999922.9870655254
 StartServiceRobots[0,12]: - 999999 v[0,12] - start_service[0]
   + start_service[12] >= -999970.4355137392
 StartServiceRobots[0,13]: - 999999 v[0,13] - start_service[0]
   + start_service[13] >= -999977.0287494746
 StartServiceRobots[0,14]: - 999999 v[0,14] - start_service[0]
   + start_service[14] >= -999994.052853037
 StartServiceRobots[0,15]: - 999999 v[0,15] - start_service[0]
   + start_service[15] >= -999930.4872109075
 StartServiceRobots[0,16]: - 999999 v[0,16] - start_service[0]
   + start_service[16] >= -999967.9589406843
 StartServiceRobots[0,17]: - 999999 v[0,17] - start_service[0]
   + start_service[17] >= -999951.4555340747
 StartServiceRobots[0,18]: - 999999 v[0,18] - start_service[0]
   + start_service[18] >= -999990.1581436468
 StartServiceRobots[0,19]: - 999999 v[0,19] - start_service[0]
   + start_service[19] >= -999960.6560937124
 StartServiceRobots[0,20]: - 999999 v[0,20] - start_service[0]
   + start_service[20] >= -999973.4984481733
 StartServiceRobots[0,21]: - 999999 v[0,21] - start_service[0]
   + start_service[21] >= -999922.9870655254
 StartServiceRobots[0,22]: - 999999 v[0,22] - start_service[0]
   + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999 v[0,23] - start_service[0]
   + start_service[23] >= -999999
 StartServiceRobots[1,0]: - 999999 v[1,0] + start_service[0]
   - start_service[1] >= -999999
 StartServiceRobots[1,2]: - 999999 v[1,2] - start_service[1]
   + start_service[2] >= -999970.4355137392
 StartServiceRobots[1,3]: - 999999 v[1,3] - start_service[1]
   + start_service[3] >= -999977.0287494746
 StartServiceRobots[1,4]: - 999999 v[1,4] - start_service[1]
   + start_service[4] >= -999994.052853037
 StartServiceRobots[1,5]: - 999999 v[1,5] - start_service[1]
   + start_service[5] >= -999930.4872109075
 StartServiceRobots[1,6]: - 999999 v[1,6] - start_service[1]
   + start_service[6] >= -999967.9589406843
 StartServiceRobots[1,7]: - 999999 v[1,7] - start_service[1]
   + start_service[7] >= -999951.4555340747
 StartServiceRobots[1,8]: - 999999 v[1,8] - start_service[1]
   + start_service[8] >= -999990.1581436468
 StartServiceRobots[1,9]: - 999999 v[1,9] - start_service[1]
   + start_service[9] >= -999960.6560937124
 StartServiceRobots[1,10]: - 999999 v[1,10] - start_service[1]
   + start_service[10] >= -999973.4984481733
 StartServiceRobots[1,11]: - 999999 v[1,11] - start_service[1]
   + start_service[11] >= -999922.9870655254
 StartServiceRobots[1,12]: - 999999 v[1,12] - start_service[1]
   + start_service[12] >= -999970.4355137392
 StartServiceRobots[1,13]: - 999999 v[1,13] - start_service[1]
   + start_service[13] >= -999977.0287494746
 StartServiceRobots[1,14]: - 999999 v[1,14] - start_service[1]
   + start_service[14] >= -999994.052853037
 StartServiceRobots[1,15]: - 999999 v[1,15] - start_service[1]
   + start_service[15] >= -999930.4872109075
 StartServiceRobots[1,16]: - 999999 v[1,16] - start_service[1]
   + start_service[16] >= -999967.9589406843
 StartServiceRobots[1,17]: - 999999 v[1,17] - start_service[1]
   + start_service[17] >= -999951.4555340747
 StartServiceRobots[1,18]: - 999999 v[1,18] - start_service[1]
   + start_service[18] >= -999990.1581436468
 StartServiceRobots[1,19]: - 999999 v[1,19] - start_service[1]
   + start_service[19] >= -999960.6560937124
 StartServiceRobots[1,20]: - 999999 v[1,20] - start_service[1]
   + start_service[20] >= -999973.4984481733
 StartServiceRobots[1,21]: - 999999 v[1,21] - start_service[1]
   + start_service[21] >= -999922.9870655254
 StartServiceRobots[1,22]: - 999999 v[1,22] - start_service[1]
   + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999 v[1,23] - start_service[1]
   + start_service[23] >= -999999
 StartServiceRobots[2,0]: - 999999 v[2,0] + start_service[0]
   - start_service[2] >= -999970.4355137392
 StartServiceRobots[2,1]: - 999999 v[2,1] + start_service[1]
   - start_service[2] >= -999970.4355137392
 StartServiceRobots[2,3]: - 999999 v[2,3] - start_service[2]
   + start_service[3] >= -999960.7530661469
 StartServiceRobots[2,4]: - 999999 v[2,4] - start_service[2]
   + start_service[4] >= -999973.8738751572
 StartServiceRobots[2,5]: - 999999 v[2,5] - start_service[2]
   + start_service[5] >= -999922.0828731196
 StartServiceRobots[2,6]: - 999999 v[2,6] - start_service[2]
   + start_service[6] >= -999963.6131301826
 StartServiceRobots[2,7]: - 999999 v[2,7] - start_service[2]
   + start_service[7] >= -999962.1348413215
 StartServiceRobots[2,8]: - 999999 v[2,8] - start_service[2]
   + start_service[8] >= -999979.0699702846
 StartServiceRobots[2,9]: - 999999 v[2,9] - start_service[2]
   + start_service[9] >= -999986.4581145474
 StartServiceRobots[2,10]: - 999999 v[2,10] - start_service[2]
   + start_service[10] >= -999957.3214814937
 StartServiceRobots[2,11]: - 999999 v[2,11] - start_service[2]
   + start_service[11] >= -999924.0650949096
 StartServiceRobots[2,13]: - 999999 v[2,13] - start_service[2]
   + start_service[13] >= -999960.7530661469
 StartServiceRobots[2,14]: - 999999 v[2,14] - start_service[2]
   + start_service[14] >= -999973.8738751572
 StartServiceRobots[2,15]: - 999999 v[2,15] - start_service[2]
   + start_service[15] >= -999922.0828731196
 StartServiceRobots[2,16]: - 999999 v[2,16] - start_service[2]
   + start_service[16] >= -999963.6131301826
 StartServiceRobots[2,17]: - 999999 v[2,17] - start_service[2]
   + start_service[17] >= -999962.1348413215
 StartServiceRobots[2,18]: - 999999 v[2,18] - start_service[2]
   + start_service[18] >= -999979.0699702846
 StartServiceRobots[2,19]: - 999999 v[2,19] - start_service[2]
   + start_service[19] >= -999986.4581145474
 StartServiceRobots[2,20]: - 999999 v[2,20] - start_service[2]
   + start_service[20] >= -999957.3214814937
 StartServiceRobots[2,21]: - 999999 v[2,21] - start_service[2]
   + start_service[21] >= -999924.0650949096
 StartServiceRobots[2,22]: - 999999 v[2,22] - start_service[2]
   + start_service[22] >= -999970.4355137392
 StartServiceRobots[2,23]: - 999999 v[2,23] - start_service[2]
   + start_service[23] >= -999970.4355137392
 StartServiceRobots[3,0]: - 999999 v[3,0] + start_service[0]
   - start_service[3] >= -999977.0287494746
 StartServiceRobots[3,1]: - 999999 v[3,1] + start_service[1]
   - start_service[3] >= -999977.0287494746
 StartServiceRobots[3,2]: - 999999 v[3,2] + start_service[2]
   - start_service[3] >= -999960.7530661469
 StartServiceRobots[3,4]: - 999999 v[3,4] - start_service[3]
   + start_service[4] >= -999979.4237314209
 StartServiceRobots[3,5]: - 999999 v[3,5] - start_service[3]
   + start_service[5] >= -999952.4465287814
 StartServiceRobots[3,6]: - 999999 v[3,6] - start_service[3]
   + start_service[6] >= -999984.5718729242
 StartServiceRobots[3,7]: - 999999 v[3,7] - start_service[3]
   + start_service[7] >= -999962.7885731502
 StartServiceRobots[3,8]: - 999999 v[3,8] - start_service[3]
   + start_service[8] >= -999972.9207620689
 StartServiceRobots[3,9]: - 999999 v[3,9] - start_service[3]
   + start_service[9] >= -999956.8858705129
 StartServiceRobots[3,10]: - 999999 v[3,10] - start_service[3]
   + start_service[10] >= -999995.2305121262
 StartServiceRobots[3,11]: - 999999 v[3,11] - start_service[3]
   + start_service[11] >= -999943.0282623119
 StartServiceRobots[3,12]: - 999999 v[3,12] - start_service[3]
   + start_service[12] >= -999960.7530661469
 StartServiceRobots[3,14]: - 999999 v[3,14] - start_service[3]
   + start_service[14] >= -999979.4237314209
 StartServiceRobots[3,15]: - 999999 v[3,15] - start_service[3]
   + start_service[15] >= -999952.4465287814
 StartServiceRobots[3,16]: - 999999 v[3,16] - start_service[3]
   + start_service[16] >= -999984.5718729242
 StartServiceRobots[3,17]: - 999999 v[3,17] - start_service[3]
   + start_service[17] >= -999962.7885731502
 StartServiceRobots[3,18]: - 999999 v[3,18] - start_service[3]
   + start_service[18] >= -999972.9207620689
 StartServiceRobots[3,19]: - 999999 v[3,19] - start_service[3]
   + start_service[19] >= -999956.8858705129
 StartServiceRobots[3,20]: - 999999 v[3,20] - start_service[3]
   + start_service[20] >= -999995.2305121262
 StartServiceRobots[3,21]: - 999999 v[3,21] - start_service[3]
   + start_service[21] >= -999943.0282623119
 StartServiceRobots[3,22]: - 999999 v[3,22] - start_service[3]
   + start_service[22] >= -999977.0287494746
 StartServiceRobots[3,23]: - 999999 v[3,23] - start_service[3]
   + start_service[23] >= -999977.0287494746
 StartServiceRobots[4,0]: - 999999 v[4,0] + start_service[0]
   - start_service[4] >= -999994.052853037
 StartServiceRobots[4,1]: - 999999 v[4,1] + start_service[1]
   - start_service[4] >= -999994.052853037
 StartServiceRobots[4,2]: - 999999 v[4,2] + start_service[2]
   - start_service[4] >= -999973.8738751572
 StartServiceRobots[4,3]: - 999999 v[4,3] + start_service[3]
   - start_service[4] >= -999979.4237314209
 StartServiceRobots[4,5]: - 999999 v[4,5] - start_service[4]
   + start_service[5] >= -999933.296967469
 StartServiceRobots[4,6]: - 999999 v[4,6] - start_service[4]
   + start_service[6] >= -999972.10830705
 StartServiceRobots[4,7]: - 999999 v[4,7] - start_service[4]
   + start_service[7] >= -999956.3421461377
 StartServiceRobots[4,8]: - 999999 v[4,8] - start_service[4]
   + start_service[8] >= -999991.9597178316
 StartServiceRobots[4,9]: - 999999 v[4,9] - start_service[4]
   + start_service[9] >= -999964.9144336868
 StartServiceRobots[4,10]: - 999999 v[4,10] - start_service[4]
   + start_service[10] >= -999975.7042393816
 StartServiceRobots[4,11]: - 999999 v[4,11] - start_service[4]
   + start_service[11] >= -999926.9998154708
 StartServiceRobots[4,12]: - 999999 v[4,12] - start_service[4]
   + start_service[12] >= -999973.8738751572
 StartServiceRobots[4,13]: - 999999 v[4,13] - start_service[4]
   + start_service[13] >= -999979.4237314209
 StartServiceRobots[4,15]: - 999999 v[4,15] - start_service[4]
   + start_service[15] >= -999933.296967469
 StartServiceRobots[4,16]: - 999999 v[4,16] - start_service[4]
   + start_service[16] >= -999972.10830705
 StartServiceRobots[4,17]: - 999999 v[4,17] - start_service[4]
   + start_service[17] >= -999956.3421461377
 StartServiceRobots[4,18]: - 999999 v[4,18] - start_service[4]
   + start_service[18] >= -999991.9597178316
 StartServiceRobots[4,19]: - 999999 v[4,19] - start_service[4]
   + start_service[19] >= -999964.9144336868
 StartServiceRobots[4,20]: - 999999 v[4,20] - start_service[4]
   + start_service[20] >= -999975.7042393816
 StartServiceRobots[4,21]: - 999999 v[4,21] - start_service[4]
   + start_service[21] >= -999926.9998154708
 StartServiceRobots[4,22]: - 999999 v[4,22] - start_service[4]
   + start_service[22] >= -999994.052853037
 StartServiceRobots[4,23]: - 999999 v[4,23] - start_service[4]
   + start_service[23] >= -999994.052853037
 StartServiceRobots[5,0]: - 999999 v[5,0] + start_service[0]
   - start_service[5] >= -999930.4872109075
 StartServiceRobots[5,1]: - 999999 v[5,1] + start_service[1]
   - start_service[5] >= -999930.4872109075
 StartServiceRobots[5,2]: - 999999 v[5,2] + start_service[2]
   - start_service[5] >= -999922.0828731196
 StartServiceRobots[5,3]: - 999999 v[5,3] + start_service[3]
   - start_service[5] >= -999952.4465287814
 StartServiceRobots[5,4]: - 999999 v[5,4] + start_service[4]
   - start_service[5] >= -999933.296967469
 StartServiceRobots[5,6]: - 999999 v[5,6] - start_service[5]
   + start_service[6] >= -999957.1706496663
 StartServiceRobots[5,7]: - 999999 v[5,7] - start_service[5]
   + start_service[7] >= -999950.3991247974
 StartServiceRobots[5,8]: - 999999 v[5,8] - start_service[5]
   + start_service[8] >= -999927.4985292086
 StartServiceRobots[5,9]: - 999999 v[5,9] - start_service[5]
   + start_service[9] >= -999925.0867816372
 StartServiceRobots[5,10]: - 999999 v[5,10] - start_service[5]
   + start_service[10] >= -999955.8491601927
 StartServiceRobots[5,11]: - 999999 v[5,11] - start_service[5]
   + start_service[11] >= -999974.0460374815
 StartServiceRobots[5,12]: - 999999 v[5,12] - start_service[5]
   + start_service[12] >= -999922.0828731196
 StartServiceRobots[5,13]: - 999999 v[5,13] - start_service[5]
   + start_service[13] >= -999952.4465287814
 StartServiceRobots[5,14]: - 999999 v[5,14] - start_service[5]
   + start_service[14] >= -999933.296967469
 StartServiceRobots[5,16]: - 999999 v[5,16] - start_service[5]
   + start_service[16] >= -999957.1706496663
 StartServiceRobots[5,17]: - 999999 v[5,17] - start_service[5]
   + start_service[17] >= -999950.3991247974
 StartServiceRobots[5,18]: - 999999 v[5,18] - start_service[5]
   + start_service[18] >= -999927.4985292086
 StartServiceRobots[5,19]: - 999999 v[5,19] - start_service[5]
   + start_service[19] >= -999925.0867816372
 StartServiceRobots[5,20]: - 999999 v[5,20] - start_service[5]
   + start_service[20] >= -999955.8491601927
 StartServiceRobots[5,21]: - 999999 v[5,21] - start_service[5]
   + start_service[21] >= -999974.0460374815
 StartServiceRobots[5,22]: - 999999 v[5,22] - start_service[5]
   + start_service[22] >= -999930.4872109075
 StartServiceRobots[5,23]: - 999999 v[5,23] - start_service[5]
   + start_service[23] >= -999930.4872109075
 StartServiceRobots[6,0]: - 999999 v[6,0] + start_service[0]
   - start_service[6] >= -999967.9589406843
 StartServiceRobots[6,1]: - 999999 v[6,1] + start_service[1]
   - start_service[6] >= -999967.9589406843
 StartServiceRobots[6,2]: - 999999 v[6,2] + start_service[2]
   - start_service[6] >= -999963.6131301826
 StartServiceRobots[6,3]: - 999999 v[6,3] + start_service[3]
   - start_service[6] >= -999984.5718729242
 StartServiceRobots[6,4]: - 999999 v[6,4] + start_service[4]
   - start_service[6] >= -999972.10830705
 StartServiceRobots[6,5]: - 999999 v[6,5] + start_service[5]
   - start_service[6] >= -999957.1706496663
 StartServiceRobots[6,7]: - 999999 v[6,7] - start_service[6]
   + start_service[7] >= -999976.9595480452
 StartServiceRobots[6,8]: - 999999 v[6,8] - start_service[6]
   + start_service[8] >= -999967.7691443588
 StartServiceRobots[6,9]: - 999999 v[6,9] - start_service[6]
   + start_service[9] >= -999964.2077640658
 StartServiceRobots[6,10]: - 999999 v[6,10] - start_service[6]
   + start_service[10] >= -999984.2983734549
 StartServiceRobots[6,11]: - 999999 v[6,11] - start_service[6]
   + start_service[11] >= -999953.8825736255
 StartServiceRobots[6,12]: - 999999 v[6,12] - start_service[6]
   + start_service[12] >= -999963.6131301826
 StartServiceRobots[6,13]: - 999999 v[6,13] - start_service[6]
   + start_service[13] >= -999984.5718729242
 StartServiceRobots[6,14]: - 999999 v[6,14] - start_service[6]
   + start_service[14] >= -999972.10830705
 StartServiceRobots[6,15]: - 999999 v[6,15] - start_service[6]
   + start_service[15] >= -999957.1706496663
 StartServiceRobots[6,17]: - 999999 v[6,17] - start_service[6]
   + start_service[17] >= -999976.9595480452
 StartServiceRobots[6,18]: - 999999 v[6,18] - start_service[6]
   + start_service[18] >= -999967.7691443588
 StartServiceRobots[6,19]: - 999999 v[6,19] - start_service[6]
   + start_service[19] >= -999964.2077640658
 StartServiceRobots[6,20]: - 999999 v[6,20] - start_service[6]
   + start_service[20] >= -999984.2983734549
 StartServiceRobots[6,21]: - 999999 v[6,21] - start_service[6]
   + start_service[21] >= -999953.8825736255
 StartServiceRobots[6,22]: - 999999 v[6,22] - start_service[6]
   + start_service[22] >= -999967.9589406843
 StartServiceRobots[6,23]: - 999999 v[6,23] - start_service[6]
   + start_service[23] >= -999967.9589406843
 StartServiceRobots[7,0]: - 999999 v[7,0] + start_service[0]
   - start_service[7] >= -999951.4555340747
 StartServiceRobots[7,1]: - 999999 v[7,1] + start_service[1]
   - start_service[7] >= -999951.4555340747
 StartServiceRobots[7,2]: - 999999 v[7,2] + start_service[2]
   - start_service[7] >= -999962.1348413215
 StartServiceRobots[7,3]: - 999999 v[7,3] + start_service[3]
   - start_service[7] >= -999962.7885731502
 StartServiceRobots[7,4]: - 999999 v[7,4] + start_service[4]
   - start_service[7] >= -999956.3421461377
 StartServiceRobots[7,5]: - 999999 v[7,5] + start_service[5]
   - start_service[7] >= -999950.3991247974
 StartServiceRobots[7,6]: - 999999 v[7,6] + start_service[6]
   - start_service[7] >= -999976.9595480452
 StartServiceRobots[7,8]: - 999999 v[7,8] - start_service[7]
   + start_service[8] >= -999955.0366450849
 StartServiceRobots[7,9]: - 999999 v[7,9] - start_service[7]
   + start_service[9] >= -999970.2808806106
 StartServiceRobots[7,10]: - 999999 v[7,10] - start_service[7]
   + start_service[10] >= -999962.2590264571
 StartServiceRobots[7,11]: - 999999 v[7,11] - start_service[7]
   + start_service[11] >= -999959.5953207669
 StartServiceRobots[7,12]: - 999999 v[7,12] - start_service[7]
   + start_service[12] >= -999962.1348413215
 StartServiceRobots[7,13]: - 999999 v[7,13] - start_service[7]
   + start_service[13] >= -999962.7885731502
 StartServiceRobots[7,14]: - 999999 v[7,14] - start_service[7]
   + start_service[14] >= -999956.3421461377
 StartServiceRobots[7,15]: - 999999 v[7,15] - start_service[7]
   + start_service[15] >= -999950.3991247974
 StartServiceRobots[7,16]: - 999999 v[7,16] - start_service[7]
   + start_service[16] >= -999976.9595480452
 StartServiceRobots[7,18]: - 999999 v[7,18] - start_service[7]
   + start_service[18] >= -999955.0366450849
 StartServiceRobots[7,19]: - 999999 v[7,19] - start_service[7]
   + start_service[19] >= -999970.2808806106
 StartServiceRobots[7,20]: - 999999 v[7,20] - start_service[7]
   + start_service[20] >= -999962.2590264571
 StartServiceRobots[7,21]: - 999999 v[7,21] - start_service[7]
   + start_service[21] >= -999959.5953207669
 StartServiceRobots[7,22]: - 999999 v[7,22] - start_service[7]
   + start_service[22] >= -999951.4555340747
 StartServiceRobots[7,23]: - 999999 v[7,23] - start_service[7]
   + start_service[23] >= -999951.4555340747
 StartServiceRobots[8,0]: - 999999 v[8,0] + start_service[0]
   - start_service[8] >= -999990.1581436468
 StartServiceRobots[8,1]: - 999999 v[8,1] + start_service[1]
   - start_service[8] >= -999990.1581436468
 StartServiceRobots[8,2]: - 999999 v[8,2] + start_service[2]
   - start_service[8] >= -999979.0699702846
 StartServiceRobots[8,3]: - 999999 v[8,3] + start_service[3]
   - start_service[8] >= -999972.9207620689
 StartServiceRobots[8,4]: - 999999 v[8,4] + start_service[4]
   - start_service[8] >= -999991.9597178316
 StartServiceRobots[8,5]: - 999999 v[8,5] + start_service[5]
   - start_service[8] >= -999927.4985292086
 StartServiceRobots[8,6]: - 999999 v[8,6] + start_service[6]
   - start_service[8] >= -999967.7691443588
 StartServiceRobots[8,7]: - 999999 v[8,7] + start_service[7]
   - start_service[8] >= -999955.0366450849
 StartServiceRobots[8,9]: - 999999 v[8,9] - start_service[8]
   + start_service[9] >= -999968.5222110867
 StartServiceRobots[8,10]: - 999999 v[8,10] - start_service[8]
   + start_service[10] >= -999969.1569164672
 StartServiceRobots[8,11]: - 999999 v[8,11] - start_service[8]
   + start_service[11] >= -999922.8849691104
 StartServiceRobots[8,12]: - 999999 v[8,12] - start_service[8]
   + start_service[12] >= -999979.0699702846
 StartServiceRobots[8,13]: - 999999 v[8,13] - start_service[8]
   + start_service[13] >= -999972.9207620689
 StartServiceRobots[8,14]: - 999999 v[8,14] - start_service[8]
   + start_service[14] >= -999991.9597178316
 StartServiceRobots[8,15]: - 999999 v[8,15] - start_service[8]
   + start_service[15] >= -999927.4985292086
 StartServiceRobots[8,16]: - 999999 v[8,16] - start_service[8]
   + start_service[16] >= -999967.7691443588
 StartServiceRobots[8,17]: - 999999 v[8,17] - start_service[8]
   + start_service[17] >= -999955.0366450849
 StartServiceRobots[8,19]: - 999999 v[8,19] - start_service[8]
   + start_service[19] >= -999968.5222110867
 StartServiceRobots[8,20]: - 999999 v[8,20] - start_service[8]
   + start_service[20] >= -999969.1569164672
 StartServiceRobots[8,21]: - 999999 v[8,21] - start_service[8]
   + start_service[21] >= -999922.8849691104
 StartServiceRobots[8,22]: - 999999 v[8,22] - start_service[8]
   + start_service[22] >= -999990.1581436468
 StartServiceRobots[8,23]: - 999999 v[8,23] - start_service[8]
   + start_service[23] >= -999990.1581436468
 StartServiceRobots[9,0]: - 999999 v[9,0] + start_service[0]
   - start_service[9] >= -999960.6560937124
 StartServiceRobots[9,1]: - 999999 v[9,1] + start_service[1]
   - start_service[9] >= -999960.6560937124
 StartServiceRobots[9,2]: - 999999 v[9,2] + start_service[2]
   - start_service[9] >= -999986.4581145474
 StartServiceRobots[9,3]: - 999999 v[9,3] + start_service[3]
   - start_service[9] >= -999956.8858705129
 StartServiceRobots[9,4]: - 999999 v[9,4] + start_service[4]
   - start_service[9] >= -999964.9144336868
 StartServiceRobots[9,5]: - 999999 v[9,5] + start_service[5]
   - start_service[9] >= -999925.0867816372
 StartServiceRobots[9,6]: - 999999 v[9,6] + start_service[6]
   - start_service[9] >= -999964.2077640658
 StartServiceRobots[9,7]: - 999999 v[9,7] + start_service[7]
   - start_service[9] >= -999970.2808806106
 StartServiceRobots[9,8]: - 999999 v[9,8] + start_service[8]
   - start_service[9] >= -999968.5222110867
 StartServiceRobots[9,10]: - 999999 v[9,10] - start_service[9]
   + start_service[10] >= -999954.0422286508
 StartServiceRobots[9,11]: - 999999 v[9,11] - start_service[9]
   + start_service[11] >= -999930.9304394979
 StartServiceRobots[9,12]: - 999999 v[9,12] - start_service[9]
   + start_service[12] >= -999986.4581145474
 StartServiceRobots[9,13]: - 999999 v[9,13] - start_service[9]
   + start_service[13] >= -999956.8858705129
 StartServiceRobots[9,14]: - 999999 v[9,14] - start_service[9]
   + start_service[14] >= -999964.9144336868
 StartServiceRobots[9,15]: - 999999 v[9,15] - start_service[9]
   + start_service[15] >= -999925.0867816372
 StartServiceRobots[9,16]: - 999999 v[9,16] - start_service[9]
   + start_service[16] >= -999964.2077640658
 StartServiceRobots[9,17]: - 999999 v[9,17] - start_service[9]
   + start_service[17] >= -999970.2808806106
 StartServiceRobots[9,18]: - 999999 v[9,18] - start_service[9]
   + start_service[18] >= -999968.5222110867
 StartServiceRobots[9,20]: - 999999 v[9,20] - start_service[9]
   + start_service[20] >= -999954.0422286508
 StartServiceRobots[9,21]: - 999999 v[9,21] - start_service[9]
   + start_service[21] >= -999930.9304394979
 StartServiceRobots[9,22]: - 999999 v[9,22] - start_service[9]
   + start_service[22] >= -999960.6560937124
 StartServiceRobots[9,23]: - 999999 v[9,23] - start_service[9]
   + start_service[23] >= -999960.6560937124
 StartServiceRobots[10,0]: - 999999 v[10,0] + start_service[0]
   - start_service[10] >= -999973.4984481733
 StartServiceRobots[10,1]: - 999999 v[10,1] + start_service[1]
   - start_service[10] >= -999973.4984481733
 StartServiceRobots[10,2]: - 999999 v[10,2] + start_service[2]
   - start_service[10] >= -999957.3214814937
 StartServiceRobots[10,3]: - 999999 v[10,3] + start_service[3]
   - start_service[10] >= -999995.2305121262
 StartServiceRobots[10,4]: - 999999 v[10,4] + start_service[4]
   - start_service[10] >= -999975.7042393816
 StartServiceRobots[10,5]: - 999999 v[10,5] + start_service[5]
   - start_service[10] >= -999955.8491601927
 StartServiceRobots[10,6]: - 999999 v[10,6] + start_service[6]
   - start_service[10] >= -999984.2983734549
 StartServiceRobots[10,7]: - 999999 v[10,7] + start_service[7]
   - start_service[10] >= -999962.2590264571
 StartServiceRobots[10,8]: - 999999 v[10,8] + start_service[8]
   - start_service[10] >= -999969.1569164672
 StartServiceRobots[10,9]: - 999999 v[10,9] + start_service[9]
   - start_service[10] >= -999954.0422286508
 StartServiceRobots[10,11]: - 999999 v[10,11] - start_service[10]
   + start_service[11] >= -999945.3398959702
 StartServiceRobots[10,12]: - 999999 v[10,12] - start_service[10]
   + start_service[12] >= -999957.3214814937
 StartServiceRobots[10,13]: - 999999 v[10,13] - start_service[10]
   + start_service[13] >= -999995.2305121262
 StartServiceRobots[10,14]: - 999999 v[10,14] - start_service[10]
   + start_service[14] >= -999975.7042393816
 StartServiceRobots[10,15]: - 999999 v[10,15] - start_service[10]
   + start_service[15] >= -999955.8491601927
 StartServiceRobots[10,16]: - 999999 v[10,16] - start_service[10]
   + start_service[16] >= -999984.2983734549
 StartServiceRobots[10,17]: - 999999 v[10,17] - start_service[10]
   + start_service[17] >= -999962.2590264571
 StartServiceRobots[10,18]: - 999999 v[10,18] - start_service[10]
   + start_service[18] >= -999969.1569164672
 StartServiceRobots[10,19]: - 999999 v[10,19] - start_service[10]
   + start_service[19] >= -999954.0422286508
 StartServiceRobots[10,21]: - 999999 v[10,21] - start_service[10]
   + start_service[21] >= -999945.3398959702
 StartServiceRobots[10,22]: - 999999 v[10,22] - start_service[10]
   + start_service[22] >= -999973.4984481733
 StartServiceRobots[10,23]: - 999999 v[10,23] - start_service[10]
   + start_service[23] >= -999973.4984481733
 StartServiceRobots[11,0]: - 999999 v[11,0] + start_service[0]
   - start_service[11] >= -999922.9870655254
 StartServiceRobots[11,1]: - 999999 v[11,1] + start_service[1]
   - start_service[11] >= -999922.9870655254
 StartServiceRobots[11,2]: - 999999 v[11,2] + start_service[2]
   - start_service[11] >= -999924.0650949096
 StartServiceRobots[11,3]: - 999999 v[11,3] + start_service[3]
   - start_service[11] >= -999943.0282623119
 StartServiceRobots[11,4]: - 999999 v[11,4] + start_service[4]
   - start_service[11] >= -999926.9998154708
 StartServiceRobots[11,5]: - 999999 v[11,5] + start_service[5]
   - start_service[11] >= -999974.0460374815
 StartServiceRobots[11,6]: - 999999 v[11,6] + start_service[6]
   - start_service[11] >= -999953.8825736255
 StartServiceRobots[11,7]: - 999999 v[11,7] + start_service[7]
   - start_service[11] >= -999959.5953207669
 StartServiceRobots[11,8]: - 999999 v[11,8] + start_service[8]
   - start_service[11] >= -999922.8849691104
 StartServiceRobots[11,9]: - 999999 v[11,9] + start_service[9]
   - start_service[11] >= -999930.9304394979
 StartServiceRobots[11,10]: - 999999 v[11,10] + start_service[10]
   - start_service[11] >= -999945.3398959702
 StartServiceRobots[11,12]: - 999999 v[11,12] - start_service[11]
   + start_service[12] >= -999924.0650949096
 StartServiceRobots[11,13]: - 999999 v[11,13] - start_service[11]
   + start_service[13] >= -999943.0282623119
 StartServiceRobots[11,14]: - 999999 v[11,14] - start_service[11]
   + start_service[14] >= -999926.9998154708
 StartServiceRobots[11,15]: - 999999 v[11,15] - start_service[11]
   + start_service[15] >= -999974.0460374815
 StartServiceRobots[11,16]: - 999999 v[11,16] - start_service[11]
   + start_service[16] >= -999953.8825736255
 StartServiceRobots[11,17]: - 999999 v[11,17] - start_service[11]
   + start_service[17] >= -999959.5953207669
 StartServiceRobots[11,18]: - 999999 v[11,18] - start_service[11]
   + start_service[18] >= -999922.8849691104
 StartServiceRobots[11,19]: - 999999 v[11,19] - start_service[11]
   + start_service[19] >= -999930.9304394979
 StartServiceRobots[11,20]: - 999999 v[11,20] - start_service[11]
   + start_service[20] >= -999945.3398959702
 StartServiceRobots[11,22]: - 999999 v[11,22] - start_service[11]
   + start_service[22] >= -999922.9870655254
 StartServiceRobots[11,23]: - 999999 v[11,23] - start_service[11]
   + start_service[23] >= -999922.9870655254
 StartServiceRobots[12,0]: - 999999 v[12,0] + start_service[0]
   - start_service[12] >= -999970.4355137392
 StartServiceRobots[12,1]: - 999999 v[12,1] + start_service[1]
   - start_service[12] >= -999970.4355137392
 StartServiceRobots[12,3]: - 999999 v[12,3] + start_service[3]
   - start_service[12] >= -999960.7530661469
 StartServiceRobots[12,4]: - 999999 v[12,4] + start_service[4]
   - start_service[12] >= -999973.8738751572
 StartServiceRobots[12,5]: - 999999 v[12,5] + start_service[5]
   - start_service[12] >= -999922.0828731196
 StartServiceRobots[12,6]: - 999999 v[12,6] + start_service[6]
   - start_service[12] >= -999963.6131301826
 StartServiceRobots[12,7]: - 999999 v[12,7] + start_service[7]
   - start_service[12] >= -999962.1348413215
 StartServiceRobots[12,8]: - 999999 v[12,8] + start_service[8]
   - start_service[12] >= -999979.0699702846
 StartServiceRobots[12,9]: - 999999 v[12,9] + start_service[9]
   - start_service[12] >= -999986.4581145474
 StartServiceRobots[12,10]: - 999999 v[12,10] + start_service[10]
   - start_service[12] >= -999957.3214814937
 StartServiceRobots[12,11]: - 999999 v[12,11] + start_service[11]
   - start_service[12] >= -999924.0650949096
 StartServiceRobots[12,13]: - 999999 v[12,13] - start_service[12]
   + start_service[13] >= -999960.7530661469
 StartServiceRobots[12,14]: - 999999 v[12,14] - start_service[12]
   + start_service[14] >= -999973.8738751572
 StartServiceRobots[12,15]: - 999999 v[12,15] - start_service[12]
   + start_service[15] >= -999922.0828731196
 StartServiceRobots[12,16]: - 999999 v[12,16] - start_service[12]
   + start_service[16] >= -999963.6131301826
 StartServiceRobots[12,17]: - 999999 v[12,17] - start_service[12]
   + start_service[17] >= -999962.1348413215
 StartServiceRobots[12,18]: - 999999 v[12,18] - start_service[12]
   + start_service[18] >= -999979.0699702846
 StartServiceRobots[12,19]: - 999999 v[12,19] - start_service[12]
   + start_service[19] >= -999986.4581145474
 StartServiceRobots[12,20]: - 999999 v[12,20] - start_service[12]
   + start_service[20] >= -999957.3214814937
 StartServiceRobots[12,21]: - 999999 v[12,21] - start_service[12]
   + start_service[21] >= -999924.0650949096
 StartServiceRobots[12,22]: - 999999 v[12,22] - start_service[12]
   + start_service[22] >= -999970.4355137392
 StartServiceRobots[12,23]: - 999999 v[12,23] - start_service[12]
   + start_service[23] >= -999970.4355137392
 StartServiceRobots[13,0]: - 999999 v[13,0] + start_service[0]
   - start_service[13] >= -999977.0287494746
 StartServiceRobots[13,1]: - 999999 v[13,1] + start_service[1]
   - start_service[13] >= -999977.0287494746
 StartServiceRobots[13,2]: - 999999 v[13,2] + start_service[2]
   - start_service[13] >= -999960.7530661469
 StartServiceRobots[13,4]: - 999999 v[13,4] + start_service[4]
   - start_service[13] >= -999979.4237314209
 StartServiceRobots[13,5]: - 999999 v[13,5] + start_service[5]
   - start_service[13] >= -999952.4465287814
 StartServiceRobots[13,6]: - 999999 v[13,6] + start_service[6]
   - start_service[13] >= -999984.5718729242
 StartServiceRobots[13,7]: - 999999 v[13,7] + start_service[7]
   - start_service[13] >= -999962.7885731502
 StartServiceRobots[13,8]: - 999999 v[13,8] + start_service[8]
   - start_service[13] >= -999972.9207620689
 StartServiceRobots[13,9]: - 999999 v[13,9] + start_service[9]
   - start_service[13] >= -999956.8858705129
 StartServiceRobots[13,10]: - 999999 v[13,10] + start_service[10]
   - start_service[13] >= -999995.2305121262
 StartServiceRobots[13,11]: - 999999 v[13,11] + start_service[11]
   - start_service[13] >= -999943.0282623119
 StartServiceRobots[13,12]: - 999999 v[13,12] + start_service[12]
   - start_service[13] >= -999960.7530661469
 StartServiceRobots[13,14]: - 999999 v[13,14] - start_service[13]
   + start_service[14] >= -999979.4237314209
 StartServiceRobots[13,15]: - 999999 v[13,15] - start_service[13]
   + start_service[15] >= -999952.4465287814
 StartServiceRobots[13,16]: - 999999 v[13,16] - start_service[13]
   + start_service[16] >= -999984.5718729242
 StartServiceRobots[13,17]: - 999999 v[13,17] - start_service[13]
   + start_service[17] >= -999962.7885731502
 StartServiceRobots[13,18]: - 999999 v[13,18] - start_service[13]
   + start_service[18] >= -999972.9207620689
 StartServiceRobots[13,19]: - 999999 v[13,19] - start_service[13]
   + start_service[19] >= -999956.8858705129
 StartServiceRobots[13,20]: - 999999 v[13,20] - start_service[13]
   + start_service[20] >= -999995.2305121262
 StartServiceRobots[13,21]: - 999999 v[13,21] - start_service[13]
   + start_service[21] >= -999943.0282623119
 StartServiceRobots[13,22]: - 999999 v[13,22] - start_service[13]
   + start_service[22] >= -999977.0287494746
 StartServiceRobots[13,23]: - 999999 v[13,23] - start_service[13]
   + start_service[23] >= -999977.0287494746
 StartServiceRobots[14,0]: - 999999 v[14,0] + start_service[0]
   - start_service[14] >= -999994.052853037
 StartServiceRobots[14,1]: - 999999 v[14,1] + start_service[1]
   - start_service[14] >= -999994.052853037
 StartServiceRobots[14,2]: - 999999 v[14,2] + start_service[2]
   - start_service[14] >= -999973.8738751572
 StartServiceRobots[14,3]: - 999999 v[14,3] + start_service[3]
   - start_service[14] >= -999979.4237314209
 StartServiceRobots[14,5]: - 999999 v[14,5] + start_service[5]
   - start_service[14] >= -999933.296967469
 StartServiceRobots[14,6]: - 999999 v[14,6] + start_service[6]
   - start_service[14] >= -999972.10830705
 StartServiceRobots[14,7]: - 999999 v[14,7] + start_service[7]
   - start_service[14] >= -999956.3421461377
 StartServiceRobots[14,8]: - 999999 v[14,8] + start_service[8]
   - start_service[14] >= -999991.9597178316
 StartServiceRobots[14,9]: - 999999 v[14,9] + start_service[9]
   - start_service[14] >= -999964.9144336868
 StartServiceRobots[14,10]: - 999999 v[14,10] + start_service[10]
   - start_service[14] >= -999975.7042393816
 StartServiceRobots[14,11]: - 999999 v[14,11] + start_service[11]
   - start_service[14] >= -999926.9998154708
 StartServiceRobots[14,12]: - 999999 v[14,12] + start_service[12]
   - start_service[14] >= -999973.8738751572
 StartServiceRobots[14,13]: - 999999 v[14,13] + start_service[13]
   - start_service[14] >= -999979.4237314209
 StartServiceRobots[14,15]: - 999999 v[14,15] - start_service[14]
   + start_service[15] >= -999933.296967469
 StartServiceRobots[14,16]: - 999999 v[14,16] - start_service[14]
   + start_service[16] >= -999972.10830705
 StartServiceRobots[14,17]: - 999999 v[14,17] - start_service[14]
   + start_service[17] >= -999956.3421461377
 StartServiceRobots[14,18]: - 999999 v[14,18] - start_service[14]
   + start_service[18] >= -999991.9597178316
 StartServiceRobots[14,19]: - 999999 v[14,19] - start_service[14]
   + start_service[19] >= -999964.9144336868
 StartServiceRobots[14,20]: - 999999 v[14,20] - start_service[14]
   + start_service[20] >= -999975.7042393816
 StartServiceRobots[14,21]: - 999999 v[14,21] - start_service[14]
   + start_service[21] >= -999926.9998154708
 StartServiceRobots[14,22]: - 999999 v[14,22] - start_service[14]
   + start_service[22] >= -999994.052853037
 StartServiceRobots[14,23]: - 999999 v[14,23] - start_service[14]
   + start_service[23] >= -999994.052853037
 StartServiceRobots[15,0]: - 999999 v[15,0] + start_service[0]
   - start_service[15] >= -999930.4872109075
 StartServiceRobots[15,1]: - 999999 v[15,1] + start_service[1]
   - start_service[15] >= -999930.4872109075
 StartServiceRobots[15,2]: - 999999 v[15,2] + start_service[2]
   - start_service[15] >= -999922.0828731196
 StartServiceRobots[15,3]: - 999999 v[15,3] + start_service[3]
   - start_service[15] >= -999952.4465287814
 StartServiceRobots[15,4]: - 999999 v[15,4] + start_service[4]
   - start_service[15] >= -999933.296967469
 StartServiceRobots[15,6]: - 999999 v[15,6] + start_service[6]
   - start_service[15] >= -999957.1706496663
 StartServiceRobots[15,7]: - 999999 v[15,7] + start_service[7]
   - start_service[15] >= -999950.3991247974
 StartServiceRobots[15,8]: - 999999 v[15,8] + start_service[8]
   - start_service[15] >= -999927.4985292086
 StartServiceRobots[15,9]: - 999999 v[15,9] + start_service[9]
   - start_service[15] >= -999925.0867816372
 StartServiceRobots[15,10]: - 999999 v[15,10] + start_service[10]
   - start_service[15] >= -999955.8491601927
 StartServiceRobots[15,11]: - 999999 v[15,11] + start_service[11]
   - start_service[15] >= -999974.0460374815
 StartServiceRobots[15,12]: - 999999 v[15,12] + start_service[12]
   - start_service[15] >= -999922.0828731196
 StartServiceRobots[15,13]: - 999999 v[15,13] + start_service[13]
   - start_service[15] >= -999952.4465287814
 StartServiceRobots[15,14]: - 999999 v[15,14] + start_service[14]
   - start_service[15] >= -999933.296967469
 StartServiceRobots[15,16]: - 999999 v[15,16] - start_service[15]
   + start_service[16] >= -999957.1706496663
 StartServiceRobots[15,17]: - 999999 v[15,17] - start_service[15]
   + start_service[17] >= -999950.3991247974
 StartServiceRobots[15,18]: - 999999 v[15,18] - start_service[15]
   + start_service[18] >= -999927.4985292086
 StartServiceRobots[15,19]: - 999999 v[15,19] - start_service[15]
   + start_service[19] >= -999925.0867816372
 StartServiceRobots[15,20]: - 999999 v[15,20] - start_service[15]
   + start_service[20] >= -999955.8491601927
 StartServiceRobots[15,21]: - 999999 v[15,21] - start_service[15]
   + start_service[21] >= -999974.0460374815
 StartServiceRobots[15,22]: - 999999 v[15,22] - start_service[15]
   + start_service[22] >= -999930.4872109075
 StartServiceRobots[15,23]: - 999999 v[15,23] - start_service[15]
   + start_service[23] >= -999930.4872109075
 StartServiceRobots[16,0]: - 999999 v[16,0] + start_service[0]
   - start_service[16] >= -999967.9589406843
 StartServiceRobots[16,1]: - 999999 v[16,1] + start_service[1]
   - start_service[16] >= -999967.9589406843
 StartServiceRobots[16,2]: - 999999 v[16,2] + start_service[2]
   - start_service[16] >= -999963.6131301826
 StartServiceRobots[16,3]: - 999999 v[16,3] + start_service[3]
   - start_service[16] >= -999984.5718729242
 StartServiceRobots[16,4]: - 999999 v[16,4] + start_service[4]
   - start_service[16] >= -999972.10830705
 StartServiceRobots[16,5]: - 999999 v[16,5] + start_service[5]
   - start_service[16] >= -999957.1706496663
 StartServiceRobots[16,7]: - 999999 v[16,7] + start_service[7]
   - start_service[16] >= -999976.9595480452
 StartServiceRobots[16,8]: - 999999 v[16,8] + start_service[8]
   - start_service[16] >= -999967.7691443588
 StartServiceRobots[16,9]: - 999999 v[16,9] + start_service[9]
   - start_service[16] >= -999964.2077640658
 StartServiceRobots[16,10]: - 999999 v[16,10] + start_service[10]
   - start_service[16] >= -999984.2983734549
 StartServiceRobots[16,11]: - 999999 v[16,11] + start_service[11]
   - start_service[16] >= -999953.8825736255
 StartServiceRobots[16,12]: - 999999 v[16,12] + start_service[12]
   - start_service[16] >= -999963.6131301826
 StartServiceRobots[16,13]: - 999999 v[16,13] + start_service[13]
   - start_service[16] >= -999984.5718729242
 StartServiceRobots[16,14]: - 999999 v[16,14] + start_service[14]
   - start_service[16] >= -999972.10830705
 StartServiceRobots[16,15]: - 999999 v[16,15] + start_service[15]
   - start_service[16] >= -999957.1706496663
 StartServiceRobots[16,17]: - 999999 v[16,17] - start_service[16]
   + start_service[17] >= -999976.9595480452
 StartServiceRobots[16,18]: - 999999 v[16,18] - start_service[16]
   + start_service[18] >= -999967.7691443588
 StartServiceRobots[16,19]: - 999999 v[16,19] - start_service[16]
   + start_service[19] >= -999964.2077640658
 StartServiceRobots[16,20]: - 999999 v[16,20] - start_service[16]
   + start_service[20] >= -999984.2983734549
 StartServiceRobots[16,21]: - 999999 v[16,21] - start_service[16]
   + start_service[21] >= -999953.8825736255
 StartServiceRobots[16,22]: - 999999 v[16,22] - start_service[16]
   + start_service[22] >= -999967.9589406843
 StartServiceRobots[16,23]: - 999999 v[16,23] - start_service[16]
   + start_service[23] >= -999967.9589406843
 StartServiceRobots[17,0]: - 999999 v[17,0] + start_service[0]
   - start_service[17] >= -999951.4555340747
 StartServiceRobots[17,1]: - 999999 v[17,1] + start_service[1]
   - start_service[17] >= -999951.4555340747
 StartServiceRobots[17,2]: - 999999 v[17,2] + start_service[2]
   - start_service[17] >= -999962.1348413215
 StartServiceRobots[17,3]: - 999999 v[17,3] + start_service[3]
   - start_service[17] >= -999962.7885731502
 StartServiceRobots[17,4]: - 999999 v[17,4] + start_service[4]
   - start_service[17] >= -999956.3421461377
 StartServiceRobots[17,5]: - 999999 v[17,5] + start_service[5]
   - start_service[17] >= -999950.3991247974
 StartServiceRobots[17,6]: - 999999 v[17,6] + start_service[6]
   - start_service[17] >= -999976.9595480452
 StartServiceRobots[17,8]: - 999999 v[17,8] + start_service[8]
   - start_service[17] >= -999955.0366450849
 StartServiceRobots[17,9]: - 999999 v[17,9] + start_service[9]
   - start_service[17] >= -999970.2808806106
 StartServiceRobots[17,10]: - 999999 v[17,10] + start_service[10]
   - start_service[17] >= -999962.2590264571
 StartServiceRobots[17,11]: - 999999 v[17,11] + start_service[11]
   - start_service[17] >= -999959.5953207669
 StartServiceRobots[17,12]: - 999999 v[17,12] + start_service[12]
   - start_service[17] >= -999962.1348413215
 StartServiceRobots[17,13]: - 999999 v[17,13] + start_service[13]
   - start_service[17] >= -999962.7885731502
 StartServiceRobots[17,14]: - 999999 v[17,14] + start_service[14]
   - start_service[17] >= -999956.3421461377
 StartServiceRobots[17,15]: - 999999 v[17,15] + start_service[15]
   - start_service[17] >= -999950.3991247974
 StartServiceRobots[17,16]: - 999999 v[17,16] + start_service[16]
   - start_service[17] >= -999976.9595480452
 StartServiceRobots[17,18]: - 999999 v[17,18] - start_service[17]
   + start_service[18] >= -999955.0366450849
 StartServiceRobots[17,19]: - 999999 v[17,19] - start_service[17]
   + start_service[19] >= -999970.2808806106
 StartServiceRobots[17,20]: - 999999 v[17,20] - start_service[17]
   + start_service[20] >= -999962.2590264571
 StartServiceRobots[17,21]: - 999999 v[17,21] - start_service[17]
   + start_service[21] >= -999959.5953207669
 StartServiceRobots[17,22]: - 999999 v[17,22] - start_service[17]
   + start_service[22] >= -999951.4555340747
 StartServiceRobots[17,23]: - 999999 v[17,23] - start_service[17]
   + start_service[23] >= -999951.4555340747
 StartServiceRobots[18,0]: - 999999 v[18,0] + start_service[0]
   - start_service[18] >= -999990.1581436468
 StartServiceRobots[18,1]: - 999999 v[18,1] + start_service[1]
   - start_service[18] >= -999990.1581436468
 StartServiceRobots[18,2]: - 999999 v[18,2] + start_service[2]
   - start_service[18] >= -999979.0699702846
 StartServiceRobots[18,3]: - 999999 v[18,3] + start_service[3]
   - start_service[18] >= -999972.9207620689
 StartServiceRobots[18,4]: - 999999 v[18,4] + start_service[4]
   - start_service[18] >= -999991.9597178316
 StartServiceRobots[18,5]: - 999999 v[18,5] + start_service[5]
   - start_service[18] >= -999927.4985292086
 StartServiceRobots[18,6]: - 999999 v[18,6] + start_service[6]
   - start_service[18] >= -999967.7691443588
 StartServiceRobots[18,7]: - 999999 v[18,7] + start_service[7]
   - start_service[18] >= -999955.0366450849
 StartServiceRobots[18,9]: - 999999 v[18,9] + start_service[9]
   - start_service[18] >= -999968.5222110867
 StartServiceRobots[18,10]: - 999999 v[18,10] + start_service[10]
   - start_service[18] >= -999969.1569164672
 StartServiceRobots[18,11]: - 999999 v[18,11] + start_service[11]
   - start_service[18] >= -999922.8849691104
 StartServiceRobots[18,12]: - 999999 v[18,12] + start_service[12]
   - start_service[18] >= -999979.0699702846
 StartServiceRobots[18,13]: - 999999 v[18,13] + start_service[13]
   - start_service[18] >= -999972.9207620689
 StartServiceRobots[18,14]: - 999999 v[18,14] + start_service[14]
   - start_service[18] >= -999991.9597178316
 StartServiceRobots[18,15]: - 999999 v[18,15] + start_service[15]
   - start_service[18] >= -999927.4985292086
 StartServiceRobots[18,16]: - 999999 v[18,16] + start_service[16]
   - start_service[18] >= -999967.7691443588
 StartServiceRobots[18,17]: - 999999 v[18,17] + start_service[17]
   - start_service[18] >= -999955.0366450849
 StartServiceRobots[18,19]: - 999999 v[18,19] - start_service[18]
   + start_service[19] >= -999968.5222110867
 StartServiceRobots[18,20]: - 999999 v[18,20] - start_service[18]
   + start_service[20] >= -999969.1569164672
 StartServiceRobots[18,21]: - 999999 v[18,21] - start_service[18]
   + start_service[21] >= -999922.8849691104
 StartServiceRobots[18,22]: - 999999 v[18,22] - start_service[18]
   + start_service[22] >= -999990.1581436468
 StartServiceRobots[18,23]: - 999999 v[18,23] - start_service[18]
   + start_service[23] >= -999990.1581436468
 StartServiceRobots[19,0]: - 999999 v[19,0] + start_service[0]
   - start_service[19] >= -999960.6560937124
 StartServiceRobots[19,1]: - 999999 v[19,1] + start_service[1]
   - start_service[19] >= -999960.6560937124
 StartServiceRobots[19,2]: - 999999 v[19,2] + start_service[2]
   - start_service[19] >= -999986.4581145474
 StartServiceRobots[19,3]: - 999999 v[19,3] + start_service[3]
   - start_service[19] >= -999956.8858705129
 StartServiceRobots[19,4]: - 999999 v[19,4] + start_service[4]
   - start_service[19] >= -999964.9144336868
 StartServiceRobots[19,5]: - 999999 v[19,5] + start_service[5]
   - start_service[19] >= -999925.0867816372
 StartServiceRobots[19,6]: - 999999 v[19,6] + start_service[6]
   - start_service[19] >= -999964.2077640658
 StartServiceRobots[19,7]: - 999999 v[19,7] + start_service[7]
   - start_service[19] >= -999970.2808806106
 StartServiceRobots[19,8]: - 999999 v[19,8] + start_service[8]
   - start_service[19] >= -999968.5222110867
 StartServiceRobots[19,10]: - 999999 v[19,10] + start_service[10]
   - start_service[19] >= -999954.0422286508
 StartServiceRobots[19,11]: - 999999 v[19,11] + start_service[11]
   - start_service[19] >= -999930.9304394979
 StartServiceRobots[19,12]: - 999999 v[19,12] + start_service[12]
   - start_service[19] >= -999986.4581145474
 StartServiceRobots[19,13]: - 999999 v[19,13] + start_service[13]
   - start_service[19] >= -999956.8858705129
 StartServiceRobots[19,14]: - 999999 v[19,14] + start_service[14]
   - start_service[19] >= -999964.9144336868
 StartServiceRobots[19,15]: - 999999 v[19,15] + start_service[15]
   - start_service[19] >= -999925.0867816372
 StartServiceRobots[19,16]: - 999999 v[19,16] + start_service[16]
   - start_service[19] >= -999964.2077640658
 StartServiceRobots[19,17]: - 999999 v[19,17] + start_service[17]
   - start_service[19] >= -999970.2808806106
 StartServiceRobots[19,18]: - 999999 v[19,18] + start_service[18]
   - start_service[19] >= -999968.5222110867
 StartServiceRobots[19,20]: - 999999 v[19,20] - start_service[19]
   + start_service[20] >= -999954.0422286508
 StartServiceRobots[19,21]: - 999999 v[19,21] - start_service[19]
   + start_service[21] >= -999930.9304394979
 StartServiceRobots[19,22]: - 999999 v[19,22] - start_service[19]
   + start_service[22] >= -999960.6560937124
 StartServiceRobots[19,23]: - 999999 v[19,23] - start_service[19]
   + start_service[23] >= -999960.6560937124
 StartServiceRobots[20,0]: - 999999 v[20,0] + start_service[0]
   - start_service[20] >= -999973.4984481733
 StartServiceRobots[20,1]: - 999999 v[20,1] + start_service[1]
   - start_service[20] >= -999973.4984481733
 StartServiceRobots[20,2]: - 999999 v[20,2] + start_service[2]
   - start_service[20] >= -999957.3214814937
 StartServiceRobots[20,3]: - 999999 v[20,3] + start_service[3]
   - start_service[20] >= -999995.2305121262
 StartServiceRobots[20,4]: - 999999 v[20,4] + start_service[4]
   - start_service[20] >= -999975.7042393816
 StartServiceRobots[20,5]: - 999999 v[20,5] + start_service[5]
   - start_service[20] >= -999955.8491601927
 StartServiceRobots[20,6]: - 999999 v[20,6] + start_service[6]
   - start_service[20] >= -999984.2983734549
 StartServiceRobots[20,7]: - 999999 v[20,7] + start_service[7]
   - start_service[20] >= -999962.2590264571
 StartServiceRobots[20,8]: - 999999 v[20,8] + start_service[8]
   - start_service[20] >= -999969.1569164672
 StartServiceRobots[20,9]: - 999999 v[20,9] + start_service[9]
   - start_service[20] >= -999954.0422286508
 StartServiceRobots[20,11]: - 999999 v[20,11] + start_service[11]
   - start_service[20] >= -999945.3398959702
 StartServiceRobots[20,12]: - 999999 v[20,12] + start_service[12]
   - start_service[20] >= -999957.3214814937
 StartServiceRobots[20,13]: - 999999 v[20,13] + start_service[13]
   - start_service[20] >= -999995.2305121262
 StartServiceRobots[20,14]: - 999999 v[20,14] + start_service[14]
   - start_service[20] >= -999975.7042393816
 StartServiceRobots[20,15]: - 999999 v[20,15] + start_service[15]
   - start_service[20] >= -999955.8491601927
 StartServiceRobots[20,16]: - 999999 v[20,16] + start_service[16]
   - start_service[20] >= -999984.2983734549
 StartServiceRobots[20,17]: - 999999 v[20,17] + start_service[17]
   - start_service[20] >= -999962.2590264571
 StartServiceRobots[20,18]: - 999999 v[20,18] + start_service[18]
   - start_service[20] >= -999969.1569164672
 StartServiceRobots[20,19]: - 999999 v[20,19] + start_service[19]
   - start_service[20] >= -999954.0422286508
 StartServiceRobots[20,21]: - 999999 v[20,21] - start_service[20]
   + start_service[21] >= -999945.3398959702
 StartServiceRobots[20,22]: - 999999 v[20,22] - start_service[20]
   + start_service[22] >= -999973.4984481733
 StartServiceRobots[20,23]: - 999999 v[20,23] - start_service[20]
   + start_service[23] >= -999973.4984481733
 StartServiceRobots[21,0]: - 999999 v[21,0] + start_service[0]
   - start_service[21] >= -999922.9870655254
 StartServiceRobots[21,1]: - 999999 v[21,1] + start_service[1]
   - start_service[21] >= -999922.9870655254
 StartServiceRobots[21,2]: - 999999 v[21,2] + start_service[2]
   - start_service[21] >= -999924.0650949096
 StartServiceRobots[21,3]: - 999999 v[21,3] + start_service[3]
   - start_service[21] >= -999943.0282623119
 StartServiceRobots[21,4]: - 999999 v[21,4] + start_service[4]
   - start_service[21] >= -999926.9998154708
 StartServiceRobots[21,5]: - 999999 v[21,5] + start_service[5]
   - start_service[21] >= -999974.0460374815
 StartServiceRobots[21,6]: - 999999 v[21,6] + start_service[6]
   - start_service[21] >= -999953.8825736255
 StartServiceRobots[21,7]: - 999999 v[21,7] + start_service[7]
   - start_service[21] >= -999959.5953207669
 StartServiceRobots[21,8]: - 999999 v[21,8] + start_service[8]
   - start_service[21] >= -999922.8849691104
 StartServiceRobots[21,9]: - 999999 v[21,9] + start_service[9]
   - start_service[21] >= -999930.9304394979
 StartServiceRobots[21,10]: - 999999 v[21,10] + start_service[10]
   - start_service[21] >= -999945.3398959702
 StartServiceRobots[21,12]: - 999999 v[21,12] + start_service[12]
   - start_service[21] >= -999924.0650949096
 StartServiceRobots[21,13]: - 999999 v[21,13] + start_service[13]
   - start_service[21] >= -999943.0282623119
 StartServiceRobots[21,14]: - 999999 v[21,14] + start_service[14]
   - start_service[21] >= -999926.9998154708
 StartServiceRobots[21,15]: - 999999 v[21,15] + start_service[15]
   - start_service[21] >= -999974.0460374815
 StartServiceRobots[21,16]: - 999999 v[21,16] + start_service[16]
   - start_service[21] >= -999953.8825736255
 StartServiceRobots[21,17]: - 999999 v[21,17] + start_service[17]
   - start_service[21] >= -999959.5953207669
 StartServiceRobots[21,18]: - 999999 v[21,18] + start_service[18]
   - start_service[21] >= -999922.8849691104
 StartServiceRobots[21,19]: - 999999 v[21,19] + start_service[19]
   - start_service[21] >= -999930.9304394979
 StartServiceRobots[21,20]: - 999999 v[21,20] + start_service[20]
   - start_service[21] >= -999945.3398959702
 StartServiceRobots[21,22]: - 999999 v[21,22] - start_service[21]
   + start_service[22] >= -999922.9870655254
 StartServiceRobots[21,23]: - 999999 v[21,23] - start_service[21]
   + start_service[23] >= -999922.9870655254
 StartServiceRobots[22,0]: - 999999 v[22,0] + start_service[0]
   - start_service[22] >= -999999
 StartServiceRobots[22,1]: - 999999 v[22,1] + start_service[1]
   - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 999999 v[22,2] + start_service[2]
   - start_service[22] >= -999970.4355137392
 StartServiceRobots[22,3]: - 999999 v[22,3] + start_service[3]
   - start_service[22] >= -999977.0287494746
 StartServiceRobots[22,4]: - 999999 v[22,4] + start_service[4]
   - start_service[22] >= -999994.052853037
 StartServiceRobots[22,5]: - 999999 v[22,5] + start_service[5]
   - start_service[22] >= -999930.4872109075
 StartServiceRobots[22,6]: - 999999 v[22,6] + start_service[6]
   - start_service[22] >= -999967.9589406843
 StartServiceRobots[22,7]: - 999999 v[22,7] + start_service[7]
   - start_service[22] >= -999951.4555340747
 StartServiceRobots[22,8]: - 999999 v[22,8] + start_service[8]
   - start_service[22] >= -999990.1581436468
 StartServiceRobots[22,9]: - 999999 v[22,9] + start_service[9]
   - start_service[22] >= -999960.6560937124
 StartServiceRobots[22,10]: - 999999 v[22,10] + start_service[10]
   - start_service[22] >= -999973.4984481733
 StartServiceRobots[22,11]: - 999999 v[22,11] + start_service[11]
   - start_service[22] >= -999922.9870655254
 StartServiceRobots[22,12]: - 999999 v[22,12] + start_service[12]
   - start_service[22] >= -999970.4355137392
 StartServiceRobots[22,13]: - 999999 v[22,13] + start_service[13]
   - start_service[22] >= -999977.0287494746
 StartServiceRobots[22,14]: - 999999 v[22,14] + start_service[14]
   - start_service[22] >= -999994.052853037
 StartServiceRobots[22,15]: - 999999 v[22,15] + start_service[15]
   - start_service[22] >= -999930.4872109075
 StartServiceRobots[22,16]: - 999999 v[22,16] + start_service[16]
   - start_service[22] >= -999967.9589406843
 StartServiceRobots[22,17]: - 999999 v[22,17] + start_service[17]
   - start_service[22] >= -999951.4555340747
 StartServiceRobots[22,18]: - 999999 v[22,18] + start_service[18]
   - start_service[22] >= -999990.1581436468
 StartServiceRobots[22,19]: - 999999 v[22,19] + start_service[19]
   - start_service[22] >= -999960.6560937124
 StartServiceRobots[22,20]: - 999999 v[22,20] + start_service[20]
   - start_service[22] >= -999973.4984481733
 StartServiceRobots[22,21]: - 999999 v[22,21] + start_service[21]
   - start_service[22] >= -999922.9870655254
 StartServiceRobots[22,23]: - 999999 v[22,23] - start_service[22]
   + start_service[23] >= -999999
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
 SettingServiceTime[2]: - 320 y[2,1] - 160 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 240 y[3,1] - 120 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 230 y[4,1] - 115 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 400 y[5,1] - 200 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 250 y[6,1] - 125 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 370 y[7,1] - 185 y[7,2] + service_time[7] = 0
 SettingServiceTime[8]: - 490 y[8,1] - 245 y[8,2] + service_time[8] = 0
 SettingServiceTime[9]: - 340 y[9,1] - 170 y[9,2] + service_time[9] = 0
 SettingServiceTime[10]: - 330 y[10,1] - 165 y[10,2] + service_time[10]
   = 0
 SettingServiceTime[11]: - 380 y[11,1] - 190 y[11,2] + service_time[11]
   = 0
 AverageTimeRobots: - 4.080640894401712 w[0,2] - 3.138750075071305 w[0,3]
   - 0.7067352804231456 w[0,4] - 9.787541298935404 w[0,5]
   - 4.434437045102564 w[0,6] - 6.792066560760564 w[0,7]
   - 1.263122336166332 w[0,8] - 5.477700898216678 w[0,9]
   - 3.643078832386904 w[0,10] - 10.85899063922461 w[0,11]
   - 4.080640894401712 w[0,12] - 3.138750075071305 w[0,13]
   - 0.7067352804231456 w[0,14] - 9.787541298935404 w[0,15]
   - 4.434437045102564 w[0,16] - 6.792066560760564 w[0,17]
   - 1.263122336166332 w[0,18] - 5.477700898216678 w[0,19]
   - 3.643078832386904 w[0,20] - 10.85899063922461 w[0,21]
   - 4.080640894401712 w[1,2] - 3.138750075071305 w[1,3]
   - 0.7067352804231456 w[1,4] - 9.787541298935404 w[1,5]
   - 4.434437045102564 w[1,6] - 6.792066560760564 w[1,7]
   - 1.263122336166332 w[1,8] - 5.477700898216678 w[1,9]
   - 3.643078832386904 w[1,10] - 10.85899063922461 w[1,11]
   - 4.080640894401712 w[1,12] - 3.138750075071305 w[1,13]
   - 0.7067352804231456 w[1,14] - 9.787541298935404 w[1,15]
   - 4.434437045102564 w[1,16] - 6.792066560760564 w[1,17]
   - 1.263122336166332 w[1,18] - 5.477700898216678 w[1,19]
   - 3.643078832386904 w[1,20] - 10.85899063922461 w[1,21]
   - 4.080640894401712 w[2,1] - 5.463847693298037 w[2,3]
   - 3.589446406106944 w[2,4] - 10.98816098290929 w[2,5]
   - 5.055267116771124 w[2,6] - 5.266451239789668 w[2,7]
   - 2.847147102208258 w[2,8] - 1.79169792179125 w[2,9]
   - 5.954074072327724 w[2,10] - 10.70498644148782 w[2,11]
   - 5.463847693298037 w[2,13] - 3.589446406106944 w[2,14]
   - 10.98816098290929 w[2,15] - 5.055267116771124 w[2,16]
   - 5.266451239789668 w[2,17] - 2.847147102208258 w[2,18]
   - 1.79169792179125 w[2,19] - 5.954074072327724 w[2,20]
   - 10.70498644148782 w[2,21] - 4.080640894401712 w[2,22]
   - 4.080640894401712 w[2,23] - 3.138750075071305 w[3,1]
   - 5.463847693298037 w[3,2] - 2.796609797013992 w[3,4]
   - 6.650495888361457 w[3,5] - 2.061161010821114 w[3,6]
   - 5.173060978548105 w[3,7] - 3.725605418716002 w[3,8]
   - 6.01630421244393 w[3,9] - 0.5384982676824961 w[3,10]
   - 7.995962526866363 w[3,11] - 5.463847693298037 w[3,12]
   - 2.796609797013992 w[3,14] - 6.650495888361457 w[3,15]
   - 2.061161010821114 w[3,16] - 5.173060978548105 w[3,17]
   - 3.725605418716002 w[3,18] - 6.01630421244393 w[3,19]
   - 0.5384982676824961 w[3,20] - 7.995962526866363 w[3,21]
   - 3.138750075071305 w[3,22] - 3.138750075071305 w[3,23]
   - 0.7067352804231456 w[4,1] - 3.589446406106944 w[4,2]
   - 2.796609797013992 w[4,3] - 9.386147504423155 w[4,5]
   - 3.841670421426427 w[4,6] - 6.093979123173384 w[4,7]
   - 1.005754595496387 w[4,8] - 4.869366616175469 w[4,9]
   - 3.327965802631674 w[4,10] - 10.28574064703515 w[4,11]
   - 3.589446406106944 w[4,12] - 2.796609797013992 w[4,13]
   - 9.386147504423155 w[4,15] - 3.841670421426427 w[4,16]
   - 6.093979123173384 w[4,17] - 1.005754595496387 w[4,18]
   - 4.869366616175469 w[4,19] - 3.327965802631674 w[4,20]
   - 10.28574064703515 w[4,21] - 0.7067352804231456 w[4,22]
   - 0.7067352804231456 w[4,23] - 9.787541298935404 w[5,1]
   - 10.98816098290929 w[5,2] - 6.650495888361457 w[5,3]
   - 9.386147504423155 w[5,4] - 5.975621476234353 w[5,6]
   - 6.942982171800037 w[5,7] - 10.21449582734513 w[5,8]
   - 10.55903119468899 w[5,9] - 6.164405686761944 w[5,10]
   - 3.564851788347822 w[5,11] - 10.98816098290929 w[5,12]
   - 6.650495888361457 w[5,13] - 9.386147504423155 w[5,14]
   - 5.975621476234353 w[5,16] - 6.942982171800037 w[5,17]
   - 10.21449582734513 w[5,18] - 10.55903119468899 w[5,19]
   - 6.164405686761944 w[5,20] - 3.564851788347822 w[5,21]
   - 9.787541298935404 w[5,22] - 9.787541298935404 w[5,23]
   - 4.434437045102564 w[6,1] - 5.055267116771124 w[6,2]
   - 2.061161010821114 w[6,3] - 3.841670421426427 w[6,4]
   - 5.975621476234353 w[6,5] - 3.148635993544644 w[6,7]
   - 4.461550805880549 w[6,8] - 4.970319419175441 w[6,9]
   - 2.100232363586338 w[6,10] - 6.445346624915645 w[6,11]
   - 5.055267116771124 w[6,12] - 2.061161010821114 w[6,13]
   - 3.841670421426427 w[6,14] - 5.975621476234353 w[6,15]
   - 3.148635993544644 w[6,17] - 4.461550805880549 w[6,18]
   - 4.970319419175441 w[6,19] - 2.100232363586338 w[6,20]
   - 6.445346624915645 w[6,21] - 4.434437045102564 w[6,22]
   - 4.434437045102564 w[6,23] - 6.792066560760564 w[7,1]
   - 5.266451239789668 w[7,2] - 5.173060978548105 w[7,3]
   - 6.093979123173384 w[7,4] - 6.942982171800037 w[7,5]
   - 3.148635993544644 w[7,6] - 6.280479273598559 w[7,8]
   - 4.102731341347222 w[7,9] - 5.248710506127384 w[7,10]
   - 5.629239890434921 w[7,11] - 5.266451239789668 w[7,12]
   - 5.173060978548105 w[7,13] - 6.093979123173384 w[7,14]
   - 6.942982171800037 w[7,15] - 3.148635993544644 w[7,16]
   - 6.280479273598559 w[7,18] - 4.102731341347222 w[7,19]
   - 5.248710506127384 w[7,20] - 5.629239890434921 w[7,21]
   - 6.792066560760564 w[7,22] - 6.792066560760564 w[7,23]
   - 1.263122336166332 w[8,1] - 2.847147102208258 w[8,2]
   - 3.725605418716002 w[8,3] - 1.005754595496387 w[8,4]
   - 10.21449582734513 w[8,5] - 4.461550805880549 w[8,6]
   - 6.280479273598559 w[8,7] - 4.353969844748855 w[8,9]
   - 4.263297647532863 w[8,10] - 10.87357584138277 w[8,11]
   - 2.847147102208258 w[8,12] - 3.725605418716002 w[8,13]
   - 1.005754595496387 w[8,14] - 10.21449582734513 w[8,15]
   - 4.461550805880549 w[8,16] - 6.280479273598559 w[8,17]
   - 4.353969844748855 w[8,19] - 4.263297647532863 w[8,20]
   - 10.87357584138277 w[8,21] - 1.263122336166332 w[8,22]
   - 1.263122336166332 w[8,23] - 5.477700898216678 w[9,1]
   - 1.79169792179125 w[9,2] - 6.01630421244393 w[9,3]
   - 4.869366616175469 w[9,4] - 10.55903119468899 w[9,5]
   - 4.970319419175441 w[9,6] - 4.102731341347222 w[9,7]
   - 4.353969844748855 w[9,8] - 6.422538764180235 w[9,10]
   - 9.724222928864579 w[9,11] - 1.79169792179125 w[9,12]
   - 6.01630421244393 w[9,13] - 4.869366616175469 w[9,14]
   - 10.55903119468899 w[9,15] - 4.970319419175441 w[9,16]
   - 4.102731341347222 w[9,17] - 4.353969844748855 w[9,18]
   - 6.422538764180235 w[9,20] - 9.724222928864579 w[9,21]
   - 5.477700898216678 w[9,22] - 5.477700898216678 w[9,23]
   - 3.643078832386904 w[10,1] - 5.954074072327724 w[10,2]
   - 0.5384982676824961 w[10,3] - 3.327965802631674 w[10,4]
   - 6.164405686761944 w[10,5] - 2.100232363586338 w[10,6]
   - 5.248710506127384 w[10,7] - 4.263297647532863 w[10,8]
   - 6.422538764180235 w[10,9] - 7.665729147116929 w[10,11]
   - 5.954074072327724 w[10,12] - 0.5384982676824961 w[10,13]
   - 3.327965802631674 w[10,14] - 6.164405686761944 w[10,15]
   - 2.100232363586338 w[10,16] - 5.248710506127384 w[10,17]
   - 4.263297647532863 w[10,18] - 6.422538764180235 w[10,19]
   - 7.665729147116929 w[10,21] - 3.643078832386904 w[10,22]
   - 3.643078832386904 w[10,23] - 10.85899063922461 w[11,1]
   - 10.70498644148782 w[11,2] - 7.995962526866363 w[11,3]
   - 10.28574064703515 w[11,4] - 3.564851788347822 w[11,5]
   - 6.445346624915645 w[11,6] - 5.629239890434921 w[11,7]
   - 10.87357584138277 w[11,8] - 9.724222928864579 w[11,9]
   - 7.665729147116929 w[11,10] - 10.70498644148782 w[11,12]
   - 7.995962526866363 w[11,13] - 10.28574064703515 w[11,14]
   - 3.564851788347822 w[11,15] - 6.445346624915645 w[11,16]
   - 5.629239890434921 w[11,17] - 10.87357584138277 w[11,18]
   - 9.724222928864579 w[11,19] - 7.665729147116929 w[11,20]
   - 10.85899063922461 w[11,22] - 10.85899063922461 w[11,23]
   - 4.080640894401712 w[12,1] - 5.463847693298037 w[12,3]
   - 3.589446406106944 w[12,4] - 10.98816098290929 w[12,5]
   - 5.055267116771124 w[12,6] - 5.266451239789668 w[12,7]
   - 2.847147102208258 w[12,8] - 1.79169792179125 w[12,9]
   - 5.954074072327724 w[12,10] - 10.70498644148782 w[12,11]
   - 5.463847693298037 w[12,13] - 3.589446406106944 w[12,14]
   - 10.98816098290929 w[12,15] - 5.055267116771124 w[12,16]
   - 5.266451239789668 w[12,17] - 2.847147102208258 w[12,18]
   - 1.79169792179125 w[12,19] - 5.954074072327724 w[12,20]
   - 10.70498644148782 w[12,21] - 4.080640894401712 w[12,22]
   - 4.080640894401712 w[12,23] - 3.138750075071305 w[13,1]
   - 5.463847693298037 w[13,2] - 2.796609797013992 w[13,4]
   - 6.650495888361457 w[13,5] - 2.061161010821114 w[13,6]
   - 5.173060978548105 w[13,7] - 3.725605418716002 w[13,8]
   - 6.01630421244393 w[13,9] - 0.5384982676824961 w[13,10]
   - 7.995962526866363 w[13,11] - 5.463847693298037 w[13,12]
   - 2.796609797013992 w[13,14] - 6.650495888361457 w[13,15]
   - 2.061161010821114 w[13,16] - 5.173060978548105 w[13,17]
   - 3.725605418716002 w[13,18] - 6.01630421244393 w[13,19]
   - 0.5384982676824961 w[13,20] - 7.995962526866363 w[13,21]
   - 3.138750075071305 w[13,22] - 3.138750075071305 w[13,23]
   - 0.7067352804231456 w[14,1] - 3.589446406106944 w[14,2]
   - 2.796609797013992 w[14,3] - 9.386147504423155 w[14,5]
   - 3.841670421426427 w[14,6] - 6.093979123173384 w[14,7]
   - 1.005754595496387 w[14,8] - 4.869366616175469 w[14,9]
   - 3.327965802631674 w[14,10] - 10.28574064703515 w[14,11]
   - 3.589446406106944 w[14,12] - 2.796609797013992 w[14,13]
   - 9.386147504423155 w[14,15] - 3.841670421426427 w[14,16]
   - 6.093979123173384 w[14,17] - 1.005754595496387 w[14,18]
   - 4.869366616175469 w[14,19] - 3.327965802631674 w[14,20]
   - 10.28574064703515 w[14,21] - 0.7067352804231456 w[14,22]
   - 0.7067352804231456 w[14,23] - 9.787541298935404 w[15,1]
   - 10.98816098290929 w[15,2] - 6.650495888361457 w[15,3]
   - 9.386147504423155 w[15,4] - 5.975621476234353 w[15,6]
   - 6.942982171800037 w[15,7] - 10.21449582734513 w[15,8]
   - 10.55903119468899 w[15,9] - 6.164405686761944 w[15,10]
   - 3.564851788347822 w[15,11] - 10.98816098290929 w[15,12]
   - 6.650495888361457 w[15,13] - 9.386147504423155 w[15,14]
   - 5.975621476234353 w[15,16] - 6.942982171800037 w[15,17]
   - 10.21449582734513 w[15,18] - 10.55903119468899 w[15,19]
   - 6.164405686761944 w[15,20] - 3.564851788347822 w[15,21]
   - 9.787541298935404 w[15,22] - 9.787541298935404 w[15,23]
   - 4.434437045102564 w[16,1] - 5.055267116771124 w[16,2]
   - 2.061161010821114 w[16,3] - 3.841670421426427 w[16,4]
   - 5.975621476234353 w[16,5] - 3.148635993544644 w[16,7]
   - 4.461550805880549 w[16,8] - 4.970319419175441 w[16,9]
   - 2.100232363586338 w[16,10] - 6.445346624915645 w[16,11]
   - 5.055267116771124 w[16,12] - 2.061161010821114 w[16,13]
   - 3.841670421426427 w[16,14] - 5.975621476234353 w[16,15]
   - 3.148635993544644 w[16,17] - 4.461550805880549 w[16,18]
   - 4.970319419175441 w[16,19] - 2.100232363586338 w[16,20]
   - 6.445346624915645 w[16,21] - 4.434437045102564 w[16,22]
   - 4.434437045102564 w[16,23] - 6.792066560760564 w[17,1]
   - 5.266451239789668 w[17,2] - 5.173060978548105 w[17,3]
   - 6.093979123173384 w[17,4] - 6.942982171800037 w[17,5]
   - 3.148635993544644 w[17,6] - 6.280479273598559 w[17,8]
   - 4.102731341347222 w[17,9] - 5.248710506127384 w[17,10]
   - 5.629239890434921 w[17,11] - 5.266451239789668 w[17,12]
   - 5.173060978548105 w[17,13] - 6.093979123173384 w[17,14]
   - 6.942982171800037 w[17,15] - 3.148635993544644 w[17,16]
   - 6.280479273598559 w[17,18] - 4.102731341347222 w[17,19]
   - 5.248710506127384 w[17,20] - 5.629239890434921 w[17,21]
   - 6.792066560760564 w[17,22] - 6.792066560760564 w[17,23]
   - 1.263122336166332 w[18,1] - 2.847147102208258 w[18,2]
   - 3.725605418716002 w[18,3] - 1.005754595496387 w[18,4]
   - 10.21449582734513 w[18,5] - 4.461550805880549 w[18,6]
   - 6.280479273598559 w[18,7] - 4.353969844748855 w[18,9]
   - 4.263297647532863 w[18,10] - 10.87357584138277 w[18,11]
   - 2.847147102208258 w[18,12] - 3.725605418716002 w[18,13]
   - 1.005754595496387 w[18,14] - 10.21449582734513 w[18,15]
   - 4.461550805880549 w[18,16] - 6.280479273598559 w[18,17]
   - 4.353969844748855 w[18,19] - 4.263297647532863 w[18,20]
   - 10.87357584138277 w[18,21] - 1.263122336166332 w[18,22]
   - 1.263122336166332 w[18,23] - 5.477700898216678 w[19,1]
   - 1.79169792179125 w[19,2] - 6.01630421244393 w[19,3]
   - 4.869366616175469 w[19,4] - 10.55903119468899 w[19,5]
   - 4.970319419175441 w[19,6] - 4.102731341347222 w[19,7]
   - 4.353969844748855 w[19,8] - 6.422538764180235 w[19,10]
   - 9.724222928864579 w[19,11] - 1.79169792179125 w[19,12]
   - 6.01630421244393 w[19,13] - 4.869366616175469 w[19,14]
   - 10.55903119468899 w[19,15] - 4.970319419175441 w[19,16]
   - 4.102731341347222 w[19,17] - 4.353969844748855 w[19,18]
   - 6.422538764180235 w[19,20] - 9.724222928864579 w[19,21]
   - 5.477700898216678 w[19,22] - 5.477700898216678 w[19,23]
   - 3.643078832386904 w[20,1] - 5.954074072327724 w[20,2]
   - 0.5384982676824961 w[20,3] - 3.327965802631674 w[20,4]
   - 6.164405686761944 w[20,5] - 2.100232363586338 w[20,6]
   - 5.248710506127384 w[20,7] - 4.263297647532863 w[20,8]
   - 6.422538764180235 w[20,9] - 7.665729147116929 w[20,11]
   - 5.954074072327724 w[20,12] - 0.5384982676824961 w[20,13]
   - 3.327965802631674 w[20,14] - 6.164405686761944 w[20,15]
   - 2.100232363586338 w[20,16] - 5.248710506127384 w[20,17]
   - 4.263297647532863 w[20,18] - 6.422538764180235 w[20,19]
   - 7.665729147116929 w[20,21] - 3.643078832386904 w[20,22]
   - 3.643078832386904 w[20,23] - 10.85899063922461 w[21,1]
   - 10.70498644148782 w[21,2] - 7.995962526866363 w[21,3]
   - 10.28574064703515 w[21,4] - 3.564851788347822 w[21,5]
   - 6.445346624915645 w[21,6] - 5.629239890434921 w[21,7]
   - 10.87357584138277 w[21,8] - 9.724222928864579 w[21,9]
   - 7.665729147116929 w[21,10] - 10.70498644148782 w[21,12]
   - 7.995962526866363 w[21,13] - 10.28574064703515 w[21,14]
   - 3.564851788347822 w[21,15] - 6.445346624915645 w[21,16]
   - 5.629239890434921 w[21,17] - 10.87357584138277 w[21,18]
   - 9.724222928864579 w[21,19] - 7.665729147116929 w[21,20]
   - 10.85899063922461 w[21,22] - 10.85899063922461 w[21,23]
   - 4.080640894401712 w[22,2] - 3.138750075071305 w[22,3]
   - 0.7067352804231456 w[22,4] - 9.787541298935404 w[22,5]
   - 4.434437045102564 w[22,6] - 6.792066560760564 w[22,7]
   - 1.263122336166332 w[22,8] - 5.477700898216678 w[22,9]
   - 3.643078832386904 w[22,10] - 10.85899063922461 w[22,11]
   - 4.080640894401712 w[22,12] - 3.138750075071305 w[22,13]
   - 0.7067352804231456 w[22,14] - 9.787541298935404 w[22,15]
   - 4.434437045102564 w[22,16] - 6.792066560760564 w[22,17]
   - 1.263122336166332 w[22,18] - 5.477700898216678 w[22,19]
   - 3.643078832386904 w[22,20] - 10.85899063922461 w[22,21]
   - 45.71428571428571 y[2,1] - 91.42857142857142 y[2,2]
   - 34.28571428571428 y[3,1] - 68.57142857142857 y[3,2]
   - 32.85714285714285 y[4,1] - 65.71428571428571 y[4,2]
   - 57.14285714285714 y[5,1] - 114.2857142857143 y[5,2]
   - 35.71428571428572 y[6,1] - 71.42857142857143 y[6,2]
   - 52.85714285714285 y[7,1] - 105.7142857142857 y[7,2] - 70 y[8,1]
   - 140 y[8,2] - 48.57142857142857 y[9,1] - 97.14285714285714 y[9,2]
   - 47.14285714285714 y[10,1] - 94.28571428571428 y[10,2]
   - 54.28571428571428 y[11,1] - 108.5714285714286 y[11,2]
   + start_service[23] >= 0
 AverageTimeVehices: - 9.521495420270661 x[0,2] - 7.323750175166379 x[0,3]
   - 1.649048987654006 x[0,4] - 22.83759636418261 x[0,5]
   - 10.34701977190598 x[0,6] - 15.84815530844132 x[0,7]
   - 2.947285451054773 x[0,8] - 12.78130209583892 x[0,9]
   - 8.500517275569443 x[0,10] - 25.33764482485742 x[0,11]
   - 9.521495420270661 x[0,12] - 7.323750175166379 x[0,13]
   - 1.649048987654006 x[0,14] - 22.83759636418261 x[0,15]
   - 10.34701977190598 x[0,16] - 15.84815530844132 x[0,17]
   - 2.947285451054773 x[0,18] - 12.78130209583892 x[0,19]
   - 8.500517275569443 x[0,20] - 25.33764482485742 x[0,21]
   - 9.521495420270661 x[1,2] - 7.323750175166379 x[1,3]
   - 1.649048987654006 x[1,4] - 22.83759636418261 x[1,5]
   - 10.34701977190598 x[1,6] - 15.84815530844132 x[1,7]
   - 2.947285451054773 x[1,8] - 12.78130209583892 x[1,9]
   - 8.500517275569443 x[1,10] - 25.33764482485742 x[1,11]
   - 9.521495420270661 x[1,12] - 7.323750175166379 x[1,13]
   - 1.649048987654006 x[1,14] - 22.83759636418261 x[1,15]
   - 10.34701977190598 x[1,16] - 15.84815530844132 x[1,17]
   - 2.947285451054773 x[1,18] - 12.78130209583892 x[1,19]
   - 8.500517275569443 x[1,20] - 25.33764482485742 x[1,21]
   - 9.521495420270661 x[2,1] - 12.74897795102875 x[2,3]
   - 8.37537494758287 x[2,4] - 25.63904229345501 x[2,5]
   - 11.79562327246596 x[2,6] - 12.28838622617589 x[2,7]
   - 6.643343238485935 x[2,8] - 4.180628484179583 x[2,9]
   - 13.89283950209802 x[2,10] - 24.97830169680491 x[2,11]
   - 12.74897795102875 x[2,13] - 8.37537494758287 x[2,14]
   - 25.63904229345501 x[2,15] - 11.79562327246596 x[2,16]
   - 12.28838622617589 x[2,17] - 6.643343238485935 x[2,18]
   - 4.180628484179583 x[2,19] - 13.89283950209802 x[2,20]
   - 24.97830169680491 x[2,21] - 9.521495420270661 x[2,22]
   - 9.521495420270661 x[2,23] - 7.323750175166379 x[3,1]
   - 12.74897795102875 x[3,2] - 6.525422859699315 x[3,4]
   - 15.51782373951007 x[3,5] - 4.809375691915934 x[3,6]
   - 12.07047561661225 x[3,7] - 8.693079310337339 x[3,8]
   - 14.03804316236917 x[3,9] - 1.256495957925824 x[3,10]
   - 18.65724589602151 x[3,11] - 12.74897795102875 x[3,12]
   - 6.525422859699315 x[3,14] - 15.51782373951007 x[3,15]
   - 4.809375691915934 x[3,16] - 12.07047561661225 x[3,17]
   - 8.693079310337339 x[3,18] - 14.03804316236917 x[3,19]
   - 1.256495957925824 x[3,20] - 18.65724589602151 x[3,21]
   - 7.323750175166379 x[3,22] - 7.323750175166379 x[3,23]
   - 1.649048987654006 x[4,1] - 8.37537494758287 x[4,2]
   - 6.525422859699315 x[4,3] - 21.90101084365403 x[4,5]
   - 8.963897649994996 x[4,6] - 14.2192846207379 x[4,7]
   - 2.346760722824903 x[4,8] - 11.36185543774276 x[4,9]
   - 7.765253539473907 x[4,10] - 24.00006150974868 x[4,11]
   - 8.37537494758287 x[4,12] - 6.525422859699315 x[4,13]
   - 21.90101084365403 x[4,15] - 8.963897649994996 x[4,16]
   - 14.2192846207379 x[4,17] - 2.346760722824903 x[4,18]
   - 11.36185543774276 x[4,19] - 7.765253539473907 x[4,20]
   - 24.00006150974868 x[4,21] - 1.649048987654006 x[4,22]
   - 1.649048987654006 x[4,23] - 22.83759636418261 x[5,1]
   - 25.63904229345501 x[5,2] - 15.51782373951007 x[5,3]
   - 21.90101084365403 x[5,4] - 13.94311677788016 x[5,6]
   - 16.20029173420009 x[5,7] - 23.83382359713863 x[5,8]
   - 24.63773945427431 x[5,9] - 14.3836132691112 x[5,10]
   - 8.317987506144917 x[5,11] - 25.63904229345501 x[5,12]
   - 15.51782373951007 x[5,13] - 21.90101084365403 x[5,14]
   - 13.94311677788016 x[5,16] - 16.20029173420009 x[5,17]
   - 23.83382359713863 x[5,18] - 24.63773945427431 x[5,19]
   - 14.3836132691112 x[5,20] - 8.317987506144917 x[5,21]
   - 22.83759636418261 x[5,22] - 22.83759636418261 x[5,23]
   - 10.34701977190598 x[6,1] - 11.79562327246596 x[6,2]
   - 4.809375691915934 x[6,3] - 8.963897649994996 x[6,4]
   - 13.94311677788016 x[6,5] - 7.346817318270835 x[6,7]
   - 10.41028521372128 x[6,8] - 11.59741197807603 x[6,9]
   - 4.900542181701455 x[6,10] - 15.03914212480317 x[6,11]
   - 11.79562327246596 x[6,12] - 4.809375691915934 x[6,13]
   - 8.963897649994996 x[6,14] - 13.94311677788016 x[6,15]
   - 7.346817318270835 x[6,17] - 10.41028521372128 x[6,18]
   - 11.59741197807603 x[6,19] - 4.900542181701455 x[6,20]
   - 15.03914212480317 x[6,21] - 10.34701977190598 x[6,22]
   - 10.34701977190598 x[6,23] - 15.84815530844132 x[7,1]
   - 12.28838622617589 x[7,2] - 12.07047561661225 x[7,3]
   - 14.2192846207379 x[7,4] - 16.20029173420009 x[7,5]
   - 7.346817318270835 x[7,6] - 14.65445163839664 x[7,8]
   - 9.573039796476852 x[7,9] - 12.2469911809639 x[7,10]
   - 13.13489307768148 x[7,11] - 12.28838622617589 x[7,12]
   - 12.07047561661225 x[7,13] - 14.2192846207379 x[7,14]
   - 16.20029173420009 x[7,15] - 7.346817318270835 x[7,16]
   - 14.65445163839664 x[7,18] - 9.573039796476852 x[7,19]
   - 12.2469911809639 x[7,20] - 13.13489307768148 x[7,21]
   - 15.84815530844132 x[7,22] - 15.84815530844132 x[7,23]
   - 2.947285451054773 x[8,1] - 6.643343238485935 x[8,2]
   - 8.693079310337339 x[8,3] - 2.346760722824903 x[8,4]
   - 23.83382359713863 x[8,5] - 10.41028521372128 x[8,6]
   - 14.65445163839664 x[8,7] - 10.15926297108066 x[8,9]
   - 9.947694510910015 x[8,10] - 25.37167696322647 x[8,11]
   - 6.643343238485935 x[8,12] - 8.693079310337339 x[8,13]
   - 2.346760722824903 x[8,14] - 23.83382359713863 x[8,15]
   - 10.41028521372128 x[8,16] - 14.65445163839664 x[8,17]
   - 10.15926297108066 x[8,19] - 9.947694510910015 x[8,20]
   - 25.37167696322647 x[8,21] - 2.947285451054773 x[8,22]
   - 2.947285451054773 x[8,23] - 12.78130209583892 x[9,1]
   - 4.180628484179583 x[9,2] - 14.03804316236917 x[9,3]
   - 11.36185543774276 x[9,4] - 24.63773945427431 x[9,5]
   - 11.59741197807603 x[9,6] - 9.573039796476852 x[9,7]
   - 10.15926297108066 x[9,8] - 14.98592378308722 x[9,10]
   - 22.68985350068402 x[9,11] - 4.180628484179583 x[9,12]
   - 14.03804316236917 x[9,13] - 11.36185543774276 x[9,14]
   - 24.63773945427431 x[9,15] - 11.59741197807603 x[9,16]
   - 9.573039796476852 x[9,17] - 10.15926297108066 x[9,18]
   - 14.98592378308722 x[9,20] - 22.68985350068402 x[9,21]
   - 12.78130209583892 x[9,22] - 12.78130209583892 x[9,23]
   - 8.500517275569443 x[10,1] - 13.89283950209802 x[10,2]
   - 1.256495957925824 x[10,3] - 7.765253539473907 x[10,4]
   - 14.3836132691112 x[10,5] - 4.900542181701455 x[10,6]
   - 12.2469911809639 x[10,7] - 9.947694510910015 x[10,8]
   - 14.98592378308722 x[10,9] - 17.88670134327283 x[10,11]
   - 13.89283950209802 x[10,12] - 1.256495957925824 x[10,13]
   - 7.765253539473907 x[10,14] - 14.3836132691112 x[10,15]
   - 4.900542181701455 x[10,16] - 12.2469911809639 x[10,17]
   - 9.947694510910015 x[10,18] - 14.98592378308722 x[10,19]
   - 17.88670134327283 x[10,21] - 8.500517275569443 x[10,22]
   - 8.500517275569443 x[10,23] - 25.33764482485742 x[11,1]
   - 24.97830169680491 x[11,2] - 18.65724589602151 x[11,3]
   - 24.00006150974868 x[11,4] - 8.317987506144917 x[11,5]
   - 15.03914212480317 x[11,6] - 13.13489307768148 x[11,7]
   - 25.37167696322647 x[11,8] - 22.68985350068402 x[11,9]
   - 17.88670134327283 x[11,10] - 24.97830169680491 x[11,12]
   - 18.65724589602151 x[11,13] - 24.00006150974868 x[11,14]
   - 8.317987506144917 x[11,15] - 15.03914212480317 x[11,16]
   - 13.13489307768148 x[11,17] - 25.37167696322647 x[11,18]
   - 22.68985350068402 x[11,19] - 17.88670134327283 x[11,20]
   - 25.33764482485742 x[11,22] - 25.33764482485742 x[11,23]
   - 9.521495420270661 x[12,1] - 12.74897795102875 x[12,3]
   - 8.37537494758287 x[12,4] - 25.63904229345501 x[12,5]
   - 11.79562327246596 x[12,6] - 12.28838622617589 x[12,7]
   - 6.643343238485935 x[12,8] - 4.180628484179583 x[12,9]
   - 13.89283950209802 x[12,10] - 24.97830169680491 x[12,11]
   - 12.74897795102875 x[12,13] - 8.37537494758287 x[12,14]
   - 25.63904229345501 x[12,15] - 11.79562327246596 x[12,16]
   - 12.28838622617589 x[12,17] - 6.643343238485935 x[12,18]
   - 4.180628484179583 x[12,19] - 13.89283950209802 x[12,20]
   - 24.97830169680491 x[12,21] - 9.521495420270661 x[12,22]
   - 9.521495420270661 x[12,23] - 7.323750175166379 x[13,1]
   - 12.74897795102875 x[13,2] - 6.525422859699315 x[13,4]
   - 15.51782373951007 x[13,5] - 4.809375691915934 x[13,6]
   - 12.07047561661225 x[13,7] - 8.693079310337339 x[13,8]
   - 14.03804316236917 x[13,9] - 1.256495957925824 x[13,10]
   - 18.65724589602151 x[13,11] - 12.74897795102875 x[13,12]
   - 6.525422859699315 x[13,14] - 15.51782373951007 x[13,15]
   - 4.809375691915934 x[13,16] - 12.07047561661225 x[13,17]
   - 8.693079310337339 x[13,18] - 14.03804316236917 x[13,19]
   - 1.256495957925824 x[13,20] - 18.65724589602151 x[13,21]
   - 7.323750175166379 x[13,22] - 7.323750175166379 x[13,23]
   - 1.649048987654006 x[14,1] - 8.37537494758287 x[14,2]
   - 6.525422859699315 x[14,3] - 21.90101084365403 x[14,5]
   - 8.963897649994996 x[14,6] - 14.2192846207379 x[14,7]
   - 2.346760722824903 x[14,8] - 11.36185543774276 x[14,9]
   - 7.765253539473907 x[14,10] - 24.00006150974868 x[14,11]
   - 8.37537494758287 x[14,12] - 6.525422859699315 x[14,13]
   - 21.90101084365403 x[14,15] - 8.963897649994996 x[14,16]
   - 14.2192846207379 x[14,17] - 2.346760722824903 x[14,18]
   - 11.36185543774276 x[14,19] - 7.765253539473907 x[14,20]
   - 24.00006150974868 x[14,21] - 1.649048987654006 x[14,22]
   - 1.649048987654006 x[14,23] - 22.83759636418261 x[15,1]
   - 25.63904229345501 x[15,2] - 15.51782373951007 x[15,3]
   - 21.90101084365403 x[15,4] - 13.94311677788016 x[15,6]
   - 16.20029173420009 x[15,7] - 23.83382359713863 x[15,8]
   - 24.63773945427431 x[15,9] - 14.3836132691112 x[15,10]
   - 8.317987506144917 x[15,11] - 25.63904229345501 x[15,12]
   - 15.51782373951007 x[15,13] - 21.90101084365403 x[15,14]
   - 13.94311677788016 x[15,16] - 16.20029173420009 x[15,17]
   - 23.83382359713863 x[15,18] - 24.63773945427431 x[15,19]
   - 14.3836132691112 x[15,20] - 8.317987506144917 x[15,21]
   - 22.83759636418261 x[15,22] - 22.83759636418261 x[15,23]
   - 10.34701977190598 x[16,1] - 11.79562327246596 x[16,2]
   - 4.809375691915934 x[16,3] - 8.963897649994996 x[16,4]
   - 13.94311677788016 x[16,5] - 7.346817318270835 x[16,7]
   - 10.41028521372128 x[16,8] - 11.59741197807603 x[16,9]
   - 4.900542181701455 x[16,10] - 15.03914212480317 x[16,11]
   - 11.79562327246596 x[16,12] - 4.809375691915934 x[16,13]
   - 8.963897649994996 x[16,14] - 13.94311677788016 x[16,15]
   - 7.346817318270835 x[16,17] - 10.41028521372128 x[16,18]
   - 11.59741197807603 x[16,19] - 4.900542181701455 x[16,20]
   - 15.03914212480317 x[16,21] - 10.34701977190598 x[16,22]
   - 10.34701977190598 x[16,23] - 15.84815530844132 x[17,1]
   - 12.28838622617589 x[17,2] - 12.07047561661225 x[17,3]
   - 14.2192846207379 x[17,4] - 16.20029173420009 x[17,5]
   - 7.346817318270835 x[17,6] - 14.65445163839664 x[17,8]
   - 9.573039796476852 x[17,9] - 12.2469911809639 x[17,10]
   - 13.13489307768148 x[17,11] - 12.28838622617589 x[17,12]
   - 12.07047561661225 x[17,13] - 14.2192846207379 x[17,14]
   - 16.20029173420009 x[17,15] - 7.346817318270835 x[17,16]
   - 14.65445163839664 x[17,18] - 9.573039796476852 x[17,19]
   - 12.2469911809639 x[17,20] - 13.13489307768148 x[17,21]
   - 15.84815530844132 x[17,22] - 15.84815530844132 x[17,23]
   - 2.947285451054773 x[18,1] - 6.643343238485935 x[18,2]
   - 8.693079310337339 x[18,3] - 2.346760722824903 x[18,4]
   - 23.83382359713863 x[18,5] - 10.41028521372128 x[18,6]
   - 14.65445163839664 x[18,7] - 10.15926297108066 x[18,9]
   - 9.947694510910015 x[18,10] - 25.37167696322647 x[18,11]
   - 6.643343238485935 x[18,12] - 8.693079310337339 x[18,13]
   - 2.346760722824903 x[18,14] - 23.83382359713863 x[18,15]
   - 10.41028521372128 x[18,16] - 14.65445163839664 x[18,17]
   - 10.15926297108066 x[18,19] - 9.947694510910015 x[18,20]
   - 25.37167696322647 x[18,21] - 2.947285451054773 x[18,22]
   - 2.947285451054773 x[18,23] - 12.78130209583892 x[19,1]
   - 4.180628484179583 x[19,2] - 14.03804316236917 x[19,3]
   - 11.36185543774276 x[19,4] - 24.63773945427431 x[19,5]
   - 11.59741197807603 x[19,6] - 9.573039796476852 x[19,7]
   - 10.15926297108066 x[19,8] - 14.98592378308722 x[19,10]
   - 22.68985350068402 x[19,11] - 4.180628484179583 x[19,12]
   - 14.03804316236917 x[19,13] - 11.36185543774276 x[19,14]
   - 24.63773945427431 x[19,15] - 11.59741197807603 x[19,16]
   - 9.573039796476852 x[19,17] - 10.15926297108066 x[19,18]
   - 14.98592378308722 x[19,20] - 22.68985350068402 x[19,21]
   - 12.78130209583892 x[19,22] - 12.78130209583892 x[19,23]
   - 8.500517275569443 x[20,1] - 13.89283950209802 x[20,2]
   - 1.256495957925824 x[20,3] - 7.765253539473907 x[20,4]
   - 14.3836132691112 x[20,5] - 4.900542181701455 x[20,6]
   - 12.2469911809639 x[20,7] - 9.947694510910015 x[20,8]
   - 14.98592378308722 x[20,9] - 17.88670134327283 x[20,11]
   - 13.89283950209802 x[20,12] - 1.256495957925824 x[20,13]
   - 7.765253539473907 x[20,14] - 14.3836132691112 x[20,15]
   - 4.900542181701455 x[20,16] - 12.2469911809639 x[20,17]
   - 9.947694510910015 x[20,18] - 14.98592378308722 x[20,19]
   - 17.88670134327283 x[20,21] - 8.500517275569443 x[20,22]
   - 8.500517275569443 x[20,23] - 25.33764482485742 x[21,1]
   - 24.97830169680491 x[21,2] - 18.65724589602151 x[21,3]
   - 24.00006150974868 x[21,4] - 8.317987506144917 x[21,5]
   - 15.03914212480317 x[21,6] - 13.13489307768148 x[21,7]
   - 25.37167696322647 x[21,8] - 22.68985350068402 x[21,9]
   - 17.88670134327283 x[21,10] - 24.97830169680491 x[21,12]
   - 18.65724589602151 x[21,13] - 24.00006150974868 x[21,14]
   - 8.317987506144917 x[21,15] - 15.03914212480317 x[21,16]
   - 13.13489307768148 x[21,17] - 25.37167696322647 x[21,18]
   - 22.68985350068402 x[21,19] - 17.88670134327283 x[21,20]
   - 25.33764482485742 x[21,22] - 25.33764482485742 x[21,23]
   - 9.521495420270661 x[22,2] - 7.323750175166379 x[22,3]
   - 1.649048987654006 x[22,4] - 22.83759636418261 x[22,5]
   - 10.34701977190598 x[22,6] - 15.84815530844132 x[22,7]
   - 2.947285451054773 x[22,8] - 12.78130209583892 x[22,9]
   - 8.500517275569443 x[22,10] - 25.33764482485742 x[22,11]
   - 9.521495420270661 x[22,12] - 7.323750175166379 x[22,13]
   - 1.649048987654006 x[22,14] - 22.83759636418261 x[22,15]
   - 10.34701977190598 x[22,16] - 15.84815530844132 x[22,17]
   - 2.947285451054773 x[22,18] - 12.78130209583892 x[22,19]
   - 8.500517275569443 x[22,20] - 25.33764482485742 x[22,21]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[0]: - 57.12897252162397 x[0,2]
   - 43.94250105099827 x[0,3] - 9.894293925924039 x[0,4]
   - 137.0255781850957 x[0,5] - 62.0821186314359 x[0,6]
   - 95.08893185064791 x[0,7] - 17.68371270632864 x[0,8]
   - 76.6878125750335 x[0,9] - 51.00310365341667 x[0,10]
   - 152.0258689491445 x[0,11] - 57.12897252162397 x[0,12]
   - 43.94250105099827 x[0,13] - 9.894293925924039 x[0,14]
   - 137.0255781850957 x[0,15] - 62.0821186314359 x[0,16]
   - 95.08893185064791 x[0,17] - 17.68371270632864 x[0,18]
   - 76.6878125750335 x[0,19] - 51.00310365341667 x[0,20]
   - 152.0258689491445 x[0,21] - service_time[0] - start_service[0]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[1]: - 57.12897252162397 x[1,2]
   - 43.94250105099827 x[1,3] - 9.894293925924039 x[1,4]
   - 137.0255781850957 x[1,5] - 62.0821186314359 x[1,6]
   - 95.08893185064791 x[1,7] - 17.68371270632864 x[1,8]
   - 76.6878125750335 x[1,9] - 51.00310365341667 x[1,10]
   - 152.0258689491445 x[1,11] - 57.12897252162397 x[1,12]
   - 43.94250105099827 x[1,13] - 9.894293925924039 x[1,14]
   - 137.0255781850957 x[1,15] - 62.0821186314359 x[1,16]
   - 95.08893185064791 x[1,17] - 17.68371270632864 x[1,18]
   - 76.6878125750335 x[1,19] - 51.00310365341667 x[1,20]
   - 152.0258689491445 x[1,21] - service_time[1] - start_service[1]
   + start_service[23] >= 0
 LowerBoundRoutingVehicle[2]: - 28.56448626081199 x[2,1]
   - 28.56448626081199 x[2,2] - 60.2181843785854 x[2,3]
   - 30.07327180571063 x[2,4] - 145.4299159729129 x[2,5]
   - 66.42792913311582 x[2,6] - 84.40962460385163 x[2,7]
   - 28.77188606862213 x[2,8] - 50.8857917400555 x[2,9]
   - 67.1800703330024 x[2,10] - 150.947839564987 x[2,11]
   - 28.56448626081199 x[2,12] - 60.2181843785854 x[2,13]
   - 30.07327180571063 x[2,14] - 145.4299159729129 x[2,15]
   - 66.42792913311582 x[2,16] - 84.40962460385163 x[2,17]
   - 28.77188606862213 x[2,18] - 50.8857917400555 x[2,19]
   - 67.1800703330024 x[2,20] - 150.947839564987 x[2,21]
   - 28.56448626081199 x[2,22] - 28.56448626081199 x[2,23]
   - service_time[2] - start_service[2] + start_service[23] >= 0
 LowerBoundRoutingVehicle[3]: - 21.97125052549914 x[3,1]
   - 66.81142011389825 x[3,2] - 21.97125052549914 x[3,3]
   - 24.52341554205996 x[3,4] - 115.066260311078 x[3,5]
   - 45.46918639146575 x[3,6] - 83.7558927751607 x[3,7]
   - 34.92109428417633 x[3,8] - 80.45803577462426 x[3,9]
   - 29.2710397004858 x[3,10] - 131.9846721626368 x[3,11]
   - 66.81142011389825 x[3,12] - 21.97125052549914 x[3,13]
   - 24.52341554205996 x[3,14] - 115.066260311078 x[3,15]
   - 45.46918639146575 x[3,16] - 83.7558927751607 x[3,17]
   - 34.92109428417633 x[3,18] - 80.45803577462426 x[3,19]
   - 29.2710397004858 x[3,20] - 131.9846721626368 x[3,21]
   - 21.97125052549914 x[3,22] - 21.97125052549914 x[3,23]
   - service_time[3] - start_service[3] + start_service[23] >= 0
 LowerBoundRoutingVehicle[4]: - 4.94714696296202 x[4,1]
   - 53.6906111035606 x[4,2] - 41.54751910459709 x[4,3]
   - 4.94714696296202 x[4,4] - 134.2158216235099 x[4,5]
   - 57.93275226570294 x[4,6] - 90.20231978753765 x[4,7]
   - 15.88213852163903 x[4,8] - 72.42947260074504 x[4,9]
   - 48.79731244513005 x[4,10] - 148.0131190038183 x[4,11]
   - 53.6906111035606 x[4,12] - 41.54751910459709 x[4,13]
   - 4.94714696296202 x[4,14] - 134.2158216235099 x[4,15]
   - 57.93275226570294 x[4,16] - 90.20231978753765 x[4,17]
   - 15.88213852163903 x[4,18] - 72.42947260074504 x[4,19]
   - 48.79731244513005 x[4,20] - 148.0131190038183 x[4,21]
   - 4.94714696296202 x[4,22] - 4.94714696296202 x[4,23] - service_time[4]
   - start_service[4] + start_service[23] >= 0
 LowerBoundRoutingVehicle[5]: - 68.51278909254783 x[5,1]
   - 105.481613141177 x[5,2] - 68.52472174402934 x[5,3]
   - 70.65017949392411 x[5,4] - 68.51278909254783 x[5,5]
   - 72.87040964935842 x[5,6] - 96.14534112792421 x[5,7]
   - 80.34332714458021 x[5,8] - 112.2571246503397 x[5,9]
   - 68.65239163404195 x[5,10] - 100.966896993007 x[5,11]
   - 105.481613141177 x[5,12] - 68.52472174402934 x[5,13]
   - 70.65017949392411 x[5,14] - 68.51278909254783 x[5,15]
   - 72.87040964935842 x[5,16] - 96.14534112792421 x[5,17]
   - 80.34332714458021 x[5,18] - 112.2571246503397 x[5,19]
   - 68.65239163404195 x[5,20] - 100.966896993007 x[5,21]
   - 68.51278909254783 x[5,22] - 68.51278909254783 x[5,23]
   - service_time[5] - start_service[5] + start_service[23] >= 0
 LowerBoundRoutingVehicle[6]: - 31.04105931571795 x[6,1]
   - 63.95135607820986 x[6,2] - 36.39937760124694 x[6,3]
   - 31.83883991294701 x[6,4] - 110.3421394261883 x[6,5]
   - 31.04105931571795 x[6,6] - 69.58491788013646 x[6,7]
   - 40.07271199432816 x[6,8] - 73.13614222174485 x[6,9]
   - 40.2031783718127 x[6,10] - 121.1303608489818 x[6,11]
   - 63.95135607820986 x[6,12] - 36.39937760124694 x[6,13]
   - 31.83883991294701 x[6,14] - 110.3421394261883 x[6,15]
   - 31.04105931571795 x[6,16] - 69.58491788013646 x[6,17]
   - 40.07271199432816 x[6,18] - 73.13614222174485 x[6,19]
   - 40.2031783718127 x[6,20] - 121.1303608489818 x[6,21]
   - 31.04105931571795 x[6,22] - 31.04105931571795 x[6,23]
   - service_time[6] - start_service[6] + start_service[23] >= 0
 LowerBoundRoutingVehicle[7]: - 47.54446592532395 x[7,1]
   - 65.42964493933967 x[7,2] - 58.18267737533588 x[7,3]
   - 47.60500082517571 x[7,4] - 117.1136642951481 x[7,5]
   - 53.08151127053046 x[7,6] - 47.54446592532395 x[7,7]
   - 52.80521126835423 x[7,8] - 67.0630256769473 x[7,9]
   - 62.24252536960002 x[7,10] - 115.4176137076167 x[7,11]
   - 65.42964493933967 x[7,12] - 58.18267737533588 x[7,13]
   - 47.60500082517571 x[7,14] - 117.1136642951481 x[7,15]
   - 53.08151127053046 x[7,16] - 47.54446592532395 x[7,17]
   - 52.80521126835423 x[7,18] - 67.0630256769473 x[7,19]
   - 62.24252536960002 x[7,20] - 115.4176137076167 x[7,21]
   - 47.54446592532395 x[7,22] - 47.54446592532395 x[7,23]
   - service_time[7] - start_service[7] + start_service[23] >= 0
 LowerBoundRoutingVehicle[8]: - 8.841856353164321 x[8,1]
   - 48.4945159762698 x[8,2] - 48.05048845651115 x[8,3]
   - 11.98742913143673 x[8,4] - 140.0142598839637 x[8,5]
   - 62.27191495688179 x[8,6] - 91.50782084051386 x[8,7]
   - 8.841856353164321 x[8,8] - 68.82169520075874 x[8,9]
   - 55.34463535943838 x[8,10] - 152.1279653642517 x[8,11]
   - 48.4945159762698 x[8,12] - 48.05048845651115 x[8,13]
   - 11.98742913143673 x[8,14] - 140.0142598839637 x[8,15]
   - 62.27191495688179 x[8,16] - 91.50782084051386 x[8,17]
   - 8.841856353164321 x[8,18] - 68.82169520075874 x[8,19]
   - 55.34463535943838 x[8,20] - 152.1279653642517 x[8,21]
   - 8.841856353164321 x[8,22] - 8.841856353164321 x[8,23]
   - service_time[8] - start_service[8] + start_service[23] >= 0
 LowerBoundRoutingVehicle[9]: - 38.34390628751675 x[9,1]
   - 41.10637171335074 x[9,2] - 64.08538001260665 x[9,3]
   - 39.03271327619031 x[9,4] - 142.4260074553708 x[9,5]
   - 65.83329524994605 x[9,6] - 76.26358531475451 x[9,7]
   - 39.31964526640631 x[9,8] - 38.34390628751675 x[9,9]
   - 70.45932317596998 x[9,10] - 144.0824949766243 x[9,11]
   - 41.10637171335074 x[9,12] - 64.08538001260665 x[9,13]
   - 39.03271327619031 x[9,14] - 142.4260074553708 x[9,15]
   - 65.83329524994605 x[9,16] - 76.26358531475451 x[9,17]
   - 39.31964526640631 x[9,18] - 38.34390628751675 x[9,19]
   - 70.45932317596998 x[9,20] - 144.0824949766243 x[9,21]
   - 38.34390628751675 x[9,22] - 38.34390628751675 x[9,23]
   - service_time[9] - start_service[9] + start_service[23] >= 0
 LowerBoundRoutingVehicle[10]: - 25.50155182670833 x[10,1]
   - 70.24300476710606 x[10,2] - 25.74073839927661 x[10,3]
   - 28.24290758138374 x[10,4] - 111.6636288998814 x[10,5]
   - 45.74268586082232 x[10,6] - 84.28543946821564 x[10,7]
   - 38.68493988589437 x[10,8] - 83.30167763677841 x[10,9]
   - 25.50155182670833 x[10,10] - 129.6730385043908 x[10,11]
   - 70.24300476710606 x[10,12] - 25.74073839927661 x[10,13]
   - 28.24290758138374 x[10,14] - 111.6636288998814 x[10,15]
   - 45.74268586082232 x[10,16] - 84.28543946821564 x[10,17]
   - 38.68493988589437 x[10,18] - 83.30167763677841 x[10,19]
   - 25.50155182670833 x[10,20] - 129.6730385043908 x[10,21]
   - 25.50155182670833 x[10,22] - 25.50155182670833 x[10,23]
   - service_time[10] - start_service[10] + start_service[23] >= 0
 LowerBoundRoutingVehicle[11]: - 76.01293447457226 x[11,1]
   - 103.4993913512267 x[11,2] - 77.94298821356368 x[11,3]
   - 76.94733149220806 x[11,4] - 93.46675161098258 x[11,5]
   - 76.15848569012746 x[11,6] - 86.9491451583684 x[11,7]
   - 84.95688724284372 x[11,8] - 106.4134667895688 x[11,9]
   - 79.16165585652683 x[11,10] - 76.01293447457226 x[11,11]
   - 103.4993913512267 x[11,12] - 77.94298821356368 x[11,13]
   - 76.94733149220806 x[11,14] - 93.46675161098258 x[11,15]
   - 76.15848569012746 x[11,16] - 86.9491451583684 x[11,17]
   - 84.95688724284372 x[11,18] - 106.4134667895688 x[11,19]
   - 79.16165585652683 x[11,20] - 76.01293447457226 x[11,21]
   - 76.01293447457226 x[11,22] - 76.01293447457226 x[11,23]
   - service_time[11] - start_service[11] + start_service[23] >= 0
 LowerBoundRoutingVehicle[12]: - 28.56448626081199 x[12,1]
   - 28.56448626081199 x[12,2] - 60.2181843785854 x[12,3]
   - 30.07327180571063 x[12,4] - 145.4299159729129 x[12,5]
   - 66.42792913311582 x[12,6] - 84.40962460385163 x[12,7]
   - 28.77188606862213 x[12,8] - 50.8857917400555 x[12,9]
   - 67.1800703330024 x[12,10] - 150.947839564987 x[12,11]
   - 28.56448626081199 x[12,12] - 60.2181843785854 x[12,13]
   - 30.07327180571063 x[12,14] - 145.4299159729129 x[12,15]
   - 66.42792913311582 x[12,16] - 84.40962460385163 x[12,17]
   - 28.77188606862213 x[12,18] - 50.8857917400555 x[12,19]
   - 67.1800703330024 x[12,20] - 150.947839564987 x[12,21]
   - 28.56448626081199 x[12,22] - 28.56448626081199 x[12,23]
   - service_time[12] - start_service[12] + start_service[23] >= 0
 LowerBoundRoutingVehicle[13]: - 21.97125052549914 x[13,1]
   - 66.81142011389825 x[13,2] - 21.97125052549914 x[13,3]
   - 24.52341554205996 x[13,4] - 115.066260311078 x[13,5]
   - 45.46918639146575 x[13,6] - 83.7558927751607 x[13,7]
   - 34.92109428417633 x[13,8] - 80.45803577462426 x[13,9]
   - 29.2710397004858 x[13,10] - 131.9846721626368 x[13,11]
   - 66.81142011389825 x[13,12] - 21.97125052549914 x[13,13]
   - 24.52341554205996 x[13,14] - 115.066260311078 x[13,15]
   - 45.46918639146575 x[13,16] - 83.7558927751607 x[13,17]
   - 34.92109428417633 x[13,18] - 80.45803577462426 x[13,19]
   - 29.2710397004858 x[13,20] - 131.9846721626368 x[13,21]
   - 21.97125052549914 x[13,22] - 21.97125052549914 x[13,23]
   - service_time[13] - start_service[13] + start_service[23] >= 0
 LowerBoundRoutingVehicle[14]: - 4.94714696296202 x[14,1]
   - 53.6906111035606 x[14,2] - 41.54751910459709 x[14,3]
   - 4.94714696296202 x[14,4] - 134.2158216235099 x[14,5]
   - 57.93275226570294 x[14,6] - 90.20231978753765 x[14,7]
   - 15.88213852163903 x[14,8] - 72.42947260074504 x[14,9]
   - 48.79731244513005 x[14,10] - 148.0131190038183 x[14,11]
   - 53.6906111035606 x[14,12] - 41.54751910459709 x[14,13]
   - 4.94714696296202 x[14,14] - 134.2158216235099 x[14,15]
   - 57.93275226570294 x[14,16] - 90.20231978753765 x[14,17]
   - 15.88213852163903 x[14,18] - 72.42947260074504 x[14,19]
   - 48.79731244513005 x[14,20] - 148.0131190038183 x[14,21]
   - 4.94714696296202 x[14,22] - 4.94714696296202 x[14,23]
   - service_time[14] - start_service[14] + start_service[23] >= 0
 LowerBoundRoutingVehicle[15]: - 68.51278909254783 x[15,1]
   - 105.481613141177 x[15,2] - 68.52472174402934 x[15,3]
   - 70.65017949392411 x[15,4] - 68.51278909254783 x[15,5]
   - 72.87040964935842 x[15,6] - 96.14534112792421 x[15,7]
   - 80.34332714458021 x[15,8] - 112.2571246503397 x[15,9]
   - 68.65239163404195 x[15,10] - 100.966896993007 x[15,11]
   - 105.481613141177 x[15,12] - 68.52472174402934 x[15,13]
   - 70.65017949392411 x[15,14] - 68.51278909254783 x[15,15]
   - 72.87040964935842 x[15,16] - 96.14534112792421 x[15,17]
   - 80.34332714458021 x[15,18] - 112.2571246503397 x[15,19]
   - 68.65239163404195 x[15,20] - 100.966896993007 x[15,21]
   - 68.51278909254783 x[15,22] - 68.51278909254783 x[15,23]
   - service_time[15] - start_service[15] + start_service[23] >= 0
 LowerBoundRoutingVehicle[16]: - 31.04105931571795 x[16,1]
   - 63.95135607820986 x[16,2] - 36.39937760124694 x[16,3]
   - 31.83883991294701 x[16,4] - 110.3421394261883 x[16,5]
   - 31.04105931571795 x[16,6] - 69.58491788013646 x[16,7]
   - 40.07271199432816 x[16,8] - 73.13614222174485 x[16,9]
   - 40.2031783718127 x[16,10] - 121.1303608489818 x[16,11]
   - 63.95135607820986 x[16,12] - 36.39937760124694 x[16,13]
   - 31.83883991294701 x[16,14] - 110.3421394261883 x[16,15]
   - 31.04105931571795 x[16,16] - 69.58491788013646 x[16,17]
   - 40.07271199432816 x[16,18] - 73.13614222174485 x[16,19]
   - 40.2031783718127 x[16,20] - 121.1303608489818 x[16,21]
   - 31.04105931571795 x[16,22] - 31.04105931571795 x[16,23]
   - service_time[16] - start_service[16] + start_service[23] >= 0
 LowerBoundRoutingVehicle[17]: - 47.54446592532395 x[17,1]
   - 65.42964493933967 x[17,2] - 58.18267737533588 x[17,3]
   - 47.60500082517571 x[17,4] - 117.1136642951481 x[17,5]
   - 53.08151127053046 x[17,6] - 47.54446592532395 x[17,7]
   - 52.80521126835423 x[17,8] - 67.0630256769473 x[17,9]
   - 62.24252536960002 x[17,10] - 115.4176137076167 x[17,11]
   - 65.42964493933967 x[17,12] - 58.18267737533588 x[17,13]
   - 47.60500082517571 x[17,14] - 117.1136642951481 x[17,15]
   - 53.08151127053046 x[17,16] - 47.54446592532395 x[17,17]
   - 52.80521126835423 x[17,18] - 67.0630256769473 x[17,19]
   - 62.24252536960002 x[17,20] - 115.4176137076167 x[17,21]
   - 47.54446592532395 x[17,22] - 47.54446592532395 x[17,23]
   - service_time[17] - start_service[17] + start_service[23] >= 0
 LowerBoundRoutingVehicle[18]: - 8.841856353164321 x[18,1]
   - 48.4945159762698 x[18,2] - 48.05048845651115 x[18,3]
   - 11.98742913143673 x[18,4] - 140.0142598839637 x[18,5]
   - 62.27191495688179 x[18,6] - 91.50782084051386 x[18,7]
   - 8.841856353164321 x[18,8] - 68.82169520075874 x[18,9]
   - 55.34463535943838 x[18,10] - 152.1279653642517 x[18,11]
   - 48.4945159762698 x[18,12] - 48.05048845651115 x[18,13]
   - 11.98742913143673 x[18,14] - 140.0142598839637 x[18,15]
   - 62.27191495688179 x[18,16] - 91.50782084051386 x[18,17]
   - 8.841856353164321 x[18,18] - 68.82169520075874 x[18,19]
   - 55.34463535943838 x[18,20] - 152.1279653642517 x[18,21]
   - 8.841856353164321 x[18,22] - 8.841856353164321 x[18,23]
   - service_time[18] - start_service[18] + start_service[23] >= 0
 LowerBoundRoutingVehicle[19]: - 38.34390628751675 x[19,1]
   - 41.10637171335074 x[19,2] - 64.08538001260665 x[19,3]
   - 39.03271327619031 x[19,4] - 142.4260074553708 x[19,5]
   - 65.83329524994605 x[19,6] - 76.26358531475451 x[19,7]
   - 39.31964526640631 x[19,8] - 38.34390628751675 x[19,9]
   - 70.45932317596998 x[19,10] - 144.0824949766243 x[19,11]
   - 41.10637171335074 x[19,12] - 64.08538001260665 x[19,13]
   - 39.03271327619031 x[19,14] - 142.4260074553708 x[19,15]
   - 65.83329524994605 x[19,16] - 76.26358531475451 x[19,17]
   - 39.31964526640631 x[19,18] - 38.34390628751675 x[19,19]
   - 70.45932317596998 x[19,20] - 144.0824949766243 x[19,21]
   - 38.34390628751675 x[19,22] - 38.34390628751675 x[19,23]
   - service_time[19] - start_service[19] + start_service[23] >= 0
 LowerBoundRoutingVehicle[20]: - 25.50155182670833 x[20,1]
   - 70.24300476710606 x[20,2] - 25.74073839927661 x[20,3]
   - 28.24290758138374 x[20,4] - 111.6636288998814 x[20,5]
   - 45.74268586082232 x[20,6] - 84.28543946821564 x[20,7]
   - 38.68493988589437 x[20,8] - 83.30167763677841 x[20,9]
   - 25.50155182670833 x[20,10] - 129.6730385043908 x[20,11]
   - 70.24300476710606 x[20,12] - 25.74073839927661 x[20,13]
   - 28.24290758138374 x[20,14] - 111.6636288998814 x[20,15]
   - 45.74268586082232 x[20,16] - 84.28543946821564 x[20,17]
   - 38.68493988589437 x[20,18] - 83.30167763677841 x[20,19]
   - 25.50155182670833 x[20,20] - 129.6730385043908 x[20,21]
   - 25.50155182670833 x[20,22] - 25.50155182670833 x[20,23]
   - service_time[20] - start_service[20] + start_service[23] >= 0
 LowerBoundRoutingVehicle[21]: - 76.01293447457226 x[21,1]
   - 103.4993913512267 x[21,2] - 77.94298821356368 x[21,3]
   - 76.94733149220806 x[21,4] - 93.46675161098258 x[21,5]
   - 76.15848569012746 x[21,6] - 86.9491451583684 x[21,7]
   - 84.95688724284372 x[21,8] - 106.4134667895688 x[21,9]
   - 79.16165585652683 x[21,10] - 76.01293447457226 x[21,11]
   - 103.4993913512267 x[21,12] - 77.94298821356368 x[21,13]
   - 76.94733149220806 x[21,14] - 93.46675161098258 x[21,15]
   - 76.15848569012746 x[21,16] - 86.9491451583684 x[21,17]
   - 84.95688724284372 x[21,18] - 106.4134667895688 x[21,19]
   - 79.16165585652683 x[21,20] - 76.01293447457226 x[21,21]
   - 76.01293447457226 x[21,22] - 76.01293447457226 x[21,23]
   - service_time[21] - start_service[21] + start_service[23] >= 0
 LowerBoundRoutingVehicle[22]: - 57.12897252162397 x[22,2]
   - 43.94250105099827 x[22,3] - 9.894293925924039 x[22,4]
   - 137.0255781850957 x[22,5] - 62.0821186314359 x[22,6]
   - 95.08893185064791 x[22,7] - 17.68371270632864 x[22,8]
   - 76.6878125750335 x[22,9] - 51.00310365341667 x[22,10]
   - 152.0258689491445 x[22,11] - 57.12897252162397 x[22,12]
   - 43.94250105099827 x[22,13] - 9.894293925924039 x[22,14]
   - 137.0255781850957 x[22,15] - 62.0821186314359 x[22,16]
   - 95.08893185064791 x[22,17] - 17.68371270632864 x[22,18]
   - 76.6878125750335 x[22,19] - 51.00310365341667 x[22,20]
   - 152.0258689491445 x[22,21] - service_time[22] - start_service[22]
   + start_service[23] >= 0
 LowerBoundArrivalVehicle[1]: - 57.12897252162397 x[2,1]
   - 43.94250105099827 x[3,1] - 9.894293925924039 x[4,1]
   - 137.0255781850957 x[5,1] - 62.0821186314359 x[6,1]
   - 95.08893185064791 x[7,1] - 17.68371270632864 x[8,1]
   - 76.6878125750335 x[9,1] - 51.00310365341667 x[10,1]
   - 152.0258689491445 x[11,1] - 57.12897252162397 x[12,1]
   - 43.94250105099827 x[13,1] - 9.894293925924039 x[14,1]
   - 137.0255781850957 x[15,1] - 62.0821186314359 x[16,1]
   - 95.08893185064791 x[17,1] - 17.68371270632864 x[18,1]
   - 76.6878125750335 x[19,1] - 51.00310365341667 x[20,1]
   - 152.0258689491445 x[21,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 28.56448626081199 x[0,2]
   - 28.56448626081199 x[1,2] - 28.56448626081199 x[2,2]
   - 60.2181843785854 x[3,2] - 30.07327180571063 x[4,2]
   - 145.4299159729129 x[5,2] - 66.42792913311582 x[6,2]
   - 84.40962460385163 x[7,2] - 28.77188606862213 x[8,2]
   - 50.8857917400555 x[9,2] - 67.1800703330024 x[10,2]
   - 150.947839564987 x[11,2] - 28.56448626081199 x[12,2]
   - 60.2181843785854 x[13,2] - 30.07327180571063 x[14,2]
   - 145.4299159729129 x[15,2] - 66.42792913311582 x[16,2]
   - 84.40962460385163 x[17,2] - 28.77188606862213 x[18,2]
   - 50.8857917400555 x[19,2] - 67.1800703330024 x[20,2]
   - 150.947839564987 x[21,2] - 28.56448626081199 x[22,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 21.97125052549914 x[0,3]
   - 21.97125052549914 x[1,3] - 66.81142011389825 x[2,3]
   - 21.97125052549914 x[3,3] - 24.52341554205996 x[4,3]
   - 115.066260311078 x[5,3] - 45.46918639146575 x[6,3]
   - 83.7558927751607 x[7,3] - 34.92109428417633 x[8,3]
   - 80.45803577462426 x[9,3] - 29.2710397004858 x[10,3]
   - 131.9846721626368 x[11,3] - 66.81142011389825 x[12,3]
   - 21.97125052549914 x[13,3] - 24.52341554205996 x[14,3]
   - 115.066260311078 x[15,3] - 45.46918639146575 x[16,3]
   - 83.7558927751607 x[17,3] - 34.92109428417633 x[18,3]
   - 80.45803577462426 x[19,3] - 29.2710397004858 x[20,3]
   - 131.9846721626368 x[21,3] - 21.97125052549914 x[22,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 4.94714696296202 x[0,4]
   - 4.94714696296202 x[1,4] - 53.6906111035606 x[2,4]
   - 41.54751910459709 x[3,4] - 4.94714696296202 x[4,4]
   - 134.2158216235099 x[5,4] - 57.93275226570294 x[6,4]
   - 90.20231978753765 x[7,4] - 15.88213852163903 x[8,4]
   - 72.42947260074504 x[9,4] - 48.79731244513005 x[10,4]
   - 148.0131190038183 x[11,4] - 53.6906111035606 x[12,4]
   - 41.54751910459709 x[13,4] - 4.94714696296202 x[14,4]
   - 134.2158216235099 x[15,4] - 57.93275226570294 x[16,4]
   - 90.20231978753765 x[17,4] - 15.88213852163903 x[18,4]
   - 72.42947260074504 x[19,4] - 48.79731244513005 x[20,4]
   - 148.0131190038183 x[21,4] - 4.94714696296202 x[22,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 68.51278909254783 x[0,5]
   - 68.51278909254783 x[1,5] - 105.481613141177 x[2,5]
   - 68.52472174402934 x[3,5] - 70.65017949392411 x[4,5]
   - 68.51278909254783 x[5,5] - 72.87040964935842 x[6,5]
   - 96.14534112792421 x[7,5] - 80.34332714458021 x[8,5]
   - 112.2571246503397 x[9,5] - 68.65239163404195 x[10,5]
   - 100.966896993007 x[11,5] - 105.481613141177 x[12,5]
   - 68.52472174402934 x[13,5] - 70.65017949392411 x[14,5]
   - 68.51278909254783 x[15,5] - 72.87040964935842 x[16,5]
   - 96.14534112792421 x[17,5] - 80.34332714458021 x[18,5]
   - 112.2571246503397 x[19,5] - 68.65239163404195 x[20,5]
   - 100.966896993007 x[21,5] - 68.51278909254783 x[22,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 31.04105931571795 x[0,6]
   - 31.04105931571795 x[1,6] - 63.95135607820986 x[2,6]
   - 36.39937760124694 x[3,6] - 31.83883991294701 x[4,6]
   - 110.3421394261883 x[5,6] - 31.04105931571795 x[6,6]
   - 69.58491788013646 x[7,6] - 40.07271199432816 x[8,6]
   - 73.13614222174485 x[9,6] - 40.2031783718127 x[10,6]
   - 121.1303608489818 x[11,6] - 63.95135607820986 x[12,6]
   - 36.39937760124694 x[13,6] - 31.83883991294701 x[14,6]
   - 110.3421394261883 x[15,6] - 31.04105931571795 x[16,6]
   - 69.58491788013646 x[17,6] - 40.07271199432816 x[18,6]
   - 73.13614222174485 x[19,6] - 40.2031783718127 x[20,6]
   - 121.1303608489818 x[21,6] - 31.04105931571795 x[22,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 47.54446592532395 x[0,7]
   - 47.54446592532395 x[1,7] - 65.42964493933967 x[2,7]
   - 58.18267737533588 x[3,7] - 47.60500082517571 x[4,7]
   - 117.1136642951481 x[5,7] - 53.08151127053046 x[6,7]
   - 47.54446592532395 x[7,7] - 52.80521126835423 x[8,7]
   - 67.0630256769473 x[9,7] - 62.24252536960002 x[10,7]
   - 115.4176137076167 x[11,7] - 65.42964493933967 x[12,7]
   - 58.18267737533588 x[13,7] - 47.60500082517571 x[14,7]
   - 117.1136642951481 x[15,7] - 53.08151127053046 x[16,7]
   - 47.54446592532395 x[17,7] - 52.80521126835423 x[18,7]
   - 67.0630256769473 x[19,7] - 62.24252536960002 x[20,7]
   - 115.4176137076167 x[21,7] - 47.54446592532395 x[22,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 8.841856353164321 x[0,8]
   - 8.841856353164321 x[1,8] - 48.4945159762698 x[2,8]
   - 48.05048845651115 x[3,8] - 11.98742913143673 x[4,8]
   - 140.0142598839637 x[5,8] - 62.27191495688179 x[6,8]
   - 91.50782084051386 x[7,8] - 8.841856353164321 x[8,8]
   - 68.82169520075874 x[9,8] - 55.34463535943838 x[10,8]
   - 152.1279653642517 x[11,8] - 48.4945159762698 x[12,8]
   - 48.05048845651115 x[13,8] - 11.98742913143673 x[14,8]
   - 140.0142598839637 x[15,8] - 62.27191495688179 x[16,8]
   - 91.50782084051386 x[17,8] - 8.841856353164321 x[18,8]
   - 68.82169520075874 x[19,8] - 55.34463535943838 x[20,8]
   - 152.1279653642517 x[21,8] - 8.841856353164321 x[22,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 38.34390628751675 x[0,9]
   - 38.34390628751675 x[1,9] - 41.10637171335074 x[2,9]
   - 64.08538001260665 x[3,9] - 39.03271327619031 x[4,9]
   - 142.4260074553708 x[5,9] - 65.83329524994605 x[6,9]
   - 76.26358531475451 x[7,9] - 39.31964526640631 x[8,9]
   - 38.34390628751675 x[9,9] - 70.45932317596998 x[10,9]
   - 144.0824949766243 x[11,9] - 41.10637171335074 x[12,9]
   - 64.08538001260665 x[13,9] - 39.03271327619031 x[14,9]
   - 142.4260074553708 x[15,9] - 65.83329524994605 x[16,9]
   - 76.26358531475451 x[17,9] - 39.31964526640631 x[18,9]
   - 38.34390628751675 x[19,9] - 70.45932317596998 x[20,9]
   - 144.0824949766243 x[21,9] - 38.34390628751675 x[22,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 25.50155182670833 x[0,10]
   - 25.50155182670833 x[1,10] - 70.24300476710606 x[2,10]
   - 25.74073839927661 x[3,10] - 28.24290758138374 x[4,10]
   - 111.6636288998814 x[5,10] - 45.74268586082232 x[6,10]
   - 84.28543946821564 x[7,10] - 38.68493988589437 x[8,10]
   - 83.30167763677841 x[9,10] - 25.50155182670833 x[10,10]
   - 129.6730385043908 x[11,10] - 70.24300476710606 x[12,10]
   - 25.74073839927661 x[13,10] - 28.24290758138374 x[14,10]
   - 111.6636288998814 x[15,10] - 45.74268586082232 x[16,10]
   - 84.28543946821564 x[17,10] - 38.68493988589437 x[18,10]
   - 83.30167763677841 x[19,10] - 25.50155182670833 x[20,10]
   - 129.6730385043908 x[21,10] - 25.50155182670833 x[22,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 76.01293447457226 x[0,11]
   - 76.01293447457226 x[1,11] - 103.4993913512267 x[2,11]
   - 77.94298821356368 x[3,11] - 76.94733149220806 x[4,11]
   - 93.46675161098258 x[5,11] - 76.15848569012746 x[6,11]
   - 86.9491451583684 x[7,11] - 84.95688724284372 x[8,11]
   - 106.4134667895688 x[9,11] - 79.16165585652683 x[10,11]
   - 76.01293447457226 x[11,11] - 103.4993913512267 x[12,11]
   - 77.94298821356368 x[13,11] - 76.94733149220806 x[14,11]
   - 93.46675161098258 x[15,11] - 76.15848569012746 x[16,11]
   - 86.9491451583684 x[17,11] - 84.95688724284372 x[18,11]
   - 106.4134667895688 x[19,11] - 79.16165585652683 x[20,11]
   - 76.01293447457226 x[21,11] - 76.01293447457226 x[22,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 28.56448626081199 x[0,12]
   - 28.56448626081199 x[1,12] - 28.56448626081199 x[2,12]
   - 60.2181843785854 x[3,12] - 30.07327180571063 x[4,12]
   - 145.4299159729129 x[5,12] - 66.42792913311582 x[6,12]
   - 84.40962460385163 x[7,12] - 28.77188606862213 x[8,12]
   - 50.8857917400555 x[9,12] - 67.1800703330024 x[10,12]
   - 150.947839564987 x[11,12] - 28.56448626081199 x[12,12]
   - 60.2181843785854 x[13,12] - 30.07327180571063 x[14,12]
   - 145.4299159729129 x[15,12] - 66.42792913311582 x[16,12]
   - 84.40962460385163 x[17,12] - 28.77188606862213 x[18,12]
   - 50.8857917400555 x[19,12] - 67.1800703330024 x[20,12]
   - 150.947839564987 x[21,12] - 28.56448626081199 x[22,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 21.97125052549914 x[0,13]
   - 21.97125052549914 x[1,13] - 66.81142011389825 x[2,13]
   - 21.97125052549914 x[3,13] - 24.52341554205996 x[4,13]
   - 115.066260311078 x[5,13] - 45.46918639146575 x[6,13]
   - 83.7558927751607 x[7,13] - 34.92109428417633 x[8,13]
   - 80.45803577462426 x[9,13] - 29.2710397004858 x[10,13]
   - 131.9846721626368 x[11,13] - 66.81142011389825 x[12,13]
   - 21.97125052549914 x[13,13] - 24.52341554205996 x[14,13]
   - 115.066260311078 x[15,13] - 45.46918639146575 x[16,13]
   - 83.7558927751607 x[17,13] - 34.92109428417633 x[18,13]
   - 80.45803577462426 x[19,13] - 29.2710397004858 x[20,13]
   - 131.9846721626368 x[21,13] - 21.97125052549914 x[22,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 4.94714696296202 x[0,14]
   - 4.94714696296202 x[1,14] - 53.6906111035606 x[2,14]
   - 41.54751910459709 x[3,14] - 4.94714696296202 x[4,14]
   - 134.2158216235099 x[5,14] - 57.93275226570294 x[6,14]
   - 90.20231978753765 x[7,14] - 15.88213852163903 x[8,14]
   - 72.42947260074504 x[9,14] - 48.79731244513005 x[10,14]
   - 148.0131190038183 x[11,14] - 53.6906111035606 x[12,14]
   - 41.54751910459709 x[13,14] - 4.94714696296202 x[14,14]
   - 134.2158216235099 x[15,14] - 57.93275226570294 x[16,14]
   - 90.20231978753765 x[17,14] - 15.88213852163903 x[18,14]
   - 72.42947260074504 x[19,14] - 48.79731244513005 x[20,14]
   - 148.0131190038183 x[21,14] - 4.94714696296202 x[22,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 68.51278909254783 x[0,15]
   - 68.51278909254783 x[1,15] - 105.481613141177 x[2,15]
   - 68.52472174402934 x[3,15] - 70.65017949392411 x[4,15]
   - 68.51278909254783 x[5,15] - 72.87040964935842 x[6,15]
   - 96.14534112792421 x[7,15] - 80.34332714458021 x[8,15]
   - 112.2571246503397 x[9,15] - 68.65239163404195 x[10,15]
   - 100.966896993007 x[11,15] - 105.481613141177 x[12,15]
   - 68.52472174402934 x[13,15] - 70.65017949392411 x[14,15]
   - 68.51278909254783 x[15,15] - 72.87040964935842 x[16,15]
   - 96.14534112792421 x[17,15] - 80.34332714458021 x[18,15]
   - 112.2571246503397 x[19,15] - 68.65239163404195 x[20,15]
   - 100.966896993007 x[21,15] - 68.51278909254783 x[22,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 31.04105931571795 x[0,16]
   - 31.04105931571795 x[1,16] - 63.95135607820986 x[2,16]
   - 36.39937760124694 x[3,16] - 31.83883991294701 x[4,16]
   - 110.3421394261883 x[5,16] - 31.04105931571795 x[6,16]
   - 69.58491788013646 x[7,16] - 40.07271199432816 x[8,16]
   - 73.13614222174485 x[9,16] - 40.2031783718127 x[10,16]
   - 121.1303608489818 x[11,16] - 63.95135607820986 x[12,16]
   - 36.39937760124694 x[13,16] - 31.83883991294701 x[14,16]
   - 110.3421394261883 x[15,16] - 31.04105931571795 x[16,16]
   - 69.58491788013646 x[17,16] - 40.07271199432816 x[18,16]
   - 73.13614222174485 x[19,16] - 40.2031783718127 x[20,16]
   - 121.1303608489818 x[21,16] - 31.04105931571795 x[22,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 47.54446592532395 x[0,17]
   - 47.54446592532395 x[1,17] - 65.42964493933967 x[2,17]
   - 58.18267737533588 x[3,17] - 47.60500082517571 x[4,17]
   - 117.1136642951481 x[5,17] - 53.08151127053046 x[6,17]
   - 47.54446592532395 x[7,17] - 52.80521126835423 x[8,17]
   - 67.0630256769473 x[9,17] - 62.24252536960002 x[10,17]
   - 115.4176137076167 x[11,17] - 65.42964493933967 x[12,17]
   - 58.18267737533588 x[13,17] - 47.60500082517571 x[14,17]
   - 117.1136642951481 x[15,17] - 53.08151127053046 x[16,17]
   - 47.54446592532395 x[17,17] - 52.80521126835423 x[18,17]
   - 67.0630256769473 x[19,17] - 62.24252536960002 x[20,17]
   - 115.4176137076167 x[21,17] - 47.54446592532395 x[22,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 8.841856353164321 x[0,18]
   - 8.841856353164321 x[1,18] - 48.4945159762698 x[2,18]
   - 48.05048845651115 x[3,18] - 11.98742913143673 x[4,18]
   - 140.0142598839637 x[5,18] - 62.27191495688179 x[6,18]
   - 91.50782084051386 x[7,18] - 8.841856353164321 x[8,18]
   - 68.82169520075874 x[9,18] - 55.34463535943838 x[10,18]
   - 152.1279653642517 x[11,18] - 48.4945159762698 x[12,18]
   - 48.05048845651115 x[13,18] - 11.98742913143673 x[14,18]
   - 140.0142598839637 x[15,18] - 62.27191495688179 x[16,18]
   - 91.50782084051386 x[17,18] - 8.841856353164321 x[18,18]
   - 68.82169520075874 x[19,18] - 55.34463535943838 x[20,18]
   - 152.1279653642517 x[21,18] - 8.841856353164321 x[22,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 38.34390628751675 x[0,19]
   - 38.34390628751675 x[1,19] - 41.10637171335074 x[2,19]
   - 64.08538001260665 x[3,19] - 39.03271327619031 x[4,19]
   - 142.4260074553708 x[5,19] - 65.83329524994605 x[6,19]
   - 76.26358531475451 x[7,19] - 39.31964526640631 x[8,19]
   - 38.34390628751675 x[9,19] - 70.45932317596998 x[10,19]
   - 144.0824949766243 x[11,19] - 41.10637171335074 x[12,19]
   - 64.08538001260665 x[13,19] - 39.03271327619031 x[14,19]
   - 142.4260074553708 x[15,19] - 65.83329524994605 x[16,19]
   - 76.26358531475451 x[17,19] - 39.31964526640631 x[18,19]
   - 38.34390628751675 x[19,19] - 70.45932317596998 x[20,19]
   - 144.0824949766243 x[21,19] - 38.34390628751675 x[22,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 25.50155182670833 x[0,20]
   - 25.50155182670833 x[1,20] - 70.24300476710606 x[2,20]
   - 25.74073839927661 x[3,20] - 28.24290758138374 x[4,20]
   - 111.6636288998814 x[5,20] - 45.74268586082232 x[6,20]
   - 84.28543946821564 x[7,20] - 38.68493988589437 x[8,20]
   - 83.30167763677841 x[9,20] - 25.50155182670833 x[10,20]
   - 129.6730385043908 x[11,20] - 70.24300476710606 x[12,20]
   - 25.74073839927661 x[13,20] - 28.24290758138374 x[14,20]
   - 111.6636288998814 x[15,20] - 45.74268586082232 x[16,20]
   - 84.28543946821564 x[17,20] - 38.68493988589437 x[18,20]
   - 83.30167763677841 x[19,20] - 25.50155182670833 x[20,20]
   - 129.6730385043908 x[21,20] - 25.50155182670833 x[22,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 76.01293447457226 x[0,21]
   - 76.01293447457226 x[1,21] - 103.4993913512267 x[2,21]
   - 77.94298821356368 x[3,21] - 76.94733149220806 x[4,21]
   - 93.46675161098258 x[5,21] - 76.15848569012746 x[6,21]
   - 86.9491451583684 x[7,21] - 84.95688724284372 x[8,21]
   - 106.4134667895688 x[9,21] - 79.16165585652683 x[10,21]
   - 76.01293447457226 x[11,21] - 103.4993913512267 x[12,21]
   - 77.94298821356368 x[13,21] - 76.94733149220806 x[14,21]
   - 93.46675161098258 x[15,21] - 76.15848569012746 x[16,21]
   - 86.9491451583684 x[17,21] - 84.95688724284372 x[18,21]
   - 106.4134667895688 x[19,21] - 79.16165585652683 x[20,21]
   - 76.01293447457226 x[21,21] - 76.01293447457226 x[22,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 57.12897252162397 x[2,22]
   - 43.94250105099827 x[3,22] - 9.894293925924039 x[4,22]
   - 137.0255781850957 x[5,22] - 62.0821186314359 x[6,22]
   - 95.08893185064791 x[7,22] - 17.68371270632864 x[8,22]
   - 76.6878125750335 x[9,22] - 51.00310365341667 x[10,22]
   - 152.0258689491445 x[11,22] - 57.12897252162397 x[12,22]
   - 43.94250105099827 x[13,22] - 9.894293925924039 x[14,22]
   - 137.0255781850957 x[15,22] - 62.0821186314359 x[16,22]
   - 95.08893185064791 x[17,22] - 17.68371270632864 x[18,22]
   - 76.6878125750335 x[19,22] - 51.00310365341667 x[20,22]
   - 152.0258689491445 x[21,22] - start_service[0] + start_service[22]
   >= 0
 LowerBoundArrivalVehicle[23]: - 57.12897252162397 x[2,23]
   - 43.94250105099827 x[3,23] - 9.894293925924039 x[4,23]
   - 137.0255781850957 x[5,23] - 62.0821186314359 x[6,23]
   - 95.08893185064791 x[7,23] - 17.68371270632864 x[8,23]
   - 76.6878125750335 x[9,23] - 51.00310365341667 x[10,23]
   - 152.0258689491445 x[11,23] - 57.12897252162397 x[12,23]
   - 43.94250105099827 x[13,23] - 9.894293925924039 x[14,23]
   - 137.0255781850957 x[15,23] - 62.0821186314359 x[16,23]
   - 95.08893185064791 x[17,23] - 17.68371270632864 x[18,23]
   - 76.6878125750335 x[19,23] - 51.00310365341667 x[20,23]
   - 152.0258689491445 x[21,23] - start_service[0] + start_service[23]
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
