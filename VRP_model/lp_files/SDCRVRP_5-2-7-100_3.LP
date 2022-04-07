\ Model SDCRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[25,0] + 0 x[25,3] + 0 x[25,4] + 0 x[25,5] + 0 x[25,6] + 0 x[25,7]
   + 0 x[25,8] + 0 x[25,9] + 0 x[25,10] + 0 x[25,11] + 0 x[25,12]
   + 0 x[25,13] + 0 x[25,14] + 0 x[25,15] + 0 x[25,16] + 0 x[25,17]
   + 0 x[25,18] + 0 x[25,19] + 0 x[25,20] + 0 x[25,21] + 0 x[25,22]
   + 0 x[25,25] + 0 w[0,0] + 0 w[25,0] + 0 w[25,1] + 0 w[25,2] + 0 w[25,3]
   + 0 w[25,4] + 0 w[25,5] + 0 w[25,6] + 0 w[25,7] + 0 w[25,8] + 0 w[25,9]
   + 0 w[25,10] + 0 w[25,11] + 0 w[25,12] + 0 w[25,23] + 0 w[25,24]
   + 0 w[25,25] + 0 v[0,0] + 0 v[1,0] + 0 v[2,0] + 0 v[3,0] + 0 v[4,0]
   + 0 v[5,0] + 0 v[6,0] + 0 v[7,0] + 0 v[8,0] + 0 v[9,0] + 0 v[10,0]
   + 0 v[11,0] + 0 v[12,0] + 0 v[13,0] + 0 v[14,0] + 0 v[15,0] + 0 v[16,0]
   + 0 v[17,0] + 0 v[18,0] + 0 v[19,0] + 0 v[20,0] + 0 v[21,0] + 0 v[22,0]
   + 0 v[23,0] + 0 v[24,0] + 0 v[25,0] + 0 v[25,1] + 0 v[25,2] + 0 v[25,3]
   + 0 v[25,4] + 0 v[25,5] + 0 v[25,6] + 0 v[25,7] + 0 v[25,8] + 0 v[25,9]
   + 0 v[25,10] + 0 v[25,11] + 0 v[25,12] + 0 v[25,13] + 0 v[25,14]
   + 0 v[25,15] + 0 v[25,16] + 0 v[25,17] + 0 v[25,18] + 0 v[25,19]
   + 0 v[25,20] + 0 v[25,21] + 0 v[25,22] + 0 v[25,23] + 0 v[25,24]
   + 0 v[25,25] + start_service[25]
Subject To
 MaxNumberVehiclesStart[1]: x[0,0] + x[0,1] + x[0,2] + x[0,3] + x[0,4]
   + x[0,5] + x[0,6] + x[0,7] + x[0,13] + x[0,14] + x[0,15] + x[0,16]
   + x[0,17] + x[0,23] + x[0,24] + x[0,25] <= 1
 MaxNumberVehiclesStart[2]: x[0,0] + x[0,1] + x[0,2] + x[0,8] + x[0,9]
   + x[0,10] + x[0,11] + x[0,12] + x[0,18] + x[0,19] + x[0,20] + x[0,21]
   + x[0,22] + x[0,23] + x[0,24] + x[0,25] <= 1
 VisitNodesNotMoreThanOnce[1]: x[1,0] + x[1,1] + x[1,2] + x[1,3] + x[1,4]
   + x[1,5] + x[1,6] + x[1,7] + x[1,8] + x[1,9] + x[1,10] + x[1,11]
   + x[1,12] + x[1,13] + x[1,14] + x[1,15] + x[1,16] + x[1,17] + x[1,18]
   + x[1,19] + x[1,20] + x[1,21] + x[1,22] + x[1,23] + x[1,24] + x[1,25]
   <= 1
 VisitNodesNotMoreThanOnce[2]: x[2,0] + x[2,1] + x[2,2] + x[2,3] + x[2,4]
   + x[2,5] + x[2,6] + x[2,7] + x[2,8] + x[2,9] + x[2,10] + x[2,11]
   + x[2,12] + x[2,13] + x[2,14] + x[2,15] + x[2,16] + x[2,17] + x[2,18]
   + x[2,19] + x[2,20] + x[2,21] + x[2,22] + x[2,23] + x[2,24] + x[2,25]
   <= 1
 VisitNodesNotMoreThanOnce[3]: x[3,0] + x[3,1] + x[3,2] + x[3,3] + x[3,4]
   + x[3,5] + x[3,6] + x[3,7] + x[3,8] + x[3,9] + x[3,10] + x[3,11]
   + x[3,12] + x[3,13] + x[3,14] + x[3,15] + x[3,16] + x[3,17] + x[3,18]
   + x[3,19] + x[3,20] + x[3,21] + x[3,22] + x[3,23] + x[3,24] + x[3,25]
   <= 1
 VisitNodesNotMoreThanOnce[4]: x[4,0] + x[4,1] + x[4,2] + x[4,3] + x[4,4]
   + x[4,5] + x[4,6] + x[4,7] + x[4,8] + x[4,9] + x[4,10] + x[4,11]
   + x[4,12] + x[4,13] + x[4,14] + x[4,15] + x[4,16] + x[4,17] + x[4,18]
   + x[4,19] + x[4,20] + x[4,21] + x[4,22] + x[4,23] + x[4,24] + x[4,25]
   <= 1
 VisitNodesNotMoreThanOnce[5]: x[5,0] + x[5,1] + x[5,2] + x[5,3] + x[5,4]
   + x[5,5] + x[5,6] + x[5,7] + x[5,8] + x[5,9] + x[5,10] + x[5,11]
   + x[5,12] + x[5,13] + x[5,14] + x[5,15] + x[5,16] + x[5,17] + x[5,18]
   + x[5,19] + x[5,20] + x[5,21] + x[5,22] + x[5,23] + x[5,24] + x[5,25]
   <= 1
 VisitNodesNotMoreThanOnce[6]: x[6,0] + x[6,1] + x[6,2] + x[6,3] + x[6,4]
   + x[6,5] + x[6,6] + x[6,7] + x[6,8] + x[6,9] + x[6,10] + x[6,11]
   + x[6,12] + x[6,13] + x[6,14] + x[6,15] + x[6,16] + x[6,17] + x[6,18]
   + x[6,19] + x[6,20] + x[6,21] + x[6,22] + x[6,23] + x[6,24] + x[6,25]
   <= 1
 VisitNodesNotMoreThanOnce[7]: x[7,0] + x[7,1] + x[7,2] + x[7,3] + x[7,4]
   + x[7,5] + x[7,6] + x[7,7] + x[7,8] + x[7,9] + x[7,10] + x[7,11]
   + x[7,12] + x[7,13] + x[7,14] + x[7,15] + x[7,16] + x[7,17] + x[7,18]
   + x[7,19] + x[7,20] + x[7,21] + x[7,22] + x[7,23] + x[7,24] + x[7,25]
   <= 1
 VisitNodesNotMoreThanOnce[8]: x[8,0] + x[8,1] + x[8,2] + x[8,3] + x[8,4]
   + x[8,5] + x[8,6] + x[8,7] + x[8,8] + x[8,9] + x[8,10] + x[8,11]
   + x[8,12] + x[8,13] + x[8,14] + x[8,15] + x[8,16] + x[8,17] + x[8,18]
   + x[8,19] + x[8,20] + x[8,21] + x[8,22] + x[8,23] + x[8,24] + x[8,25]
   <= 1
 VisitNodesNotMoreThanOnce[9]: x[9,0] + x[9,1] + x[9,2] + x[9,3] + x[9,4]
   + x[9,5] + x[9,6] + x[9,7] + x[9,8] + x[9,9] + x[9,10] + x[9,11]
   + x[9,12] + x[9,13] + x[9,14] + x[9,15] + x[9,16] + x[9,17] + x[9,18]
   + x[9,19] + x[9,20] + x[9,21] + x[9,22] + x[9,23] + x[9,24] + x[9,25]
   <= 1
 VisitNodesNotMoreThanOnce[10]: x[10,0] + x[10,1] + x[10,2] + x[10,3]
   + x[10,4] + x[10,5] + x[10,6] + x[10,7] + x[10,8] + x[10,9] + x[10,10]
   + x[10,11] + x[10,12] + x[10,13] + x[10,14] + x[10,15] + x[10,16]
   + x[10,17] + x[10,18] + x[10,19] + x[10,20] + x[10,21] + x[10,22]
   + x[10,23] + x[10,24] + x[10,25] <= 1
 VisitNodesNotMoreThanOnce[11]: x[11,0] + x[11,1] + x[11,2] + x[11,3]
   + x[11,4] + x[11,5] + x[11,6] + x[11,7] + x[11,8] + x[11,9] + x[11,10]
   + x[11,11] + x[11,12] + x[11,13] + x[11,14] + x[11,15] + x[11,16]
   + x[11,17] + x[11,18] + x[11,19] + x[11,20] + x[11,21] + x[11,22]
   + x[11,23] + x[11,24] + x[11,25] <= 1
 VisitNodesNotMoreThanOnce[12]: x[12,0] + x[12,1] + x[12,2] + x[12,3]
   + x[12,4] + x[12,5] + x[12,6] + x[12,7] + x[12,8] + x[12,9] + x[12,10]
   + x[12,11] + x[12,12] + x[12,13] + x[12,14] + x[12,15] + x[12,16]
   + x[12,17] + x[12,18] + x[12,19] + x[12,20] + x[12,21] + x[12,22]
   + x[12,23] + x[12,24] + x[12,25] <= 1
 VisitNodesNotMoreThanOnce[13]: x[13,0] + x[13,1] + x[13,2] + x[13,3]
   + x[13,4] + x[13,5] + x[13,6] + x[13,7] + x[13,8] + x[13,9] + x[13,10]
   + x[13,11] + x[13,12] + x[13,13] + x[13,14] + x[13,15] + x[13,16]
   + x[13,17] + x[13,18] + x[13,19] + x[13,20] + x[13,21] + x[13,22]
   + x[13,23] + x[13,24] + x[13,25] <= 1
 VisitNodesNotMoreThanOnce[14]: x[14,0] + x[14,1] + x[14,2] + x[14,3]
   + x[14,4] + x[14,5] + x[14,6] + x[14,7] + x[14,8] + x[14,9] + x[14,10]
   + x[14,11] + x[14,12] + x[14,13] + x[14,14] + x[14,15] + x[14,16]
   + x[14,17] + x[14,18] + x[14,19] + x[14,20] + x[14,21] + x[14,22]
   + x[14,23] + x[14,24] + x[14,25] <= 1
 VisitNodesNotMoreThanOnce[15]: x[15,0] + x[15,1] + x[15,2] + x[15,3]
   + x[15,4] + x[15,5] + x[15,6] + x[15,7] + x[15,8] + x[15,9] + x[15,10]
   + x[15,11] + x[15,12] + x[15,13] + x[15,14] + x[15,15] + x[15,16]
   + x[15,17] + x[15,18] + x[15,19] + x[15,20] + x[15,21] + x[15,22]
   + x[15,23] + x[15,24] + x[15,25] <= 1
 VisitNodesNotMoreThanOnce[16]: x[16,0] + x[16,1] + x[16,2] + x[16,3]
   + x[16,4] + x[16,5] + x[16,6] + x[16,7] + x[16,8] + x[16,9] + x[16,10]
   + x[16,11] + x[16,12] + x[16,13] + x[16,14] + x[16,15] + x[16,16]
   + x[16,17] + x[16,18] + x[16,19] + x[16,20] + x[16,21] + x[16,22]
   + x[16,23] + x[16,24] + x[16,25] <= 1
 VisitNodesNotMoreThanOnce[17]: x[17,0] + x[17,1] + x[17,2] + x[17,3]
   + x[17,4] + x[17,5] + x[17,6] + x[17,7] + x[17,8] + x[17,9] + x[17,10]
   + x[17,11] + x[17,12] + x[17,13] + x[17,14] + x[17,15] + x[17,16]
   + x[17,17] + x[17,18] + x[17,19] + x[17,20] + x[17,21] + x[17,22]
   + x[17,23] + x[17,24] + x[17,25] <= 1
 VisitNodesNotMoreThanOnce[18]: x[18,0] + x[18,1] + x[18,2] + x[18,3]
   + x[18,4] + x[18,5] + x[18,6] + x[18,7] + x[18,8] + x[18,9] + x[18,10]
   + x[18,11] + x[18,12] + x[18,13] + x[18,14] + x[18,15] + x[18,16]
   + x[18,17] + x[18,18] + x[18,19] + x[18,20] + x[18,21] + x[18,22]
   + x[18,23] + x[18,24] + x[18,25] <= 1
 VisitNodesNotMoreThanOnce[19]: x[19,0] + x[19,1] + x[19,2] + x[19,3]
   + x[19,4] + x[19,5] + x[19,6] + x[19,7] + x[19,8] + x[19,9] + x[19,10]
   + x[19,11] + x[19,12] + x[19,13] + x[19,14] + x[19,15] + x[19,16]
   + x[19,17] + x[19,18] + x[19,19] + x[19,20] + x[19,21] + x[19,22]
   + x[19,23] + x[19,24] + x[19,25] <= 1
 VisitNodesNotMoreThanOnce[20]: x[20,0] + x[20,1] + x[20,2] + x[20,3]
   + x[20,4] + x[20,5] + x[20,6] + x[20,7] + x[20,8] + x[20,9] + x[20,10]
   + x[20,11] + x[20,12] + x[20,13] + x[20,14] + x[20,15] + x[20,16]
   + x[20,17] + x[20,18] + x[20,19] + x[20,20] + x[20,21] + x[20,22]
   + x[20,23] + x[20,24] + x[20,25] <= 1
 VisitNodesNotMoreThanOnce[21]: x[21,0] + x[21,1] + x[21,2] + x[21,3]
   + x[21,4] + x[21,5] + x[21,6] + x[21,7] + x[21,8] + x[21,9] + x[21,10]
   + x[21,11] + x[21,12] + x[21,13] + x[21,14] + x[21,15] + x[21,16]
   + x[21,17] + x[21,18] + x[21,19] + x[21,20] + x[21,21] + x[21,22]
   + x[21,23] + x[21,24] + x[21,25] <= 1
 VisitNodesNotMoreThanOnce[22]: x[22,0] + x[22,1] + x[22,2] + x[22,3]
   + x[22,4] + x[22,5] + x[22,6] + x[22,7] + x[22,8] + x[22,9] + x[22,10]
   + x[22,11] + x[22,12] + x[22,13] + x[22,14] + x[22,15] + x[22,16]
   + x[22,17] + x[22,18] + x[22,19] + x[22,20] + x[22,21] + x[22,22]
   + x[22,23] + x[22,24] + x[22,25] <= 1
 VisitNodesNotMoreThanOnce[23]: x[23,0] + x[23,1] + x[23,2] + x[23,3]
   + x[23,4] + x[23,5] + x[23,6] + x[23,7] + x[23,8] + x[23,9] + x[23,10]
   + x[23,11] + x[23,12] + x[23,13] + x[23,14] + x[23,15] + x[23,16]
   + x[23,17] + x[23,18] + x[23,19] + x[23,20] + x[23,21] + x[23,22]
   + x[23,23] + x[23,24] + x[23,25] <= 1
 VisitNodesNotMoreThanOnce[24]: x[24,0] + x[24,1] + x[24,2] + x[24,3]
   + x[24,4] + x[24,5] + x[24,6] + x[24,7] + x[24,8] + x[24,9] + x[24,10]
   + x[24,11] + x[24,12] + x[24,13] + x[24,14] + x[24,15] + x[24,16]
   + x[24,17] + x[24,18] + x[24,19] + x[24,20] + x[24,21] + x[24,22]
   + x[24,23] + x[24,24] + x[24,25] <= 1
 FlowVehicles[1]: x[0,1] - x[1,2] - x[1,3] - x[1,4] - x[1,5] - x[1,6]
   - x[1,7] - x[1,8] - x[1,9] - x[1,10] - x[1,11] - x[1,12] - x[1,13]
   - x[1,14] - x[1,15] - x[1,16] - x[1,17] - x[1,18] - x[1,19] - x[1,20]
   - x[1,21] - x[1,22] - x[1,23] - x[1,24] - x[1,25] + x[2,1] + x[3,1]
   + x[4,1] + x[5,1] + x[6,1] + x[7,1] + x[8,1] + x[9,1] + x[10,1]
   + x[11,1] + x[12,1] + x[13,1] + x[14,1] + x[15,1] + x[16,1] + x[17,1]
   + x[18,1] + x[19,1] + x[20,1] + x[21,1] + x[22,1] + x[23,1] + x[24,1]
   = 0
 FlowVehicles[2]: x[0,2] + x[1,2] - x[2,1] - x[2,3] - x[2,4] - x[2,5]
   - x[2,6] - x[2,7] - x[2,8] - x[2,9] - x[2,10] - x[2,11] - x[2,12]
   - x[2,13] - x[2,14] - x[2,15] - x[2,16] - x[2,17] - x[2,18] - x[2,19]
   - x[2,20] - x[2,21] - x[2,22] - x[2,23] - x[2,24] - x[2,25] + x[3,2]
   + x[4,2] + x[5,2] + x[6,2] + x[7,2] + x[8,2] + x[9,2] + x[10,2]
   + x[11,2] + x[12,2] + x[13,2] + x[14,2] + x[15,2] + x[16,2] + x[17,2]
   + x[18,2] + x[19,2] + x[20,2] + x[21,2] + x[22,2] + x[23,2] + x[24,2]
   = 0
 FlowVehicles[3]: x[0,3] + x[1,3] + x[2,3] - x[3,1] - x[3,2] - x[3,4]
   - x[3,5] - x[3,6] - x[3,7] - x[3,8] - x[3,9] - x[3,10] - x[3,11]
   - x[3,12] - x[3,13] - x[3,14] - x[3,15] - x[3,16] - x[3,17] - x[3,18]
   - x[3,19] - x[3,20] - x[3,21] - x[3,22] - x[3,23] - x[3,24] - x[3,25]
   + x[4,3] + x[5,3] + x[6,3] + x[7,3] + x[8,3] + x[9,3] + x[10,3]
   + x[11,3] + x[12,3] + x[13,3] + x[14,3] + x[15,3] + x[16,3] + x[17,3]
   + x[18,3] + x[19,3] + x[20,3] + x[21,3] + x[22,3] + x[23,3] + x[24,3]
   = 0
 FlowVehicles[4]: x[0,4] + x[1,4] + x[2,4] + x[3,4] - x[4,1] - x[4,2]
   - x[4,3] - x[4,5] - x[4,6] - x[4,7] - x[4,8] - x[4,9] - x[4,10]
   - x[4,11] - x[4,12] - x[4,13] - x[4,14] - x[4,15] - x[4,16] - x[4,17]
   - x[4,18] - x[4,19] - x[4,20] - x[4,21] - x[4,22] - x[4,23] - x[4,24]
   - x[4,25] + x[5,4] + x[6,4] + x[7,4] + x[8,4] + x[9,4] + x[10,4]
   + x[11,4] + x[12,4] + x[13,4] + x[14,4] + x[15,4] + x[16,4] + x[17,4]
   + x[18,4] + x[19,4] + x[20,4] + x[21,4] + x[22,4] + x[23,4] + x[24,4]
   = 0
 FlowVehicles[5]: x[0,5] + x[1,5] + x[2,5] + x[3,5] + x[4,5] - x[5,1]
   - x[5,2] - x[5,3] - x[5,4] - x[5,6] - x[5,7] - x[5,8] - x[5,9] - x[5,10]
   - x[5,11] - x[5,12] - x[5,13] - x[5,14] - x[5,15] - x[5,16] - x[5,17]
   - x[5,18] - x[5,19] - x[5,20] - x[5,21] - x[5,22] - x[5,23] - x[5,24]
   - x[5,25] + x[6,5] + x[7,5] + x[8,5] + x[9,5] + x[10,5] + x[11,5]
   + x[12,5] + x[13,5] + x[14,5] + x[15,5] + x[16,5] + x[17,5] + x[18,5]
   + x[19,5] + x[20,5] + x[21,5] + x[22,5] + x[23,5] + x[24,5] = 0
 FlowVehicles[6]: x[0,6] + x[1,6] + x[2,6] + x[3,6] + x[4,6] + x[5,6]
   - x[6,1] - x[6,2] - x[6,3] - x[6,4] - x[6,5] - x[6,7] - x[6,8] - x[6,9]
   - x[6,10] - x[6,11] - x[6,12] - x[6,13] - x[6,14] - x[6,15] - x[6,16]
   - x[6,17] - x[6,18] - x[6,19] - x[6,20] - x[6,21] - x[6,22] - x[6,23]
   - x[6,24] - x[6,25] + x[7,6] + x[8,6] + x[9,6] + x[10,6] + x[11,6]
   + x[12,6] + x[13,6] + x[14,6] + x[15,6] + x[16,6] + x[17,6] + x[18,6]
   + x[19,6] + x[20,6] + x[21,6] + x[22,6] + x[23,6] + x[24,6] = 0
 FlowVehicles[7]: x[0,7] + x[1,7] + x[2,7] + x[3,7] + x[4,7] + x[5,7]
   + x[6,7] - x[7,1] - x[7,2] - x[7,3] - x[7,4] - x[7,5] - x[7,6] - x[7,8]
   - x[7,9] - x[7,10] - x[7,11] - x[7,12] - x[7,13] - x[7,14] - x[7,15]
   - x[7,16] - x[7,17] - x[7,18] - x[7,19] - x[7,20] - x[7,21] - x[7,22]
   - x[7,23] - x[7,24] - x[7,25] + x[8,7] + x[9,7] + x[10,7] + x[11,7]
   + x[12,7] + x[13,7] + x[14,7] + x[15,7] + x[16,7] + x[17,7] + x[18,7]
   + x[19,7] + x[20,7] + x[21,7] + x[22,7] + x[23,7] + x[24,7] = 0
 FlowVehicles[8]: x[0,8] + x[1,8] + x[2,8] + x[3,8] + x[4,8] + x[5,8]
   + x[6,8] + x[7,8] - x[8,1] - x[8,2] - x[8,3] - x[8,4] - x[8,5] - x[8,6]
   - x[8,7] - x[8,9] - x[8,10] - x[8,11] - x[8,12] - x[8,13] - x[8,14]
   - x[8,15] - x[8,16] - x[8,17] - x[8,18] - x[8,19] - x[8,20] - x[8,21]
   - x[8,22] - x[8,23] - x[8,24] - x[8,25] + x[9,8] + x[10,8] + x[11,8]
   + x[12,8] + x[13,8] + x[14,8] + x[15,8] + x[16,8] + x[17,8] + x[18,8]
   + x[19,8] + x[20,8] + x[21,8] + x[22,8] + x[23,8] + x[24,8] = 0
 FlowVehicles[9]: x[0,9] + x[1,9] + x[2,9] + x[3,9] + x[4,9] + x[5,9]
   + x[6,9] + x[7,9] + x[8,9] - x[9,1] - x[9,2] - x[9,3] - x[9,4] - x[9,5]
   - x[9,6] - x[9,7] - x[9,8] - x[9,10] - x[9,11] - x[9,12] - x[9,13]
   - x[9,14] - x[9,15] - x[9,16] - x[9,17] - x[9,18] - x[9,19] - x[9,20]
   - x[9,21] - x[9,22] - x[9,23] - x[9,24] - x[9,25] + x[10,9] + x[11,9]
   + x[12,9] + x[13,9] + x[14,9] + x[15,9] + x[16,9] + x[17,9] + x[18,9]
   + x[19,9] + x[20,9] + x[21,9] + x[22,9] + x[23,9] + x[24,9] = 0
 FlowVehicles[10]: x[0,10] + x[1,10] + x[2,10] + x[3,10] + x[4,10]
   + x[5,10] + x[6,10] + x[7,10] + x[8,10] + x[9,10] - x[10,1] - x[10,2]
   - x[10,3] - x[10,4] - x[10,5] - x[10,6] - x[10,7] - x[10,8] - x[10,9]
   - x[10,11] - x[10,12] - x[10,13] - x[10,14] - x[10,15] - x[10,16]
   - x[10,17] - x[10,18] - x[10,19] - x[10,20] - x[10,21] - x[10,22]
   - x[10,23] - x[10,24] - x[10,25] + x[11,10] + x[12,10] + x[13,10]
   + x[14,10] + x[15,10] + x[16,10] + x[17,10] + x[18,10] + x[19,10]
   + x[20,10] + x[21,10] + x[22,10] + x[23,10] + x[24,10] = 0
 FlowVehicles[11]: x[0,11] + x[1,11] + x[2,11] + x[3,11] + x[4,11]
   + x[5,11] + x[6,11] + x[7,11] + x[8,11] + x[9,11] + x[10,11] - x[11,1]
   - x[11,2] - x[11,3] - x[11,4] - x[11,5] - x[11,6] - x[11,7] - x[11,8]
   - x[11,9] - x[11,10] - x[11,12] - x[11,13] - x[11,14] - x[11,15]
   - x[11,16] - x[11,17] - x[11,18] - x[11,19] - x[11,20] - x[11,21]
   - x[11,22] - x[11,23] - x[11,24] - x[11,25] + x[12,11] + x[13,11]
   + x[14,11] + x[15,11] + x[16,11] + x[17,11] + x[18,11] + x[19,11]
   + x[20,11] + x[21,11] + x[22,11] + x[23,11] + x[24,11] = 0
 FlowVehicles[12]: x[0,12] + x[1,12] + x[2,12] + x[3,12] + x[4,12]
   + x[5,12] + x[6,12] + x[7,12] + x[8,12] + x[9,12] + x[10,12] + x[11,12]
   - x[12,1] - x[12,2] - x[12,3] - x[12,4] - x[12,5] - x[12,6] - x[12,7]
   - x[12,8] - x[12,9] - x[12,10] - x[12,11] - x[12,13] - x[12,14]
   - x[12,15] - x[12,16] - x[12,17] - x[12,18] - x[12,19] - x[12,20]
   - x[12,21] - x[12,22] - x[12,23] - x[12,24] - x[12,25] + x[13,12]
   + x[14,12] + x[15,12] + x[16,12] + x[17,12] + x[18,12] + x[19,12]
   + x[20,12] + x[21,12] + x[22,12] + x[23,12] + x[24,12] = 0
 FlowVehicles[13]: x[0,13] + x[1,13] + x[2,13] + x[3,13] + x[4,13]
   + x[5,13] + x[6,13] + x[7,13] + x[8,13] + x[9,13] + x[10,13] + x[11,13]
   + x[12,13] - x[13,1] - x[13,2] - x[13,3] - x[13,4] - x[13,5] - x[13,6]
   - x[13,7] - x[13,8] - x[13,9] - x[13,10] - x[13,11] - x[13,12]
   - x[13,14] - x[13,15] - x[13,16] - x[13,17] - x[13,18] - x[13,19]
   - x[13,20] - x[13,21] - x[13,22] - x[13,23] - x[13,24] - x[13,25]
   + x[14,13] + x[15,13] + x[16,13] + x[17,13] + x[18,13] + x[19,13]
   + x[20,13] + x[21,13] + x[22,13] + x[23,13] + x[24,13] = 0
 FlowVehicles[14]: x[0,14] + x[1,14] + x[2,14] + x[3,14] + x[4,14]
   + x[5,14] + x[6,14] + x[7,14] + x[8,14] + x[9,14] + x[10,14] + x[11,14]
   + x[12,14] + x[13,14] - x[14,1] - x[14,2] - x[14,3] - x[14,4] - x[14,5]
   - x[14,6] - x[14,7] - x[14,8] - x[14,9] - x[14,10] - x[14,11] - x[14,12]
   - x[14,13] - x[14,15] - x[14,16] - x[14,17] - x[14,18] - x[14,19]
   - x[14,20] - x[14,21] - x[14,22] - x[14,23] - x[14,24] - x[14,25]
   + x[15,14] + x[16,14] + x[17,14] + x[18,14] + x[19,14] + x[20,14]
   + x[21,14] + x[22,14] + x[23,14] + x[24,14] = 0
 FlowVehicles[15]: x[0,15] + x[1,15] + x[2,15] + x[3,15] + x[4,15]
   + x[5,15] + x[6,15] + x[7,15] + x[8,15] + x[9,15] + x[10,15] + x[11,15]
   + x[12,15] + x[13,15] + x[14,15] - x[15,1] - x[15,2] - x[15,3] - x[15,4]
   - x[15,5] - x[15,6] - x[15,7] - x[15,8] - x[15,9] - x[15,10] - x[15,11]
   - x[15,12] - x[15,13] - x[15,14] - x[15,16] - x[15,17] - x[15,18]
   - x[15,19] - x[15,20] - x[15,21] - x[15,22] - x[15,23] - x[15,24]
   - x[15,25] + x[16,15] + x[17,15] + x[18,15] + x[19,15] + x[20,15]
   + x[21,15] + x[22,15] + x[23,15] + x[24,15] = 0
 FlowVehicles[16]: x[0,16] + x[1,16] + x[2,16] + x[3,16] + x[4,16]
   + x[5,16] + x[6,16] + x[7,16] + x[8,16] + x[9,16] + x[10,16] + x[11,16]
   + x[12,16] + x[13,16] + x[14,16] + x[15,16] - x[16,1] - x[16,2]
   - x[16,3] - x[16,4] - x[16,5] - x[16,6] - x[16,7] - x[16,8] - x[16,9]
   - x[16,10] - x[16,11] - x[16,12] - x[16,13] - x[16,14] - x[16,15]
   - x[16,17] - x[16,18] - x[16,19] - x[16,20] - x[16,21] - x[16,22]
   - x[16,23] - x[16,24] - x[16,25] + x[17,16] + x[18,16] + x[19,16]
   + x[20,16] + x[21,16] + x[22,16] + x[23,16] + x[24,16] = 0
 FlowVehicles[17]: x[0,17] + x[1,17] + x[2,17] + x[3,17] + x[4,17]
   + x[5,17] + x[6,17] + x[7,17] + x[8,17] + x[9,17] + x[10,17] + x[11,17]
   + x[12,17] + x[13,17] + x[14,17] + x[15,17] + x[16,17] - x[17,1]
   - x[17,2] - x[17,3] - x[17,4] - x[17,5] - x[17,6] - x[17,7] - x[17,8]
   - x[17,9] - x[17,10] - x[17,11] - x[17,12] - x[17,13] - x[17,14]
   - x[17,15] - x[17,16] - x[17,18] - x[17,19] - x[17,20] - x[17,21]
   - x[17,22] - x[17,23] - x[17,24] - x[17,25] + x[18,17] + x[19,17]
   + x[20,17] + x[21,17] + x[22,17] + x[23,17] + x[24,17] = 0
 FlowVehicles[18]: x[0,18] + x[1,18] + x[2,18] + x[3,18] + x[4,18]
   + x[5,18] + x[6,18] + x[7,18] + x[8,18] + x[9,18] + x[10,18] + x[11,18]
   + x[12,18] + x[13,18] + x[14,18] + x[15,18] + x[16,18] + x[17,18]
   - x[18,1] - x[18,2] - x[18,3] - x[18,4] - x[18,5] - x[18,6] - x[18,7]
   - x[18,8] - x[18,9] - x[18,10] - x[18,11] - x[18,12] - x[18,13]
   - x[18,14] - x[18,15] - x[18,16] - x[18,17] - x[18,19] - x[18,20]
   - x[18,21] - x[18,22] - x[18,23] - x[18,24] - x[18,25] + x[19,18]
   + x[20,18] + x[21,18] + x[22,18] + x[23,18] + x[24,18] = 0
 FlowVehicles[19]: x[0,19] + x[1,19] + x[2,19] + x[3,19] + x[4,19]
   + x[5,19] + x[6,19] + x[7,19] + x[8,19] + x[9,19] + x[10,19] + x[11,19]
   + x[12,19] + x[13,19] + x[14,19] + x[15,19] + x[16,19] + x[17,19]
   + x[18,19] - x[19,1] - x[19,2] - x[19,3] - x[19,4] - x[19,5] - x[19,6]
   - x[19,7] - x[19,8] - x[19,9] - x[19,10] - x[19,11] - x[19,12]
   - x[19,13] - x[19,14] - x[19,15] - x[19,16] - x[19,17] - x[19,18]
   - x[19,20] - x[19,21] - x[19,22] - x[19,23] - x[19,24] - x[19,25]
   + x[20,19] + x[21,19] + x[22,19] + x[23,19] + x[24,19] = 0
 FlowVehicles[20]: x[0,20] + x[1,20] + x[2,20] + x[3,20] + x[4,20]
   + x[5,20] + x[6,20] + x[7,20] + x[8,20] + x[9,20] + x[10,20] + x[11,20]
   + x[12,20] + x[13,20] + x[14,20] + x[15,20] + x[16,20] + x[17,20]
   + x[18,20] + x[19,20] - x[20,1] - x[20,2] - x[20,3] - x[20,4] - x[20,5]
   - x[20,6] - x[20,7] - x[20,8] - x[20,9] - x[20,10] - x[20,11] - x[20,12]
   - x[20,13] - x[20,14] - x[20,15] - x[20,16] - x[20,17] - x[20,18]
   - x[20,19] - x[20,21] - x[20,22] - x[20,23] - x[20,24] - x[20,25]
   + x[21,20] + x[22,20] + x[23,20] + x[24,20] = 0
 FlowVehicles[21]: x[0,21] + x[1,21] + x[2,21] + x[3,21] + x[4,21]
   + x[5,21] + x[6,21] + x[7,21] + x[8,21] + x[9,21] + x[10,21] + x[11,21]
   + x[12,21] + x[13,21] + x[14,21] + x[15,21] + x[16,21] + x[17,21]
   + x[18,21] + x[19,21] + x[20,21] - x[21,1] - x[21,2] - x[21,3] - x[21,4]
   - x[21,5] - x[21,6] - x[21,7] - x[21,8] - x[21,9] - x[21,10] - x[21,11]
   - x[21,12] - x[21,13] - x[21,14] - x[21,15] - x[21,16] - x[21,17]
   - x[21,18] - x[21,19] - x[21,20] - x[21,22] - x[21,23] - x[21,24]
   - x[21,25] + x[22,21] + x[23,21] + x[24,21] = 0
 FlowVehicles[22]: x[0,22] + x[1,22] + x[2,22] + x[3,22] + x[4,22]
   + x[5,22] + x[6,22] + x[7,22] + x[8,22] + x[9,22] + x[10,22] + x[11,22]
   + x[12,22] + x[13,22] + x[14,22] + x[15,22] + x[16,22] + x[17,22]
   + x[18,22] + x[19,22] + x[20,22] + x[21,22] - x[22,1] - x[22,2]
   - x[22,3] - x[22,4] - x[22,5] - x[22,6] - x[22,7] - x[22,8] - x[22,9]
   - x[22,10] - x[22,11] - x[22,12] - x[22,13] - x[22,14] - x[22,15]
   - x[22,16] - x[22,17] - x[22,18] - x[22,19] - x[22,20] - x[22,21]
   - x[22,23] - x[22,24] - x[22,25] + x[23,22] + x[24,22] = 0
 FlowVehicles[23]: x[0,23] + x[1,23] + x[2,23] + x[3,23] + x[4,23]
   + x[5,23] + x[6,23] + x[7,23] + x[8,23] + x[9,23] + x[10,23] + x[11,23]
   + x[12,23] + x[13,23] + x[14,23] + x[15,23] + x[16,23] + x[17,23]
   + x[18,23] + x[19,23] + x[20,23] + x[21,23] + x[22,23] - x[23,1]
   - x[23,2] - x[23,3] - x[23,4] - x[23,5] - x[23,6] - x[23,7] - x[23,8]
   - x[23,9] - x[23,10] - x[23,11] - x[23,12] - x[23,13] - x[23,14]
   - x[23,15] - x[23,16] - x[23,17] - x[23,18] - x[23,19] - x[23,20]
   - x[23,21] - x[23,22] - x[23,24] - x[23,25] + x[24,23] = 0
 FlowVehicles[24]: x[0,24] + x[1,24] + x[2,24] + x[3,24] + x[4,24]
   + x[5,24] + x[6,24] + x[7,24] + x[8,24] + x[9,24] + x[10,24] + x[11,24]
   + x[12,24] + x[13,24] + x[14,24] + x[15,24] + x[16,24] + x[17,24]
   + x[18,24] + x[19,24] + x[20,24] + x[21,24] + x[22,24] + x[23,24]
   - x[24,1] - x[24,2] - x[24,3] - x[24,4] - x[24,5] - x[24,6] - x[24,7]
   - x[24,8] - x[24,9] - x[24,10] - x[24,11] - x[24,12] - x[24,13]
   - x[24,14] - x[24,15] - x[24,16] - x[24,17] - x[24,18] - x[24,19]
   - x[24,20] - x[24,21] - x[24,22] - x[24,23] - x[24,25] = 0
 NodeArrivalVehicles[0,1]: - 999999.01 x[0,1] - start_service[0]
   + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 999999.01 x[0,2] - start_service[0]
   + start_service[2] >= -999999
 NodeArrivalVehicles[0,3]: - 1.0000538900827599e+06 x[0,3]
   - start_service[0] + start_service[3] >= -999999
 NodeArrivalVehicles[0,4]: - 1.0000315560591351e+06 x[0,4]
   - start_service[0] + start_service[4] >= -999999
 NodeArrivalVehicles[0,5]: - 1.0000086606162182e+06 x[0,5]
   - start_service[0] + start_service[5] >= -999999
 NodeArrivalVehicles[0,6]: - 1.0000880721248598e+06 x[0,6]
   - start_service[0] + start_service[6] >= -999999
 NodeArrivalVehicles[0,7]: - 1.0000120359099368e+06 x[0,7]
   - start_service[0] + start_service[7] >= -999999
 NodeArrivalVehicles[0,8]: - 1.0000538900827599e+06 x[0,8]
   - start_service[0] + start_service[8] >= -999999
 NodeArrivalVehicles[0,9]: - 1.0000315560591351e+06 x[0,9]
   - start_service[0] + start_service[9] >= -999999
 NodeArrivalVehicles[0,10]: - 1.0000086606162182e+06 x[0,10]
   - start_service[0] + start_service[10] >= -999999
 NodeArrivalVehicles[0,11]: - 1.0000880721248598e+06 x[0,11]
   - start_service[0] + start_service[11] >= -999999
 NodeArrivalVehicles[0,12]: - 1.0000120359099368e+06 x[0,12]
   - start_service[0] + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 1.0000538900827599e+06 x[0,13]
   - start_service[0] + start_service[13] >= -999999
 NodeArrivalVehicles[0,14]: - 1.0000315560591351e+06 x[0,14]
   - start_service[0] + start_service[14] >= -999999
 NodeArrivalVehicles[0,15]: - 1.0000086606162182e+06 x[0,15]
   - start_service[0] + start_service[15] >= -999999
 NodeArrivalVehicles[0,16]: - 1.0000880721248598e+06 x[0,16]
   - start_service[0] + start_service[16] >= -999999
 NodeArrivalVehicles[0,17]: - 1.0000120359099368e+06 x[0,17]
   - start_service[0] + start_service[17] >= -999999
 NodeArrivalVehicles[0,18]: - 1.0000538900827599e+06 x[0,18]
   - start_service[0] + start_service[18] >= -999999
 NodeArrivalVehicles[0,19]: - 1.0000315560591351e+06 x[0,19]
   - start_service[0] + start_service[19] >= -999999
 NodeArrivalVehicles[0,20]: - 1.0000086606162182e+06 x[0,20]
   - start_service[0] + start_service[20] >= -999999
 NodeArrivalVehicles[0,21]: - 1.0000880721248598e+06 x[0,21]
   - start_service[0] + start_service[21] >= -999999
 NodeArrivalVehicles[0,22]: - 1.0000120359099368e+06 x[0,22]
   - start_service[0] + start_service[22] >= -999999
 NodeArrivalVehicles[0,23]: - 999999.01 x[0,23] - start_service[0]
   + start_service[23] >= -999999
 NodeArrivalVehicles[0,24]: - 999999.01 x[0,24] - start_service[0]
   + start_service[24] >= -999999
 NodeArrivalVehicles[0,25]: - 999999.01 x[0,25] - start_service[0]
   + start_service[25] >= -999999
 NodeArrivalVehicles[1,1]: - 999999.01 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999.01 x[1,2] - start_service[1]
   + start_service[2] >= -999999
 NodeArrivalVehicles[1,3]: - 1.0000538900827599e+06 x[1,3]
   - start_service[1] + start_service[3] >= -999999
 NodeArrivalVehicles[1,4]: - 1.0000315560591351e+06 x[1,4]
   - start_service[1] + start_service[4] >= -999999
 NodeArrivalVehicles[1,5]: - 1.0000086606162182e+06 x[1,5]
   - start_service[1] + start_service[5] >= -999999
 NodeArrivalVehicles[1,6]: - 1.0000880721248598e+06 x[1,6]
   - start_service[1] + start_service[6] >= -999999
 NodeArrivalVehicles[1,7]: - 1.0000120359099368e+06 x[1,7]
   - start_service[1] + start_service[7] >= -999999
 NodeArrivalVehicles[1,8]: - 1.0000538900827599e+06 x[1,8]
   - start_service[1] + start_service[8] >= -999999
 NodeArrivalVehicles[1,9]: - 1.0000315560591351e+06 x[1,9]
   - start_service[1] + start_service[9] >= -999999
 NodeArrivalVehicles[1,10]: - 1.0000086606162182e+06 x[1,10]
   - start_service[1] + start_service[10] >= -999999
 NodeArrivalVehicles[1,11]: - 1.0000880721248598e+06 x[1,11]
   - start_service[1] + start_service[11] >= -999999
 NodeArrivalVehicles[1,12]: - 1.0000120359099368e+06 x[1,12]
   - start_service[1] + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 1.0000538900827599e+06 x[1,13]
   - start_service[1] + start_service[13] >= -999999
 NodeArrivalVehicles[1,14]: - 1.0000315560591351e+06 x[1,14]
   - start_service[1] + start_service[14] >= -999999
 NodeArrivalVehicles[1,15]: - 1.0000086606162182e+06 x[1,15]
   - start_service[1] + start_service[15] >= -999999
 NodeArrivalVehicles[1,16]: - 1.0000880721248598e+06 x[1,16]
   - start_service[1] + start_service[16] >= -999999
 NodeArrivalVehicles[1,17]: - 1.0000120359099368e+06 x[1,17]
   - start_service[1] + start_service[17] >= -999999
 NodeArrivalVehicles[1,18]: - 1.0000538900827599e+06 x[1,18]
   - start_service[1] + start_service[18] >= -999999
 NodeArrivalVehicles[1,19]: - 1.0000315560591351e+06 x[1,19]
   - start_service[1] + start_service[19] >= -999999
 NodeArrivalVehicles[1,20]: - 1.0000086606162182e+06 x[1,20]
   - start_service[1] + start_service[20] >= -999999
 NodeArrivalVehicles[1,21]: - 1.0000880721248598e+06 x[1,21]
   - start_service[1] + start_service[21] >= -999999
 NodeArrivalVehicles[1,22]: - 1.0000120359099368e+06 x[1,22]
   - start_service[1] + start_service[22] >= -999999
 NodeArrivalVehicles[1,23]: - 999999.01 x[1,23] - start_service[1]
   + start_service[23] >= -999999
 NodeArrivalVehicles[1,24]: - 999999.01 x[1,24] - start_service[1]
   + start_service[24] >= -999999
 NodeArrivalVehicles[1,25]: - 999999.01 x[1,25] - start_service[1]
   + start_service[25] >= -999999
 NodeArrivalVehicles[2,1]: - 999999.01 x[2,1] + start_service[1]
   - start_service[2] >= -999999
 NodeArrivalVehicles[2,2]: - 999999.01 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 1.0000538900827599e+06 x[2,3]
   - start_service[2] + start_service[3] >= -999999
 NodeArrivalVehicles[2,4]: - 1.0000315560591351e+06 x[2,4]
   - start_service[2] + start_service[4] >= -999999
 NodeArrivalVehicles[2,5]: - 1.0000086606162182e+06 x[2,5]
   - start_service[2] + start_service[5] >= -999999
 NodeArrivalVehicles[2,6]: - 1.0000880721248598e+06 x[2,6]
   - start_service[2] + start_service[6] >= -999999
 NodeArrivalVehicles[2,7]: - 1.0000120359099368e+06 x[2,7]
   - start_service[2] + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 1.0000538900827599e+06 x[2,8]
   - start_service[2] + start_service[8] >= -999999
 NodeArrivalVehicles[2,9]: - 1.0000315560591351e+06 x[2,9]
   - start_service[2] + start_service[9] >= -999999
 NodeArrivalVehicles[2,10]: - 1.0000086606162182e+06 x[2,10]
   - start_service[2] + start_service[10] >= -999999
 NodeArrivalVehicles[2,11]: - 1.0000880721248598e+06 x[2,11]
   - start_service[2] + start_service[11] >= -999999
 NodeArrivalVehicles[2,12]: - 1.0000120359099368e+06 x[2,12]
   - start_service[2] + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 1.0000538900827599e+06 x[2,13]
   - start_service[2] + start_service[13] >= -999999
 NodeArrivalVehicles[2,14]: - 1.0000315560591351e+06 x[2,14]
   - start_service[2] + start_service[14] >= -999999
 NodeArrivalVehicles[2,15]: - 1.0000086606162182e+06 x[2,15]
   - start_service[2] + start_service[15] >= -999999
 NodeArrivalVehicles[2,16]: - 1.0000880721248598e+06 x[2,16]
   - start_service[2] + start_service[16] >= -999999
 NodeArrivalVehicles[2,17]: - 1.0000120359099368e+06 x[2,17]
   - start_service[2] + start_service[17] >= -999999
 NodeArrivalVehicles[2,18]: - 1.0000538900827599e+06 x[2,18]
   - start_service[2] + start_service[18] >= -999999
 NodeArrivalVehicles[2,19]: - 1.0000315560591351e+06 x[2,19]
   - start_service[2] + start_service[19] >= -999999
 NodeArrivalVehicles[2,20]: - 1.0000086606162182e+06 x[2,20]
   - start_service[2] + start_service[20] >= -999999
 NodeArrivalVehicles[2,21]: - 1.0000880721248598e+06 x[2,21]
   - start_service[2] + start_service[21] >= -999999
 NodeArrivalVehicles[2,22]: - 1.0000120359099368e+06 x[2,22]
   - start_service[2] + start_service[22] >= -999999
 NodeArrivalVehicles[2,23]: - 999999.01 x[2,23] - start_service[2]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,24]: - 999999.01 x[2,24] - start_service[2]
   + start_service[24] >= -999999
 NodeArrivalVehicles[2,25]: - 999999.01 x[2,25] - start_service[2]
   + start_service[25] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000538900827599e+06 x[3,1]
   + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.0000538900827599e+06 x[3,2]
   + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999.01 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0000399322151314e+06 x[3,4]
   - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.0000562206776621e+06 x[3,5]
   - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.0000706995536003e+06 x[3,6]
   - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.0000408655636616e+06 x[3,7]
   - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 999999.01 x[3,8] - start_service[3]
   + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0000399322151314e+06 x[3,9]
   - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.0000562206776621e+06 x[3,10]
   - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.0000706995536003e+06 x[3,11]
   - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 1.0000408655636616e+06 x[3,12]
   - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 999999.01 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 1.0000399322151314e+06 x[3,14]
   - start_service[3] + start_service[14] >= -999999
 NodeArrivalVehicles[3,15]: - 1.0000562206776621e+06 x[3,15]
   - start_service[3] + start_service[15] >= -999999
 NodeArrivalVehicles[3,16]: - 1.0000706995536003e+06 x[3,16]
   - start_service[3] + start_service[16] >= -999999
 NodeArrivalVehicles[3,17]: - 1.0000408655636616e+06 x[3,17]
   - start_service[3] + start_service[17] >= -999999
 NodeArrivalVehicles[3,18]: - 999999.01 x[3,18] - start_service[3]
   + start_service[18] >= -999999
 NodeArrivalVehicles[3,19]: - 1.0000399322151314e+06 x[3,19]
   - start_service[3] + start_service[19] >= -999999
 NodeArrivalVehicles[3,20]: - 1.0000562206776621e+06 x[3,20]
   - start_service[3] + start_service[20] >= -999999
 NodeArrivalVehicles[3,21]: - 1.0000706995536003e+06 x[3,21]
   - start_service[3] + start_service[21] >= -999999
 NodeArrivalVehicles[3,22]: - 1.0000408655636616e+06 x[3,22]
   - start_service[3] + start_service[22] >= -999999
 NodeArrivalVehicles[3,23]: - 1.0000538900827599e+06 x[3,23]
   - start_service[3] + start_service[23] >= -999999
 NodeArrivalVehicles[3,24]: - 1.0000538900827599e+06 x[3,24]
   - start_service[3] + start_service[24] >= -999999
 NodeArrivalVehicles[3,25]: - 1.0000538900827599e+06 x[3,25]
   - start_service[3] + start_service[25] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000315560591351e+06 x[4,1]
   + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.0000315560591351e+06 x[4,2]
   + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0000399322151314e+06 x[4,3]
   + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999.01 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.0000399960725273e+06 x[4,5]
   - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000557694957714e+06 x[4,6]
   - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000242562644748e+06 x[4,7]
   - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0000399322151314e+06 x[4,8]
   - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 999999.01 x[4,9] - start_service[4]
   + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.0000399960725273e+06 x[4,10]
   - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000557694957714e+06 x[4,11]
   - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 1.0000242562644748e+06 x[4,12]
   - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 1.0000399322151314e+06 x[4,13]
   - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[4,14]: - 999999.01 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 1.0000399960725273e+06 x[4,15]
   - start_service[4] + start_service[15] >= -999999
 NodeArrivalVehicles[4,16]: - 1.0000557694957714e+06 x[4,16]
   - start_service[4] + start_service[16] >= -999999
 NodeArrivalVehicles[4,17]: - 1.0000242562644748e+06 x[4,17]
   - start_service[4] + start_service[17] >= -999999
 NodeArrivalVehicles[4,18]: - 1.0000399322151314e+06 x[4,18]
   - start_service[4] + start_service[18] >= -999999
 NodeArrivalVehicles[4,19]: - 999999.01 x[4,19] - start_service[4]
   + start_service[19] >= -999999
 NodeArrivalVehicles[4,20]: - 1.0000399960725273e+06 x[4,20]
   - start_service[4] + start_service[20] >= -999999
 NodeArrivalVehicles[4,21]: - 1.0000557694957714e+06 x[4,21]
   - start_service[4] + start_service[21] >= -999999
 NodeArrivalVehicles[4,22]: - 1.0000242562644748e+06 x[4,22]
   - start_service[4] + start_service[22] >= -999999
 NodeArrivalVehicles[4,23]: - 1.0000315560591351e+06 x[4,23]
   - start_service[4] + start_service[23] >= -999999
 NodeArrivalVehicles[4,24]: - 1.0000315560591351e+06 x[4,24]
   - start_service[4] + start_service[24] >= -999999
 NodeArrivalVehicles[4,25]: - 1.0000315560591351e+06 x[4,25]
   - start_service[4] + start_service[25] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000086606162182e+06 x[5,1]
   + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000086606162182e+06 x[5,2]
   + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.0000562206776621e+06 x[5,3]
   + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.0000399960725273e+06 x[5,4]
   + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999.01 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000967563598768e+06 x[5,6]
   - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000166692290857e+06 x[5,7]
   - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.0000562206776621e+06 x[5,8]
   - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.0000399960725273e+06 x[5,9]
   - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 999999.01 x[5,10] - start_service[5]
   + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000967563598768e+06 x[5,11]
   - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 1.0000166692290857e+06 x[5,12]
   - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 1.0000562206776621e+06 x[5,13]
   - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[5,14]: - 1.0000399960725273e+06 x[5,14]
   - start_service[5] + start_service[14] >= -999999
 NodeArrivalVehicles[5,15]: - 999999.01 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 1.0000967563598768e+06 x[5,16]
   - start_service[5] + start_service[16] >= -999999
 NodeArrivalVehicles[5,17]: - 1.0000166692290857e+06 x[5,17]
   - start_service[5] + start_service[17] >= -999999
 NodeArrivalVehicles[5,18]: - 1.0000562206776621e+06 x[5,18]
   - start_service[5] + start_service[18] >= -999999
 NodeArrivalVehicles[5,19]: - 1.0000399960725273e+06 x[5,19]
   - start_service[5] + start_service[19] >= -999999
 NodeArrivalVehicles[5,20]: - 999999.01 x[5,20] - start_service[5]
   + start_service[20] >= -999999
 NodeArrivalVehicles[5,21]: - 1.0000967563598768e+06 x[5,21]
   - start_service[5] + start_service[21] >= -999999
 NodeArrivalVehicles[5,22]: - 1.0000166692290857e+06 x[5,22]
   - start_service[5] + start_service[22] >= -999999
 NodeArrivalVehicles[5,23]: - 1.0000086606162182e+06 x[5,23]
   - start_service[5] + start_service[23] >= -999999
 NodeArrivalVehicles[5,24]: - 1.0000086606162182e+06 x[5,24]
   - start_service[5] + start_service[24] >= -999999
 NodeArrivalVehicles[5,25]: - 1.0000086606162182e+06 x[5,25]
   - start_service[5] + start_service[25] >= -999999
 NodeArrivalVehicles[6,1]: - 1.0000880721248598e+06 x[6,1]
   + start_service[1] - start_service[6] >= -999999
 NodeArrivalVehicles[6,2]: - 1.0000880721248598e+06 x[6,2]
   + start_service[2] - start_service[6] >= -999999
 NodeArrivalVehicles[6,3]: - 1.0000706995536003e+06 x[6,3]
   + start_service[3] - start_service[6] >= -999999
 NodeArrivalVehicles[6,4]: - 1.0000557694957714e+06 x[6,4]
   + start_service[4] - start_service[6] >= -999999
 NodeArrivalVehicles[6,5]: - 1.0000967563598768e+06 x[6,5]
   + start_service[5] - start_service[6] >= -999999
 NodeArrivalVehicles[6,6]: - 999999.01 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000805943163345e+06 x[6,7]
   - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.0000706995536003e+06 x[6,8]
   - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000557694957714e+06 x[6,9]
   - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000967563598768e+06 x[6,10]
   - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 999999.01 x[6,11] - start_service[6]
   + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 1.0000805943163345e+06 x[6,12]
   - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 1.0000706995536003e+06 x[6,13]
   - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[6,14]: - 1.0000557694957714e+06 x[6,14]
   - start_service[6] + start_service[14] >= -999999
 NodeArrivalVehicles[6,15]: - 1.0000967563598768e+06 x[6,15]
   - start_service[6] + start_service[15] >= -999999
 NodeArrivalVehicles[6,16]: - 999999.01 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 1.0000805943163345e+06 x[6,17]
   - start_service[6] + start_service[17] >= -999999
 NodeArrivalVehicles[6,18]: - 1.0000706995536003e+06 x[6,18]
   - start_service[6] + start_service[18] >= -999999
 NodeArrivalVehicles[6,19]: - 1.0000557694957714e+06 x[6,19]
   - start_service[6] + start_service[19] >= -999999
 NodeArrivalVehicles[6,20]: - 1.0000967563598768e+06 x[6,20]
   - start_service[6] + start_service[20] >= -999999
 NodeArrivalVehicles[6,21]: - 999999.01 x[6,21] - start_service[6]
   + start_service[21] >= -999999
 NodeArrivalVehicles[6,22]: - 1.0000805943163345e+06 x[6,22]
   - start_service[6] + start_service[22] >= -999999
 NodeArrivalVehicles[6,23]: - 1.0000880721248598e+06 x[6,23]
   - start_service[6] + start_service[23] >= -999999
 NodeArrivalVehicles[6,24]: - 1.0000880721248598e+06 x[6,24]
   - start_service[6] + start_service[24] >= -999999
 NodeArrivalVehicles[6,25]: - 1.0000880721248598e+06 x[6,25]
   - start_service[6] + start_service[25] >= -999999
 NodeArrivalVehicles[7,1]: - 1.0000120359099368e+06 x[7,1]
   + start_service[1] - start_service[7] >= -999999
 NodeArrivalVehicles[7,2]: - 1.0000120359099368e+06 x[7,2]
   + start_service[2] - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 1.0000408655636616e+06 x[7,3]
   + start_service[3] - start_service[7] >= -999999
 NodeArrivalVehicles[7,4]: - 1.0000242562644748e+06 x[7,4]
   + start_service[4] - start_service[7] >= -999999
 NodeArrivalVehicles[7,5]: - 1.0000166692290857e+06 x[7,5]
   + start_service[5] - start_service[7] >= -999999
 NodeArrivalVehicles[7,6]: - 1.0000805943163345e+06 x[7,6]
   + start_service[6] - start_service[7] >= -999999
 NodeArrivalVehicles[7,7]: - 999999.01 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.0000408655636616e+06 x[7,8]
   - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 1.0000242562644748e+06 x[7,9]
   - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.0000166692290857e+06 x[7,10]
   - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000805943163345e+06 x[7,11]
   - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 999999.01 x[7,12] - start_service[7]
   + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 1.0000408655636616e+06 x[7,13]
   - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[7,14]: - 1.0000242562644748e+06 x[7,14]
   - start_service[7] + start_service[14] >= -999999
 NodeArrivalVehicles[7,15]: - 1.0000166692290857e+06 x[7,15]
   - start_service[7] + start_service[15] >= -999999
 NodeArrivalVehicles[7,16]: - 1.0000805943163345e+06 x[7,16]
   - start_service[7] + start_service[16] >= -999999
 NodeArrivalVehicles[7,17]: - 999999.01 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 1.0000408655636616e+06 x[7,18]
   - start_service[7] + start_service[18] >= -999999
 NodeArrivalVehicles[7,19]: - 1.0000242562644748e+06 x[7,19]
   - start_service[7] + start_service[19] >= -999999
 NodeArrivalVehicles[7,20]: - 1.0000166692290857e+06 x[7,20]
   - start_service[7] + start_service[20] >= -999999
 NodeArrivalVehicles[7,21]: - 1.0000805943163345e+06 x[7,21]
   - start_service[7] + start_service[21] >= -999999
 NodeArrivalVehicles[7,22]: - 999999.01 x[7,22] - start_service[7]
   + start_service[22] >= -999999
 NodeArrivalVehicles[7,23]: - 1.0000120359099368e+06 x[7,23]
   - start_service[7] + start_service[23] >= -999999
 NodeArrivalVehicles[7,24]: - 1.0000120359099368e+06 x[7,24]
   - start_service[7] + start_service[24] >= -999999
 NodeArrivalVehicles[7,25]: - 1.0000120359099368e+06 x[7,25]
   - start_service[7] + start_service[25] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000538900827599e+06 x[8,1]
   + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.0000538900827599e+06 x[8,2]
   + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 999999.01 x[8,3] + start_service[3]
   - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0000399322151314e+06 x[8,4]
   + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.0000562206776621e+06 x[8,5]
   + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.0000706995536003e+06 x[8,6]
   + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.0000408655636616e+06 x[8,7]
   + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999.01 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0000399322151314e+06 x[8,9]
   - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.0000562206776621e+06 x[8,10]
   - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.0000706995536003e+06 x[8,11]
   - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 1.0000408655636616e+06 x[8,12]
   - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 999999.01 x[8,13] - start_service[8]
   + start_service[13] >= -999999
 NodeArrivalVehicles[8,14]: - 1.0000399322151314e+06 x[8,14]
   - start_service[8] + start_service[14] >= -999999
 NodeArrivalVehicles[8,15]: - 1.0000562206776621e+06 x[8,15]
   - start_service[8] + start_service[15] >= -999999
 NodeArrivalVehicles[8,16]: - 1.0000706995536003e+06 x[8,16]
   - start_service[8] + start_service[16] >= -999999
 NodeArrivalVehicles[8,17]: - 1.0000408655636616e+06 x[8,17]
   - start_service[8] + start_service[17] >= -999999
 NodeArrivalVehicles[8,18]: - 999999.01 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 1.0000399322151314e+06 x[8,19]
   - start_service[8] + start_service[19] >= -999999
 NodeArrivalVehicles[8,20]: - 1.0000562206776621e+06 x[8,20]
   - start_service[8] + start_service[20] >= -999999
 NodeArrivalVehicles[8,21]: - 1.0000706995536003e+06 x[8,21]
   - start_service[8] + start_service[21] >= -999999
 NodeArrivalVehicles[8,22]: - 1.0000408655636616e+06 x[8,22]
   - start_service[8] + start_service[22] >= -999999
 NodeArrivalVehicles[8,23]: - 1.0000538900827599e+06 x[8,23]
   - start_service[8] + start_service[23] >= -999999
 NodeArrivalVehicles[8,24]: - 1.0000538900827599e+06 x[8,24]
   - start_service[8] + start_service[24] >= -999999
 NodeArrivalVehicles[8,25]: - 1.0000538900827599e+06 x[8,25]
   - start_service[8] + start_service[25] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0000315560591351e+06 x[9,1]
   + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000315560591351e+06 x[9,2]
   + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0000399322151314e+06 x[9,3]
   + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 999999.01 x[9,4] + start_service[4]
   - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.0000399960725273e+06 x[9,5]
   + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000557694957714e+06 x[9,6]
   + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 1.0000242562644748e+06 x[9,7]
   + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0000399322151314e+06 x[9,8]
   + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999.01 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.0000399960725273e+06 x[9,10]
   - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000557694957714e+06 x[9,11]
   - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 1.0000242562644748e+06 x[9,12]
   - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 1.0000399322151314e+06 x[9,13]
   - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[9,14]: - 999999.01 x[9,14] - start_service[9]
   + start_service[14] >= -999999
 NodeArrivalVehicles[9,15]: - 1.0000399960725273e+06 x[9,15]
   - start_service[9] + start_service[15] >= -999999
 NodeArrivalVehicles[9,16]: - 1.0000557694957714e+06 x[9,16]
   - start_service[9] + start_service[16] >= -999999
 NodeArrivalVehicles[9,17]: - 1.0000242562644748e+06 x[9,17]
   - start_service[9] + start_service[17] >= -999999
 NodeArrivalVehicles[9,18]: - 1.0000399322151314e+06 x[9,18]
   - start_service[9] + start_service[18] >= -999999
 NodeArrivalVehicles[9,19]: - 999999.01 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 1.0000399960725273e+06 x[9,20]
   - start_service[9] + start_service[20] >= -999999
 NodeArrivalVehicles[9,21]: - 1.0000557694957714e+06 x[9,21]
   - start_service[9] + start_service[21] >= -999999
 NodeArrivalVehicles[9,22]: - 1.0000242562644748e+06 x[9,22]
   - start_service[9] + start_service[22] >= -999999
 NodeArrivalVehicles[9,23]: - 1.0000315560591351e+06 x[9,23]
   - start_service[9] + start_service[23] >= -999999
 NodeArrivalVehicles[9,24]: - 1.0000315560591351e+06 x[9,24]
   - start_service[9] + start_service[24] >= -999999
 NodeArrivalVehicles[9,25]: - 1.0000315560591351e+06 x[9,25]
   - start_service[9] + start_service[25] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000086606162182e+06 x[10,1]
   + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000086606162182e+06 x[10,2]
   + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.0000562206776621e+06 x[10,3]
   + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.0000399960725273e+06 x[10,4]
   + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 999999.01 x[10,5] + start_service[5]
   - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000967563598768e+06 x[10,6]
   + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.0000166692290857e+06 x[10,7]
   + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.0000562206776621e+06 x[10,8]
   + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.0000399960725273e+06 x[10,9]
   + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999.01 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000967563598768e+06 x[10,11]
   - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 1.0000166692290857e+06 x[10,12]
   - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 1.0000562206776621e+06 x[10,13]
   - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[10,14]: - 1.0000399960725273e+06 x[10,14]
   - start_service[10] + start_service[14] >= -999999
 NodeArrivalVehicles[10,15]: - 999999.01 x[10,15] - start_service[10]
   + start_service[15] >= -999999
 NodeArrivalVehicles[10,16]: - 1.0000967563598768e+06 x[10,16]
   - start_service[10] + start_service[16] >= -999999
 NodeArrivalVehicles[10,17]: - 1.0000166692290857e+06 x[10,17]
   - start_service[10] + start_service[17] >= -999999
 NodeArrivalVehicles[10,18]: - 1.0000562206776621e+06 x[10,18]
   - start_service[10] + start_service[18] >= -999999
 NodeArrivalVehicles[10,19]: - 1.0000399960725273e+06 x[10,19]
   - start_service[10] + start_service[19] >= -999999
 NodeArrivalVehicles[10,20]: - 999999.01 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 1.0000967563598768e+06 x[10,21]
   - start_service[10] + start_service[21] >= -999999
 NodeArrivalVehicles[10,22]: - 1.0000166692290857e+06 x[10,22]
   - start_service[10] + start_service[22] >= -999999
 NodeArrivalVehicles[10,23]: - 1.0000086606162182e+06 x[10,23]
   - start_service[10] + start_service[23] >= -999999
 NodeArrivalVehicles[10,24]: - 1.0000086606162182e+06 x[10,24]
   - start_service[10] + start_service[24] >= -999999
 NodeArrivalVehicles[10,25]: - 1.0000086606162182e+06 x[10,25]
   - start_service[10] + start_service[25] >= -999999
 NodeArrivalVehicles[11,1]: - 1.0000880721248598e+06 x[11,1]
   + start_service[1] - start_service[11] >= -999999
 NodeArrivalVehicles[11,2]: - 1.0000880721248598e+06 x[11,2]
   + start_service[2] - start_service[11] >= -999999
 NodeArrivalVehicles[11,3]: - 1.0000706995536003e+06 x[11,3]
   + start_service[3] - start_service[11] >= -999999
 NodeArrivalVehicles[11,4]: - 1.0000557694957714e+06 x[11,4]
   + start_service[4] - start_service[11] >= -999999
 NodeArrivalVehicles[11,5]: - 1.0000967563598768e+06 x[11,5]
   + start_service[5] - start_service[11] >= -999999
 NodeArrivalVehicles[11,6]: - 999999.01 x[11,6] + start_service[6]
   - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000805943163345e+06 x[11,7]
   + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.0000706995536003e+06 x[11,8]
   + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000557694957714e+06 x[11,9]
   + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000967563598768e+06 x[11,10]
   + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999.01 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 1.0000805943163345e+06 x[11,12]
   - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 1.0000706995536003e+06 x[11,13]
   - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[11,14]: - 1.0000557694957714e+06 x[11,14]
   - start_service[11] + start_service[14] >= -999999
 NodeArrivalVehicles[11,15]: - 1.0000967563598768e+06 x[11,15]
   - start_service[11] + start_service[15] >= -999999
 NodeArrivalVehicles[11,16]: - 999999.01 x[11,16] - start_service[11]
   + start_service[16] >= -999999
 NodeArrivalVehicles[11,17]: - 1.0000805943163345e+06 x[11,17]
   - start_service[11] + start_service[17] >= -999999
 NodeArrivalVehicles[11,18]: - 1.0000706995536003e+06 x[11,18]
   - start_service[11] + start_service[18] >= -999999
 NodeArrivalVehicles[11,19]: - 1.0000557694957714e+06 x[11,19]
   - start_service[11] + start_service[19] >= -999999
 NodeArrivalVehicles[11,20]: - 1.0000967563598768e+06 x[11,20]
   - start_service[11] + start_service[20] >= -999999
 NodeArrivalVehicles[11,21]: - 999999.01 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 1.0000805943163345e+06 x[11,22]
   - start_service[11] + start_service[22] >= -999999
 NodeArrivalVehicles[11,23]: - 1.0000880721248598e+06 x[11,23]
   - start_service[11] + start_service[23] >= -999999
 NodeArrivalVehicles[11,24]: - 1.0000880721248598e+06 x[11,24]
   - start_service[11] + start_service[24] >= -999999
 NodeArrivalVehicles[11,25]: - 1.0000880721248598e+06 x[11,25]
   - start_service[11] + start_service[25] >= -999999
 NodeArrivalVehicles[12,1]: - 1.0000120359099368e+06 x[12,1]
   + start_service[1] - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 1.0000120359099368e+06 x[12,2]
   + start_service[2] - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 1.0000408655636616e+06 x[12,3]
   + start_service[3] - start_service[12] >= -999999
 NodeArrivalVehicles[12,4]: - 1.0000242562644748e+06 x[12,4]
   + start_service[4] - start_service[12] >= -999999
 NodeArrivalVehicles[12,5]: - 1.0000166692290857e+06 x[12,5]
   + start_service[5] - start_service[12] >= -999999
 NodeArrivalVehicles[12,6]: - 1.0000805943163345e+06 x[12,6]
   + start_service[6] - start_service[12] >= -999999
 NodeArrivalVehicles[12,7]: - 999999.01 x[12,7] + start_service[7]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000408655636616e+06 x[12,8]
   + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000242562644748e+06 x[12,9]
   + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000166692290857e+06 x[12,10]
   + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.0000805943163345e+06 x[12,11]
   + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999.01 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 1.0000408655636616e+06 x[12,13]
   - start_service[12] + start_service[13] >= -999999
 NodeArrivalVehicles[12,14]: - 1.0000242562644748e+06 x[12,14]
   - start_service[12] + start_service[14] >= -999999
 NodeArrivalVehicles[12,15]: - 1.0000166692290857e+06 x[12,15]
   - start_service[12] + start_service[15] >= -999999
 NodeArrivalVehicles[12,16]: - 1.0000805943163345e+06 x[12,16]
   - start_service[12] + start_service[16] >= -999999
 NodeArrivalVehicles[12,17]: - 999999.01 x[12,17] - start_service[12]
   + start_service[17] >= -999999
 NodeArrivalVehicles[12,18]: - 1.0000408655636616e+06 x[12,18]
   - start_service[12] + start_service[18] >= -999999
 NodeArrivalVehicles[12,19]: - 1.0000242562644748e+06 x[12,19]
   - start_service[12] + start_service[19] >= -999999
 NodeArrivalVehicles[12,20]: - 1.0000166692290857e+06 x[12,20]
   - start_service[12] + start_service[20] >= -999999
 NodeArrivalVehicles[12,21]: - 1.0000805943163345e+06 x[12,21]
   - start_service[12] + start_service[21] >= -999999
 NodeArrivalVehicles[12,22]: - 999999.01 x[12,22] - start_service[12]
   + start_service[22] >= -999999
 NodeArrivalVehicles[12,23]: - 1.0000120359099368e+06 x[12,23]
   - start_service[12] + start_service[23] >= -999999
 NodeArrivalVehicles[12,24]: - 1.0000120359099368e+06 x[12,24]
   - start_service[12] + start_service[24] >= -999999
 NodeArrivalVehicles[12,25]: - 1.0000120359099368e+06 x[12,25]
   - start_service[12] + start_service[25] >= -999999
 NodeArrivalVehicles[13,1]: - 1.0000538900827599e+06 x[13,1]
   + start_service[1] - start_service[13] >= -999999
 NodeArrivalVehicles[13,2]: - 1.0000538900827599e+06 x[13,2]
   + start_service[2] - start_service[13] >= -999999
 NodeArrivalVehicles[13,3]: - 999999.01 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 1.0000399322151314e+06 x[13,4]
   + start_service[4] - start_service[13] >= -999999
 NodeArrivalVehicles[13,5]: - 1.0000562206776621e+06 x[13,5]
   + start_service[5] - start_service[13] >= -999999
 NodeArrivalVehicles[13,6]: - 1.0000706995536003e+06 x[13,6]
   + start_service[6] - start_service[13] >= -999999
 NodeArrivalVehicles[13,7]: - 1.0000408655636616e+06 x[13,7]
   + start_service[7] - start_service[13] >= -999999
 NodeArrivalVehicles[13,8]: - 999999.01 x[13,8] + start_service[8]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,9]: - 1.0000399322151314e+06 x[13,9]
   + start_service[9] - start_service[13] >= -999999
 NodeArrivalVehicles[13,10]: - 1.0000562206776621e+06 x[13,10]
   + start_service[10] - start_service[13] >= -999999
 NodeArrivalVehicles[13,11]: - 1.0000706995536003e+06 x[13,11]
   + start_service[11] - start_service[13] >= -999999
 NodeArrivalVehicles[13,12]: - 1.0000408655636616e+06 x[13,12]
   + start_service[12] - start_service[13] >= -999999
 NodeArrivalVehicles[13,13]: - 999999.01 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 1.0000399322151314e+06 x[13,14]
   - start_service[13] + start_service[14] >= -999999
 NodeArrivalVehicles[13,15]: - 1.0000562206776621e+06 x[13,15]
   - start_service[13] + start_service[15] >= -999999
 NodeArrivalVehicles[13,16]: - 1.0000706995536003e+06 x[13,16]
   - start_service[13] + start_service[16] >= -999999
 NodeArrivalVehicles[13,17]: - 1.0000408655636616e+06 x[13,17]
   - start_service[13] + start_service[17] >= -999999
 NodeArrivalVehicles[13,18]: - 999999.01 x[13,18] - start_service[13]
   + start_service[18] >= -999999
 NodeArrivalVehicles[13,19]: - 1.0000399322151314e+06 x[13,19]
   - start_service[13] + start_service[19] >= -999999
 NodeArrivalVehicles[13,20]: - 1.0000562206776621e+06 x[13,20]
   - start_service[13] + start_service[20] >= -999999
 NodeArrivalVehicles[13,21]: - 1.0000706995536003e+06 x[13,21]
   - start_service[13] + start_service[21] >= -999999
 NodeArrivalVehicles[13,22]: - 1.0000408655636616e+06 x[13,22]
   - start_service[13] + start_service[22] >= -999999
 NodeArrivalVehicles[13,23]: - 1.0000538900827599e+06 x[13,23]
   - start_service[13] + start_service[23] >= -999999
 NodeArrivalVehicles[13,24]: - 1.0000538900827599e+06 x[13,24]
   - start_service[13] + start_service[24] >= -999999
 NodeArrivalVehicles[13,25]: - 1.0000538900827599e+06 x[13,25]
   - start_service[13] + start_service[25] >= -999999
 NodeArrivalVehicles[14,1]: - 1.0000315560591351e+06 x[14,1]
   + start_service[1] - start_service[14] >= -999999
 NodeArrivalVehicles[14,2]: - 1.0000315560591351e+06 x[14,2]
   + start_service[2] - start_service[14] >= -999999
 NodeArrivalVehicles[14,3]: - 1.0000399322151314e+06 x[14,3]
   + start_service[3] - start_service[14] >= -999999
 NodeArrivalVehicles[14,4]: - 999999.01 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 1.0000399960725273e+06 x[14,5]
   + start_service[5] - start_service[14] >= -999999
 NodeArrivalVehicles[14,6]: - 1.0000557694957714e+06 x[14,6]
   + start_service[6] - start_service[14] >= -999999
 NodeArrivalVehicles[14,7]: - 1.0000242562644748e+06 x[14,7]
   + start_service[7] - start_service[14] >= -999999
 NodeArrivalVehicles[14,8]: - 1.0000399322151314e+06 x[14,8]
   + start_service[8] - start_service[14] >= -999999
 NodeArrivalVehicles[14,9]: - 999999.01 x[14,9] + start_service[9]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,10]: - 1.0000399960725273e+06 x[14,10]
   + start_service[10] - start_service[14] >= -999999
 NodeArrivalVehicles[14,11]: - 1.0000557694957714e+06 x[14,11]
   + start_service[11] - start_service[14] >= -999999
 NodeArrivalVehicles[14,12]: - 1.0000242562644748e+06 x[14,12]
   + start_service[12] - start_service[14] >= -999999
 NodeArrivalVehicles[14,13]: - 1.0000399322151314e+06 x[14,13]
   + start_service[13] - start_service[14] >= -999999
 NodeArrivalVehicles[14,14]: - 999999.01 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 1.0000399960725273e+06 x[14,15]
   - start_service[14] + start_service[15] >= -999999
 NodeArrivalVehicles[14,16]: - 1.0000557694957714e+06 x[14,16]
   - start_service[14] + start_service[16] >= -999999
 NodeArrivalVehicles[14,17]: - 1.0000242562644748e+06 x[14,17]
   - start_service[14] + start_service[17] >= -999999
 NodeArrivalVehicles[14,18]: - 1.0000399322151314e+06 x[14,18]
   - start_service[14] + start_service[18] >= -999999
 NodeArrivalVehicles[14,19]: - 999999.01 x[14,19] - start_service[14]
   + start_service[19] >= -999999
 NodeArrivalVehicles[14,20]: - 1.0000399960725273e+06 x[14,20]
   - start_service[14] + start_service[20] >= -999999
 NodeArrivalVehicles[14,21]: - 1.0000557694957714e+06 x[14,21]
   - start_service[14] + start_service[21] >= -999999
 NodeArrivalVehicles[14,22]: - 1.0000242562644748e+06 x[14,22]
   - start_service[14] + start_service[22] >= -999999
 NodeArrivalVehicles[14,23]: - 1.0000315560591351e+06 x[14,23]
   - start_service[14] + start_service[23] >= -999999
 NodeArrivalVehicles[14,24]: - 1.0000315560591351e+06 x[14,24]
   - start_service[14] + start_service[24] >= -999999
 NodeArrivalVehicles[14,25]: - 1.0000315560591351e+06 x[14,25]
   - start_service[14] + start_service[25] >= -999999
 NodeArrivalVehicles[15,1]: - 1.0000086606162182e+06 x[15,1]
   + start_service[1] - start_service[15] >= -999999
 NodeArrivalVehicles[15,2]: - 1.0000086606162182e+06 x[15,2]
   + start_service[2] - start_service[15] >= -999999
 NodeArrivalVehicles[15,3]: - 1.0000562206776621e+06 x[15,3]
   + start_service[3] - start_service[15] >= -999999
 NodeArrivalVehicles[15,4]: - 1.0000399960725273e+06 x[15,4]
   + start_service[4] - start_service[15] >= -999999
 NodeArrivalVehicles[15,5]: - 999999.01 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 1.0000967563598768e+06 x[15,6]
   + start_service[6] - start_service[15] >= -999999
 NodeArrivalVehicles[15,7]: - 1.0000166692290857e+06 x[15,7]
   + start_service[7] - start_service[15] >= -999999
 NodeArrivalVehicles[15,8]: - 1.0000562206776621e+06 x[15,8]
   + start_service[8] - start_service[15] >= -999999
 NodeArrivalVehicles[15,9]: - 1.0000399960725273e+06 x[15,9]
   + start_service[9] - start_service[15] >= -999999
 NodeArrivalVehicles[15,10]: - 999999.01 x[15,10] + start_service[10]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,11]: - 1.0000967563598768e+06 x[15,11]
   + start_service[11] - start_service[15] >= -999999
 NodeArrivalVehicles[15,12]: - 1.0000166692290857e+06 x[15,12]
   + start_service[12] - start_service[15] >= -999999
 NodeArrivalVehicles[15,13]: - 1.0000562206776621e+06 x[15,13]
   + start_service[13] - start_service[15] >= -999999
 NodeArrivalVehicles[15,14]: - 1.0000399960725273e+06 x[15,14]
   + start_service[14] - start_service[15] >= -999999
 NodeArrivalVehicles[15,15]: - 999999.01 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 1.0000967563598768e+06 x[15,16]
   - start_service[15] + start_service[16] >= -999999
 NodeArrivalVehicles[15,17]: - 1.0000166692290857e+06 x[15,17]
   - start_service[15] + start_service[17] >= -999999
 NodeArrivalVehicles[15,18]: - 1.0000562206776621e+06 x[15,18]
   - start_service[15] + start_service[18] >= -999999
 NodeArrivalVehicles[15,19]: - 1.0000399960725273e+06 x[15,19]
   - start_service[15] + start_service[19] >= -999999
 NodeArrivalVehicles[15,20]: - 999999.01 x[15,20] - start_service[15]
   + start_service[20] >= -999999
 NodeArrivalVehicles[15,21]: - 1.0000967563598768e+06 x[15,21]
   - start_service[15] + start_service[21] >= -999999
 NodeArrivalVehicles[15,22]: - 1.0000166692290857e+06 x[15,22]
   - start_service[15] + start_service[22] >= -999999
 NodeArrivalVehicles[15,23]: - 1.0000086606162182e+06 x[15,23]
   - start_service[15] + start_service[23] >= -999999
 NodeArrivalVehicles[15,24]: - 1.0000086606162182e+06 x[15,24]
   - start_service[15] + start_service[24] >= -999999
 NodeArrivalVehicles[15,25]: - 1.0000086606162182e+06 x[15,25]
   - start_service[15] + start_service[25] >= -999999
 NodeArrivalVehicles[16,1]: - 1.0000880721248598e+06 x[16,1]
   + start_service[1] - start_service[16] >= -999999
 NodeArrivalVehicles[16,2]: - 1.0000880721248598e+06 x[16,2]
   + start_service[2] - start_service[16] >= -999999
 NodeArrivalVehicles[16,3]: - 1.0000706995536003e+06 x[16,3]
   + start_service[3] - start_service[16] >= -999999
 NodeArrivalVehicles[16,4]: - 1.0000557694957714e+06 x[16,4]
   + start_service[4] - start_service[16] >= -999999
 NodeArrivalVehicles[16,5]: - 1.0000967563598768e+06 x[16,5]
   + start_service[5] - start_service[16] >= -999999
 NodeArrivalVehicles[16,6]: - 999999.01 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 1.0000805943163345e+06 x[16,7]
   + start_service[7] - start_service[16] >= -999999
 NodeArrivalVehicles[16,8]: - 1.0000706995536003e+06 x[16,8]
   + start_service[8] - start_service[16] >= -999999
 NodeArrivalVehicles[16,9]: - 1.0000557694957714e+06 x[16,9]
   + start_service[9] - start_service[16] >= -999999
 NodeArrivalVehicles[16,10]: - 1.0000967563598768e+06 x[16,10]
   + start_service[10] - start_service[16] >= -999999
 NodeArrivalVehicles[16,11]: - 999999.01 x[16,11] + start_service[11]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,12]: - 1.0000805943163345e+06 x[16,12]
   + start_service[12] - start_service[16] >= -999999
 NodeArrivalVehicles[16,13]: - 1.0000706995536003e+06 x[16,13]
   + start_service[13] - start_service[16] >= -999999
 NodeArrivalVehicles[16,14]: - 1.0000557694957714e+06 x[16,14]
   + start_service[14] - start_service[16] >= -999999
 NodeArrivalVehicles[16,15]: - 1.0000967563598768e+06 x[16,15]
   + start_service[15] - start_service[16] >= -999999
 NodeArrivalVehicles[16,16]: - 999999.01 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 1.0000805943163345e+06 x[16,17]
   - start_service[16] + start_service[17] >= -999999
 NodeArrivalVehicles[16,18]: - 1.0000706995536003e+06 x[16,18]
   - start_service[16] + start_service[18] >= -999999
 NodeArrivalVehicles[16,19]: - 1.0000557694957714e+06 x[16,19]
   - start_service[16] + start_service[19] >= -999999
 NodeArrivalVehicles[16,20]: - 1.0000967563598768e+06 x[16,20]
   - start_service[16] + start_service[20] >= -999999
 NodeArrivalVehicles[16,21]: - 999999.01 x[16,21] - start_service[16]
   + start_service[21] >= -999999
 NodeArrivalVehicles[16,22]: - 1.0000805943163345e+06 x[16,22]
   - start_service[16] + start_service[22] >= -999999
 NodeArrivalVehicles[16,23]: - 1.0000880721248598e+06 x[16,23]
   - start_service[16] + start_service[23] >= -999999
 NodeArrivalVehicles[16,24]: - 1.0000880721248598e+06 x[16,24]
   - start_service[16] + start_service[24] >= -999999
 NodeArrivalVehicles[16,25]: - 1.0000880721248598e+06 x[16,25]
   - start_service[16] + start_service[25] >= -999999
 NodeArrivalVehicles[17,1]: - 1.0000120359099368e+06 x[17,1]
   + start_service[1] - start_service[17] >= -999999
 NodeArrivalVehicles[17,2]: - 1.0000120359099368e+06 x[17,2]
   + start_service[2] - start_service[17] >= -999999
 NodeArrivalVehicles[17,3]: - 1.0000408655636616e+06 x[17,3]
   + start_service[3] - start_service[17] >= -999999
 NodeArrivalVehicles[17,4]: - 1.0000242562644748e+06 x[17,4]
   + start_service[4] - start_service[17] >= -999999
 NodeArrivalVehicles[17,5]: - 1.0000166692290857e+06 x[17,5]
   + start_service[5] - start_service[17] >= -999999
 NodeArrivalVehicles[17,6]: - 1.0000805943163345e+06 x[17,6]
   + start_service[6] - start_service[17] >= -999999
 NodeArrivalVehicles[17,7]: - 999999.01 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 1.0000408655636616e+06 x[17,8]
   + start_service[8] - start_service[17] >= -999999
 NodeArrivalVehicles[17,9]: - 1.0000242562644748e+06 x[17,9]
   + start_service[9] - start_service[17] >= -999999
 NodeArrivalVehicles[17,10]: - 1.0000166692290857e+06 x[17,10]
   + start_service[10] - start_service[17] >= -999999
 NodeArrivalVehicles[17,11]: - 1.0000805943163345e+06 x[17,11]
   + start_service[11] - start_service[17] >= -999999
 NodeArrivalVehicles[17,12]: - 999999.01 x[17,12] + start_service[12]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,13]: - 1.0000408655636616e+06 x[17,13]
   + start_service[13] - start_service[17] >= -999999
 NodeArrivalVehicles[17,14]: - 1.0000242562644748e+06 x[17,14]
   + start_service[14] - start_service[17] >= -999999
 NodeArrivalVehicles[17,15]: - 1.0000166692290857e+06 x[17,15]
   + start_service[15] - start_service[17] >= -999999
 NodeArrivalVehicles[17,16]: - 1.0000805943163345e+06 x[17,16]
   + start_service[16] - start_service[17] >= -999999
 NodeArrivalVehicles[17,17]: - 999999.01 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 1.0000408655636616e+06 x[17,18]
   - start_service[17] + start_service[18] >= -999999
 NodeArrivalVehicles[17,19]: - 1.0000242562644748e+06 x[17,19]
   - start_service[17] + start_service[19] >= -999999
 NodeArrivalVehicles[17,20]: - 1.0000166692290857e+06 x[17,20]
   - start_service[17] + start_service[20] >= -999999
 NodeArrivalVehicles[17,21]: - 1.0000805943163345e+06 x[17,21]
   - start_service[17] + start_service[21] >= -999999
 NodeArrivalVehicles[17,22]: - 999999.01 x[17,22] - start_service[17]
   + start_service[22] >= -999999
 NodeArrivalVehicles[17,23]: - 1.0000120359099368e+06 x[17,23]
   - start_service[17] + start_service[23] >= -999999
 NodeArrivalVehicles[17,24]: - 1.0000120359099368e+06 x[17,24]
   - start_service[17] + start_service[24] >= -999999
 NodeArrivalVehicles[17,25]: - 1.0000120359099368e+06 x[17,25]
   - start_service[17] + start_service[25] >= -999999
 NodeArrivalVehicles[18,1]: - 1.0000538900827599e+06 x[18,1]
   + start_service[1] - start_service[18] >= -999999
 NodeArrivalVehicles[18,2]: - 1.0000538900827599e+06 x[18,2]
   + start_service[2] - start_service[18] >= -999999
 NodeArrivalVehicles[18,3]: - 999999.01 x[18,3] + start_service[3]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,4]: - 1.0000399322151314e+06 x[18,4]
   + start_service[4] - start_service[18] >= -999999
 NodeArrivalVehicles[18,5]: - 1.0000562206776621e+06 x[18,5]
   + start_service[5] - start_service[18] >= -999999
 NodeArrivalVehicles[18,6]: - 1.0000706995536003e+06 x[18,6]
   + start_service[6] - start_service[18] >= -999999
 NodeArrivalVehicles[18,7]: - 1.0000408655636616e+06 x[18,7]
   + start_service[7] - start_service[18] >= -999999
 NodeArrivalVehicles[18,8]: - 999999.01 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 1.0000399322151314e+06 x[18,9]
   + start_service[9] - start_service[18] >= -999999
 NodeArrivalVehicles[18,10]: - 1.0000562206776621e+06 x[18,10]
   + start_service[10] - start_service[18] >= -999999
 NodeArrivalVehicles[18,11]: - 1.0000706995536003e+06 x[18,11]
   + start_service[11] - start_service[18] >= -999999
 NodeArrivalVehicles[18,12]: - 1.0000408655636616e+06 x[18,12]
   + start_service[12] - start_service[18] >= -999999
 NodeArrivalVehicles[18,13]: - 999999.01 x[18,13] + start_service[13]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,14]: - 1.0000399322151314e+06 x[18,14]
   + start_service[14] - start_service[18] >= -999999
 NodeArrivalVehicles[18,15]: - 1.0000562206776621e+06 x[18,15]
   + start_service[15] - start_service[18] >= -999999
 NodeArrivalVehicles[18,16]: - 1.0000706995536003e+06 x[18,16]
   + start_service[16] - start_service[18] >= -999999
 NodeArrivalVehicles[18,17]: - 1.0000408655636616e+06 x[18,17]
   + start_service[17] - start_service[18] >= -999999
 NodeArrivalVehicles[18,18]: - 999999.01 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 1.0000399322151314e+06 x[18,19]
   - start_service[18] + start_service[19] >= -999999
 NodeArrivalVehicles[18,20]: - 1.0000562206776621e+06 x[18,20]
   - start_service[18] + start_service[20] >= -999999
 NodeArrivalVehicles[18,21]: - 1.0000706995536003e+06 x[18,21]
   - start_service[18] + start_service[21] >= -999999
 NodeArrivalVehicles[18,22]: - 1.0000408655636616e+06 x[18,22]
   - start_service[18] + start_service[22] >= -999999
 NodeArrivalVehicles[18,23]: - 1.0000538900827599e+06 x[18,23]
   - start_service[18] + start_service[23] >= -999999
 NodeArrivalVehicles[18,24]: - 1.0000538900827599e+06 x[18,24]
   - start_service[18] + start_service[24] >= -999999
 NodeArrivalVehicles[18,25]: - 1.0000538900827599e+06 x[18,25]
   - start_service[18] + start_service[25] >= -999999
 NodeArrivalVehicles[19,1]: - 1.0000315560591351e+06 x[19,1]
   + start_service[1] - start_service[19] >= -999999
 NodeArrivalVehicles[19,2]: - 1.0000315560591351e+06 x[19,2]
   + start_service[2] - start_service[19] >= -999999
 NodeArrivalVehicles[19,3]: - 1.0000399322151314e+06 x[19,3]
   + start_service[3] - start_service[19] >= -999999
 NodeArrivalVehicles[19,4]: - 999999.01 x[19,4] + start_service[4]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,5]: - 1.0000399960725273e+06 x[19,5]
   + start_service[5] - start_service[19] >= -999999
 NodeArrivalVehicles[19,6]: - 1.0000557694957714e+06 x[19,6]
   + start_service[6] - start_service[19] >= -999999
 NodeArrivalVehicles[19,7]: - 1.0000242562644748e+06 x[19,7]
   + start_service[7] - start_service[19] >= -999999
 NodeArrivalVehicles[19,8]: - 1.0000399322151314e+06 x[19,8]
   + start_service[8] - start_service[19] >= -999999
 NodeArrivalVehicles[19,9]: - 999999.01 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 1.0000399960725273e+06 x[19,10]
   + start_service[10] - start_service[19] >= -999999
 NodeArrivalVehicles[19,11]: - 1.0000557694957714e+06 x[19,11]
   + start_service[11] - start_service[19] >= -999999
 NodeArrivalVehicles[19,12]: - 1.0000242562644748e+06 x[19,12]
   + start_service[12] - start_service[19] >= -999999
 NodeArrivalVehicles[19,13]: - 1.0000399322151314e+06 x[19,13]
   + start_service[13] - start_service[19] >= -999999
 NodeArrivalVehicles[19,14]: - 999999.01 x[19,14] + start_service[14]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,15]: - 1.0000399960725273e+06 x[19,15]
   + start_service[15] - start_service[19] >= -999999
 NodeArrivalVehicles[19,16]: - 1.0000557694957714e+06 x[19,16]
   + start_service[16] - start_service[19] >= -999999
 NodeArrivalVehicles[19,17]: - 1.0000242562644748e+06 x[19,17]
   + start_service[17] - start_service[19] >= -999999
 NodeArrivalVehicles[19,18]: - 1.0000399322151314e+06 x[19,18]
   + start_service[18] - start_service[19] >= -999999
 NodeArrivalVehicles[19,19]: - 999999.01 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 1.0000399960725273e+06 x[19,20]
   - start_service[19] + start_service[20] >= -999999
 NodeArrivalVehicles[19,21]: - 1.0000557694957714e+06 x[19,21]
   - start_service[19] + start_service[21] >= -999999
 NodeArrivalVehicles[19,22]: - 1.0000242562644748e+06 x[19,22]
   - start_service[19] + start_service[22] >= -999999
 NodeArrivalVehicles[19,23]: - 1.0000315560591351e+06 x[19,23]
   - start_service[19] + start_service[23] >= -999999
 NodeArrivalVehicles[19,24]: - 1.0000315560591351e+06 x[19,24]
   - start_service[19] + start_service[24] >= -999999
 NodeArrivalVehicles[19,25]: - 1.0000315560591351e+06 x[19,25]
   - start_service[19] + start_service[25] >= -999999
 NodeArrivalVehicles[20,1]: - 1.0000086606162182e+06 x[20,1]
   + start_service[1] - start_service[20] >= -999999
 NodeArrivalVehicles[20,2]: - 1.0000086606162182e+06 x[20,2]
   + start_service[2] - start_service[20] >= -999999
 NodeArrivalVehicles[20,3]: - 1.0000562206776621e+06 x[20,3]
   + start_service[3] - start_service[20] >= -999999
 NodeArrivalVehicles[20,4]: - 1.0000399960725273e+06 x[20,4]
   + start_service[4] - start_service[20] >= -999999
 NodeArrivalVehicles[20,5]: - 999999.01 x[20,5] + start_service[5]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,6]: - 1.0000967563598768e+06 x[20,6]
   + start_service[6] - start_service[20] >= -999999
 NodeArrivalVehicles[20,7]: - 1.0000166692290857e+06 x[20,7]
   + start_service[7] - start_service[20] >= -999999
 NodeArrivalVehicles[20,8]: - 1.0000562206776621e+06 x[20,8]
   + start_service[8] - start_service[20] >= -999999
 NodeArrivalVehicles[20,9]: - 1.0000399960725273e+06 x[20,9]
   + start_service[9] - start_service[20] >= -999999
 NodeArrivalVehicles[20,10]: - 999999.01 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 1.0000967563598768e+06 x[20,11]
   + start_service[11] - start_service[20] >= -999999
 NodeArrivalVehicles[20,12]: - 1.0000166692290857e+06 x[20,12]
   + start_service[12] - start_service[20] >= -999999
 NodeArrivalVehicles[20,13]: - 1.0000562206776621e+06 x[20,13]
   + start_service[13] - start_service[20] >= -999999
 NodeArrivalVehicles[20,14]: - 1.0000399960725273e+06 x[20,14]
   + start_service[14] - start_service[20] >= -999999
 NodeArrivalVehicles[20,15]: - 999999.01 x[20,15] + start_service[15]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,16]: - 1.0000967563598768e+06 x[20,16]
   + start_service[16] - start_service[20] >= -999999
 NodeArrivalVehicles[20,17]: - 1.0000166692290857e+06 x[20,17]
   + start_service[17] - start_service[20] >= -999999
 NodeArrivalVehicles[20,18]: - 1.0000562206776621e+06 x[20,18]
   + start_service[18] - start_service[20] >= -999999
 NodeArrivalVehicles[20,19]: - 1.0000399960725273e+06 x[20,19]
   + start_service[19] - start_service[20] >= -999999
 NodeArrivalVehicles[20,20]: - 999999.01 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 1.0000967563598768e+06 x[20,21]
   - start_service[20] + start_service[21] >= -999999
 NodeArrivalVehicles[20,22]: - 1.0000166692290857e+06 x[20,22]
   - start_service[20] + start_service[22] >= -999999
 NodeArrivalVehicles[20,23]: - 1.0000086606162182e+06 x[20,23]
   - start_service[20] + start_service[23] >= -999999
 NodeArrivalVehicles[20,24]: - 1.0000086606162182e+06 x[20,24]
   - start_service[20] + start_service[24] >= -999999
 NodeArrivalVehicles[20,25]: - 1.0000086606162182e+06 x[20,25]
   - start_service[20] + start_service[25] >= -999999
 NodeArrivalVehicles[21,1]: - 1.0000880721248598e+06 x[21,1]
   + start_service[1] - start_service[21] >= -999999
 NodeArrivalVehicles[21,2]: - 1.0000880721248598e+06 x[21,2]
   + start_service[2] - start_service[21] >= -999999
 NodeArrivalVehicles[21,3]: - 1.0000706995536003e+06 x[21,3]
   + start_service[3] - start_service[21] >= -999999
 NodeArrivalVehicles[21,4]: - 1.0000557694957714e+06 x[21,4]
   + start_service[4] - start_service[21] >= -999999
 NodeArrivalVehicles[21,5]: - 1.0000967563598768e+06 x[21,5]
   + start_service[5] - start_service[21] >= -999999
 NodeArrivalVehicles[21,6]: - 999999.01 x[21,6] + start_service[6]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,7]: - 1.0000805943163345e+06 x[21,7]
   + start_service[7] - start_service[21] >= -999999
 NodeArrivalVehicles[21,8]: - 1.0000706995536003e+06 x[21,8]
   + start_service[8] - start_service[21] >= -999999
 NodeArrivalVehicles[21,9]: - 1.0000557694957714e+06 x[21,9]
   + start_service[9] - start_service[21] >= -999999
 NodeArrivalVehicles[21,10]: - 1.0000967563598768e+06 x[21,10]
   + start_service[10] - start_service[21] >= -999999
 NodeArrivalVehicles[21,11]: - 999999.01 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 1.0000805943163345e+06 x[21,12]
   + start_service[12] - start_service[21] >= -999999
 NodeArrivalVehicles[21,13]: - 1.0000706995536003e+06 x[21,13]
   + start_service[13] - start_service[21] >= -999999
 NodeArrivalVehicles[21,14]: - 1.0000557694957714e+06 x[21,14]
   + start_service[14] - start_service[21] >= -999999
 NodeArrivalVehicles[21,15]: - 1.0000967563598768e+06 x[21,15]
   + start_service[15] - start_service[21] >= -999999
 NodeArrivalVehicles[21,16]: - 999999.01 x[21,16] + start_service[16]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,17]: - 1.0000805943163345e+06 x[21,17]
   + start_service[17] - start_service[21] >= -999999
 NodeArrivalVehicles[21,18]: - 1.0000706995536003e+06 x[21,18]
   + start_service[18] - start_service[21] >= -999999
 NodeArrivalVehicles[21,19]: - 1.0000557694957714e+06 x[21,19]
   + start_service[19] - start_service[21] >= -999999
 NodeArrivalVehicles[21,20]: - 1.0000967563598768e+06 x[21,20]
   + start_service[20] - start_service[21] >= -999999
 NodeArrivalVehicles[21,21]: - 999999.01 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 1.0000805943163345e+06 x[21,22]
   - start_service[21] + start_service[22] >= -999999
 NodeArrivalVehicles[21,23]: - 1.0000880721248598e+06 x[21,23]
   - start_service[21] + start_service[23] >= -999999
 NodeArrivalVehicles[21,24]: - 1.0000880721248598e+06 x[21,24]
   - start_service[21] + start_service[24] >= -999999
 NodeArrivalVehicles[21,25]: - 1.0000880721248598e+06 x[21,25]
   - start_service[21] + start_service[25] >= -999999
 NodeArrivalVehicles[22,1]: - 1.0000120359099368e+06 x[22,1]
   + start_service[1] - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 1.0000120359099368e+06 x[22,2]
   + start_service[2] - start_service[22] >= -999999
 NodeArrivalVehicles[22,3]: - 1.0000408655636616e+06 x[22,3]
   + start_service[3] - start_service[22] >= -999999
 NodeArrivalVehicles[22,4]: - 1.0000242562644748e+06 x[22,4]
   + start_service[4] - start_service[22] >= -999999
 NodeArrivalVehicles[22,5]: - 1.0000166692290857e+06 x[22,5]
   + start_service[5] - start_service[22] >= -999999
 NodeArrivalVehicles[22,6]: - 1.0000805943163345e+06 x[22,6]
   + start_service[6] - start_service[22] >= -999999
 NodeArrivalVehicles[22,7]: - 999999.01 x[22,7] + start_service[7]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,8]: - 1.0000408655636616e+06 x[22,8]
   + start_service[8] - start_service[22] >= -999999
 NodeArrivalVehicles[22,9]: - 1.0000242562644748e+06 x[22,9]
   + start_service[9] - start_service[22] >= -999999
 NodeArrivalVehicles[22,10]: - 1.0000166692290857e+06 x[22,10]
   + start_service[10] - start_service[22] >= -999999
 NodeArrivalVehicles[22,11]: - 1.0000805943163345e+06 x[22,11]
   + start_service[11] - start_service[22] >= -999999
 NodeArrivalVehicles[22,12]: - 999999.01 x[22,12] + start_service[12]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,13]: - 1.0000408655636616e+06 x[22,13]
   + start_service[13] - start_service[22] >= -999999
 NodeArrivalVehicles[22,14]: - 1.0000242562644748e+06 x[22,14]
   + start_service[14] - start_service[22] >= -999999
 NodeArrivalVehicles[22,15]: - 1.0000166692290857e+06 x[22,15]
   + start_service[15] - start_service[22] >= -999999
 NodeArrivalVehicles[22,16]: - 1.0000805943163345e+06 x[22,16]
   + start_service[16] - start_service[22] >= -999999
 NodeArrivalVehicles[22,17]: - 999999.01 x[22,17] + start_service[17]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,18]: - 1.0000408655636616e+06 x[22,18]
   + start_service[18] - start_service[22] >= -999999
 NodeArrivalVehicles[22,19]: - 1.0000242562644748e+06 x[22,19]
   + start_service[19] - start_service[22] >= -999999
 NodeArrivalVehicles[22,20]: - 1.0000166692290857e+06 x[22,20]
   + start_service[20] - start_service[22] >= -999999
 NodeArrivalVehicles[22,21]: - 1.0000805943163345e+06 x[22,21]
   + start_service[21] - start_service[22] >= -999999
 NodeArrivalVehicles[22,22]: - 999999.01 x[22,22] >= -999999
 NodeArrivalVehicles[22,23]: - 1.0000120359099368e+06 x[22,23]
   - start_service[22] + start_service[23] >= -999999
 NodeArrivalVehicles[22,24]: - 1.0000120359099368e+06 x[22,24]
   - start_service[22] + start_service[24] >= -999999
 NodeArrivalVehicles[22,25]: - 1.0000120359099368e+06 x[22,25]
   - start_service[22] + start_service[25] >= -999999
 NodeArrivalVehicles[23,1]: - 999999.01 x[23,1] + start_service[1]
   - start_service[23] >= -999999
 NodeArrivalVehicles[23,2]: - 999999.01 x[23,2] + start_service[2]
   - start_service[23] >= -999999
 NodeArrivalVehicles[23,3]: - 1.0000538900827599e+06 x[23,3]
   + start_service[3] - start_service[23] >= -999999
 NodeArrivalVehicles[23,4]: - 1.0000315560591351e+06 x[23,4]
   + start_service[4] - start_service[23] >= -999999
 NodeArrivalVehicles[23,5]: - 1.0000086606162182e+06 x[23,5]
   + start_service[5] - start_service[23] >= -999999
 NodeArrivalVehicles[23,6]: - 1.0000880721248598e+06 x[23,6]
   + start_service[6] - start_service[23] >= -999999
 NodeArrivalVehicles[23,7]: - 1.0000120359099368e+06 x[23,7]
   + start_service[7] - start_service[23] >= -999999
 NodeArrivalVehicles[23,8]: - 1.0000538900827599e+06 x[23,8]
   + start_service[8] - start_service[23] >= -999999
 NodeArrivalVehicles[23,9]: - 1.0000315560591351e+06 x[23,9]
   + start_service[9] - start_service[23] >= -999999
 NodeArrivalVehicles[23,10]: - 1.0000086606162182e+06 x[23,10]
   + start_service[10] - start_service[23] >= -999999
 NodeArrivalVehicles[23,11]: - 1.0000880721248598e+06 x[23,11]
   + start_service[11] - start_service[23] >= -999999
 NodeArrivalVehicles[23,12]: - 1.0000120359099368e+06 x[23,12]
   + start_service[12] - start_service[23] >= -999999
 NodeArrivalVehicles[23,13]: - 1.0000538900827599e+06 x[23,13]
   + start_service[13] - start_service[23] >= -999999
 NodeArrivalVehicles[23,14]: - 1.0000315560591351e+06 x[23,14]
   + start_service[14] - start_service[23] >= -999999
 NodeArrivalVehicles[23,15]: - 1.0000086606162182e+06 x[23,15]
   + start_service[15] - start_service[23] >= -999999
 NodeArrivalVehicles[23,16]: - 1.0000880721248598e+06 x[23,16]
   + start_service[16] - start_service[23] >= -999999
 NodeArrivalVehicles[23,17]: - 1.0000120359099368e+06 x[23,17]
   + start_service[17] - start_service[23] >= -999999
 NodeArrivalVehicles[23,18]: - 1.0000538900827599e+06 x[23,18]
   + start_service[18] - start_service[23] >= -999999
 NodeArrivalVehicles[23,19]: - 1.0000315560591351e+06 x[23,19]
   + start_service[19] - start_service[23] >= -999999
 NodeArrivalVehicles[23,20]: - 1.0000086606162182e+06 x[23,20]
   + start_service[20] - start_service[23] >= -999999
 NodeArrivalVehicles[23,21]: - 1.0000880721248598e+06 x[23,21]
   + start_service[21] - start_service[23] >= -999999
 NodeArrivalVehicles[23,22]: - 1.0000120359099368e+06 x[23,22]
   + start_service[22] - start_service[23] >= -999999
 NodeArrivalVehicles[23,23]: - 999999.01 x[23,23] >= -999999
 NodeArrivalVehicles[23,24]: - 999999.01 x[23,24] - start_service[23]
   + start_service[24] >= -999999
 NodeArrivalVehicles[23,25]: - 999999.01 x[23,25] - start_service[23]
   + start_service[25] >= -999999
 NodeArrivalVehicles[24,1]: - 999999.01 x[24,1] + start_service[1]
   - start_service[24] >= -999999
 NodeArrivalVehicles[24,2]: - 999999.01 x[24,2] + start_service[2]
   - start_service[24] >= -999999
 NodeArrivalVehicles[24,3]: - 1.0000538900827599e+06 x[24,3]
   + start_service[3] - start_service[24] >= -999999
 NodeArrivalVehicles[24,4]: - 1.0000315560591351e+06 x[24,4]
   + start_service[4] - start_service[24] >= -999999
 NodeArrivalVehicles[24,5]: - 1.0000086606162182e+06 x[24,5]
   + start_service[5] - start_service[24] >= -999999
 NodeArrivalVehicles[24,6]: - 1.0000880721248598e+06 x[24,6]
   + start_service[6] - start_service[24] >= -999999
 NodeArrivalVehicles[24,7]: - 1.0000120359099368e+06 x[24,7]
   + start_service[7] - start_service[24] >= -999999
 NodeArrivalVehicles[24,8]: - 1.0000538900827599e+06 x[24,8]
   + start_service[8] - start_service[24] >= -999999
 NodeArrivalVehicles[24,9]: - 1.0000315560591351e+06 x[24,9]
   + start_service[9] - start_service[24] >= -999999
 NodeArrivalVehicles[24,10]: - 1.0000086606162182e+06 x[24,10]
   + start_service[10] - start_service[24] >= -999999
 NodeArrivalVehicles[24,11]: - 1.0000880721248598e+06 x[24,11]
   + start_service[11] - start_service[24] >= -999999
 NodeArrivalVehicles[24,12]: - 1.0000120359099368e+06 x[24,12]
   + start_service[12] - start_service[24] >= -999999
 NodeArrivalVehicles[24,13]: - 1.0000538900827599e+06 x[24,13]
   + start_service[13] - start_service[24] >= -999999
 NodeArrivalVehicles[24,14]: - 1.0000315560591351e+06 x[24,14]
   + start_service[14] - start_service[24] >= -999999
 NodeArrivalVehicles[24,15]: - 1.0000086606162182e+06 x[24,15]
   + start_service[15] - start_service[24] >= -999999
 NodeArrivalVehicles[24,16]: - 1.0000880721248598e+06 x[24,16]
   + start_service[16] - start_service[24] >= -999999
 NodeArrivalVehicles[24,17]: - 1.0000120359099368e+06 x[24,17]
   + start_service[17] - start_service[24] >= -999999
 NodeArrivalVehicles[24,18]: - 1.0000538900827599e+06 x[24,18]
   + start_service[18] - start_service[24] >= -999999
 NodeArrivalVehicles[24,19]: - 1.0000315560591351e+06 x[24,19]
   + start_service[19] - start_service[24] >= -999999
 NodeArrivalVehicles[24,20]: - 1.0000086606162182e+06 x[24,20]
   + start_service[20] - start_service[24] >= -999999
 NodeArrivalVehicles[24,21]: - 1.0000880721248598e+06 x[24,21]
   + start_service[21] - start_service[24] >= -999999
 NodeArrivalVehicles[24,22]: - 1.0000120359099368e+06 x[24,22]
   + start_service[22] - start_service[24] >= -999999
 NodeArrivalVehicles[24,23]: - 999999.01 x[24,23] + start_service[23]
   - start_service[24] >= -999999
 NodeArrivalVehicles[24,24]: - 999999.01 x[24,24] >= -999999
 NodeArrivalVehicles[24,25]: - 999999.01 x[24,25] - start_service[24]
   + start_service[25] >= -999999
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[0,14] + w[0,15] + w[0,16] + w[0,17] + w[0,18] + w[0,19]
   + w[0,20] + w[0,21] + w[0,22] + w[0,23] + w[0,24] + w[0,25] <= 4
 TotalRobotStarts: w[0,3] + w[0,4] + w[0,5] + w[0,6] + w[0,7] + w[0,8]
   + w[0,9] + w[0,10] + w[0,11] + w[0,12] + w[0,13] + w[0,14] + w[0,15]
   + w[0,16] + w[0,17] + w[0,18] + w[0,19] + w[0,20] + w[0,21] + w[0,22]
   + w[0,23] + w[0,24] + w[0,25] + w[1,3] + w[1,4] + w[1,5] + w[1,6]
   + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12] + w[1,13]
   + w[1,14] + w[1,15] + w[1,16] + w[1,17] + w[1,18] + w[1,19] + w[1,20]
   + w[1,21] + w[1,22] + w[1,23] + w[1,24] + w[1,25] + w[2,3] + w[2,4]
   + w[2,5] + w[2,6] + w[2,7] + w[2,8] + w[2,9] + w[2,10] + w[2,11]
   + w[2,12] + w[2,13] + w[2,14] + w[2,15] + w[2,16] + w[2,17] + w[2,18]
   + w[2,19] + w[2,20] + w[2,21] + w[2,22] + w[2,23] + w[2,24] + w[2,25]
   <= 7
 RobotsStartAtRobotDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[0,14] + w[0,15] + w[0,16] + w[0,17] + w[0,18] + w[0,19]
   + w[0,20] + w[0,21] + w[0,22] + w[0,23] + w[0,24] + w[0,25] = 4
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] + w[1,14] + w[1,15] + w[1,16] + w[1,17] + w[1,18] + w[1,19]
   + w[1,20] + w[1,21] + w[1,22] + w[1,23] + w[1,24] + w[1,25] + w[2,1]
   + w[2,2] + w[2,3] + w[2,4] + w[2,5] + w[2,6] + w[2,7] + w[2,8] + w[2,9]
   + w[2,10] + w[2,11] + w[2,12] + w[2,13] + w[2,14] + w[2,15] + w[2,16]
   + w[2,17] + w[2,18] + w[2,19] + w[2,20] + w[2,21] + w[2,22] + w[2,23]
   + w[2,24] + w[2,25] = 3
 RobotsFinishAtVehicleDepot: w[0,25] + w[1,25] + w[2,25] + w[3,25]
   + w[4,25] + w[5,25] + w[6,25] + w[7,25] + w[8,25] + w[9,25] + w[10,25]
   + w[11,25] + w[12,25] + w[13,25] + w[14,25] + w[15,25] + w[16,25]
   + w[17,25] + w[18,25] + w[19,25] + w[20,25] + w[21,25] + w[22,25]
   + w[23,25] + w[24,25] <= 4
 TotalRobotFinishes: w[0,23] + w[0,24] + w[0,25] + w[1,23] + w[1,24]
   + w[1,25] + w[2,23] + w[2,24] + w[2,25] + w[3,23] + w[3,24] + w[3,25]
   + w[4,23] + w[4,24] + w[4,25] + w[5,23] + w[5,24] + w[5,25] + w[6,23]
   + w[6,24] + w[6,25] + w[7,23] + w[7,24] + w[7,25] + w[8,23] + w[8,24]
   + w[8,25] + w[9,23] + w[9,24] + w[9,25] + w[10,23] + w[10,24] + w[10,25]
   + w[11,23] + w[11,24] + w[11,25] + w[12,23] + w[12,24] + w[12,25]
   + w[13,23] + w[13,24] + w[13,25] + w[14,23] + w[14,24] + w[14,25]
   + w[15,23] + w[15,24] + w[15,25] + w[16,23] + w[16,24] + w[16,25]
   + w[17,23] + w[17,24] + w[17,25] + w[18,23] + w[18,24] + w[18,25]
   + w[19,23] + w[19,24] + w[19,25] + w[20,23] + w[20,24] + w[20,25]
   + w[21,23] + w[21,24] + w[21,25] + w[22,23] + w[22,24] + w[22,25] <= 7
 FlowConservationRobots[3]: w[0,3] + w[1,3] + w[2,3] - w[3,1] - w[3,2]
   - w[3,4] - w[3,5] - w[3,6] - w[3,7] - w[3,8] - w[3,9] - w[3,10]
   - w[3,11] - w[3,12] - w[3,13] - w[3,14] - w[3,15] - w[3,16] - w[3,17]
   - w[3,18] - w[3,19] - w[3,20] - w[3,21] - w[3,22] - w[3,23] - w[3,24]
   - w[3,25] + w[4,3] + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3]
   + w[10,3] + w[11,3] + w[12,3] + w[13,3] + w[14,3] + w[15,3] + w[16,3]
   + w[17,3] + w[18,3] + w[19,3] + w[20,3] + w[21,3] + w[22,3] + w[23,3]
   + w[24,3] = 0
 FlowConservationRobots[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] - w[4,1]
   - w[4,2] - w[4,3] - w[4,5] - w[4,6] - w[4,7] - w[4,8] - w[4,9] - w[4,10]
   - w[4,11] - w[4,12] - w[4,13] - w[4,14] - w[4,15] - w[4,16] - w[4,17]
   - w[4,18] - w[4,19] - w[4,20] - w[4,21] - w[4,22] - w[4,23] - w[4,24]
   - w[4,25] + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4]
   + w[11,4] + w[12,4] + w[13,4] + w[14,4] + w[15,4] + w[16,4] + w[17,4]
   + w[18,4] + w[19,4] + w[20,4] + w[21,4] + w[22,4] + w[23,4] + w[24,4]
   = 0
 FlowConservationRobots[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   - w[5,1] - w[5,2] - w[5,3] - w[5,4] - w[5,6] - w[5,7] - w[5,8] - w[5,9]
   - w[5,10] - w[5,11] - w[5,12] - w[5,13] - w[5,14] - w[5,15] - w[5,16]
   - w[5,17] - w[5,18] - w[5,19] - w[5,20] - w[5,21] - w[5,22] - w[5,23]
   - w[5,24] - w[5,25] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5]
   + w[11,5] + w[12,5] + w[13,5] + w[14,5] + w[15,5] + w[16,5] + w[17,5]
   + w[18,5] + w[19,5] + w[20,5] + w[21,5] + w[22,5] + w[23,5] + w[24,5]
   = 0
 FlowConservationRobots[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] - w[6,1] - w[6,2] - w[6,3] - w[6,4] - w[6,5] - w[6,7] - w[6,8]
   - w[6,9] - w[6,10] - w[6,11] - w[6,12] - w[6,13] - w[6,14] - w[6,15]
   - w[6,16] - w[6,17] - w[6,18] - w[6,19] - w[6,20] - w[6,21] - w[6,22]
   - w[6,23] - w[6,24] - w[6,25] + w[7,6] + w[8,6] + w[9,6] + w[10,6]
   + w[11,6] + w[12,6] + w[13,6] + w[14,6] + w[15,6] + w[16,6] + w[17,6]
   + w[18,6] + w[19,6] + w[20,6] + w[21,6] + w[22,6] + w[23,6] + w[24,6]
   = 0
 FlowConservationRobots[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] - w[7,1] - w[7,2] - w[7,3] - w[7,4] - w[7,5] - w[7,6]
   - w[7,8] - w[7,9] - w[7,10] - w[7,11] - w[7,12] - w[7,13] - w[7,14]
   - w[7,15] - w[7,16] - w[7,17] - w[7,18] - w[7,19] - w[7,20] - w[7,21]
   - w[7,22] - w[7,23] - w[7,24] - w[7,25] + w[8,7] + w[9,7] + w[10,7]
   + w[11,7] + w[12,7] + w[13,7] + w[14,7] + w[15,7] + w[16,7] + w[17,7]
   + w[18,7] + w[19,7] + w[20,7] + w[21,7] + w[22,7] + w[23,7] + w[24,7]
   = 0
 FlowConservationRobots[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] - w[8,1] - w[8,2] - w[8,3] - w[8,4] - w[8,5]
   - w[8,6] - w[8,7] - w[8,9] - w[8,10] - w[8,11] - w[8,12] - w[8,13]
   - w[8,14] - w[8,15] - w[8,16] - w[8,17] - w[8,18] - w[8,19] - w[8,20]
   - w[8,21] - w[8,22] - w[8,23] - w[8,24] - w[8,25] + w[9,8] + w[10,8]
   + w[11,8] + w[12,8] + w[13,8] + w[14,8] + w[15,8] + w[16,8] + w[17,8]
   + w[18,8] + w[19,8] + w[20,8] + w[21,8] + w[22,8] + w[23,8] + w[24,8]
   = 0
 FlowConservationRobots[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] - w[9,1] - w[9,2] - w[9,3] - w[9,4]
   - w[9,5] - w[9,6] - w[9,7] - w[9,8] - w[9,10] - w[9,11] - w[9,12]
   - w[9,13] - w[9,14] - w[9,15] - w[9,16] - w[9,17] - w[9,18] - w[9,19]
   - w[9,20] - w[9,21] - w[9,22] - w[9,23] - w[9,24] - w[9,25] + w[10,9]
   + w[11,9] + w[12,9] + w[13,9] + w[14,9] + w[15,9] + w[16,9] + w[17,9]
   + w[18,9] + w[19,9] + w[20,9] + w[21,9] + w[22,9] + w[23,9] + w[24,9]
   = 0
 FlowConservationRobots[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] - w[10,1]
   - w[10,2] - w[10,3] - w[10,4] - w[10,5] - w[10,6] - w[10,7] - w[10,8]
   - w[10,9] - w[10,11] - w[10,12] - w[10,13] - w[10,14] - w[10,15]
   - w[10,16] - w[10,17] - w[10,18] - w[10,19] - w[10,20] - w[10,21]
   - w[10,22] - w[10,23] - w[10,24] - w[10,25] + w[11,10] + w[12,10]
   + w[13,10] + w[14,10] + w[15,10] + w[16,10] + w[17,10] + w[18,10]
   + w[19,10] + w[20,10] + w[21,10] + w[22,10] + w[23,10] + w[24,10] = 0
 FlowConservationRobots[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   - w[11,1] - w[11,2] - w[11,3] - w[11,4] - w[11,5] - w[11,6] - w[11,7]
   - w[11,8] - w[11,9] - w[11,10] - w[11,12] - w[11,13] - w[11,14]
   - w[11,15] - w[11,16] - w[11,17] - w[11,18] - w[11,19] - w[11,20]
   - w[11,21] - w[11,22] - w[11,23] - w[11,24] - w[11,25] + w[12,11]
   + w[13,11] + w[14,11] + w[15,11] + w[16,11] + w[17,11] + w[18,11]
   + w[19,11] + w[20,11] + w[21,11] + w[22,11] + w[23,11] + w[24,11] = 0
 FlowConservationRobots[12]: w[0,12] + w[1,12] + w[2,12] + w[3,12]
   + w[4,12] + w[5,12] + w[6,12] + w[7,12] + w[8,12] + w[9,12] + w[10,12]
   + w[11,12] - w[12,1] - w[12,2] - w[12,3] - w[12,4] - w[12,5] - w[12,6]
   - w[12,7] - w[12,8] - w[12,9] - w[12,10] - w[12,11] - w[12,13]
   - w[12,14] - w[12,15] - w[12,16] - w[12,17] - w[12,18] - w[12,19]
   - w[12,20] - w[12,21] - w[12,22] - w[12,23] - w[12,24] - w[12,25]
   + w[13,12] + w[14,12] + w[15,12] + w[16,12] + w[17,12] + w[18,12]
   + w[19,12] + w[20,12] + w[21,12] + w[22,12] + w[23,12] + w[24,12] = 0
 FlowConservationRobots[13]: w[0,13] + w[1,13] + w[2,13] + w[3,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] - w[13,1] - w[13,2] - w[13,3] - w[13,4] - w[13,5]
   - w[13,6] - w[13,7] - w[13,8] - w[13,9] - w[13,10] - w[13,11] - w[13,12]
   - w[13,14] - w[13,15] - w[13,16] - w[13,17] - w[13,18] - w[13,19]
   - w[13,20] - w[13,21] - w[13,22] - w[13,23] - w[13,24] - w[13,25]
   + w[14,13] + w[15,13] + w[16,13] + w[17,13] + w[18,13] + w[19,13]
   + w[20,13] + w[21,13] + w[22,13] + w[23,13] + w[24,13] = 0
 FlowConservationRobots[14]: w[0,14] + w[1,14] + w[2,14] + w[3,14]
   + w[4,14] + w[5,14] + w[6,14] + w[7,14] + w[8,14] + w[9,14] + w[10,14]
   + w[11,14] + w[12,14] + w[13,14] - w[14,1] - w[14,2] - w[14,3] - w[14,4]
   - w[14,5] - w[14,6] - w[14,7] - w[14,8] - w[14,9] - w[14,10] - w[14,11]
   - w[14,12] - w[14,13] - w[14,15] - w[14,16] - w[14,17] - w[14,18]
   - w[14,19] - w[14,20] - w[14,21] - w[14,22] - w[14,23] - w[14,24]
   - w[14,25] + w[15,14] + w[16,14] + w[17,14] + w[18,14] + w[19,14]
   + w[20,14] + w[21,14] + w[22,14] + w[23,14] + w[24,14] = 0
 FlowConservationRobots[15]: w[0,15] + w[1,15] + w[2,15] + w[3,15]
   + w[4,15] + w[5,15] + w[6,15] + w[7,15] + w[8,15] + w[9,15] + w[10,15]
   + w[11,15] + w[12,15] + w[13,15] + w[14,15] - w[15,1] - w[15,2]
   - w[15,3] - w[15,4] - w[15,5] - w[15,6] - w[15,7] - w[15,8] - w[15,9]
   - w[15,10] - w[15,11] - w[15,12] - w[15,13] - w[15,14] - w[15,16]
   - w[15,17] - w[15,18] - w[15,19] - w[15,20] - w[15,21] - w[15,22]
   - w[15,23] - w[15,24] - w[15,25] + w[16,15] + w[17,15] + w[18,15]
   + w[19,15] + w[20,15] + w[21,15] + w[22,15] + w[23,15] + w[24,15] = 0
 FlowConservationRobots[16]: w[0,16] + w[1,16] + w[2,16] + w[3,16]
   + w[4,16] + w[5,16] + w[6,16] + w[7,16] + w[8,16] + w[9,16] + w[10,16]
   + w[11,16] + w[12,16] + w[13,16] + w[14,16] + w[15,16] - w[16,1]
   - w[16,2] - w[16,3] - w[16,4] - w[16,5] - w[16,6] - w[16,7] - w[16,8]
   - w[16,9] - w[16,10] - w[16,11] - w[16,12] - w[16,13] - w[16,14]
   - w[16,15] - w[16,17] - w[16,18] - w[16,19] - w[16,20] - w[16,21]
   - w[16,22] - w[16,23] - w[16,24] - w[16,25] + w[17,16] + w[18,16]
   + w[19,16] + w[20,16] + w[21,16] + w[22,16] + w[23,16] + w[24,16] = 0
 FlowConservationRobots[17]: w[0,17] + w[1,17] + w[2,17] + w[3,17]
   + w[4,17] + w[5,17] + w[6,17] + w[7,17] + w[8,17] + w[9,17] + w[10,17]
   + w[11,17] + w[12,17] + w[13,17] + w[14,17] + w[15,17] + w[16,17]
   - w[17,1] - w[17,2] - w[17,3] - w[17,4] - w[17,5] - w[17,6] - w[17,7]
   - w[17,8] - w[17,9] - w[17,10] - w[17,11] - w[17,12] - w[17,13]
   - w[17,14] - w[17,15] - w[17,16] - w[17,18] - w[17,19] - w[17,20]
   - w[17,21] - w[17,22] - w[17,23] - w[17,24] - w[17,25] + w[18,17]
   + w[19,17] + w[20,17] + w[21,17] + w[22,17] + w[23,17] + w[24,17] = 0
 FlowConservationRobots[18]: w[0,18] + w[1,18] + w[2,18] + w[3,18]
   + w[4,18] + w[5,18] + w[6,18] + w[7,18] + w[8,18] + w[9,18] + w[10,18]
   + w[11,18] + w[12,18] + w[13,18] + w[14,18] + w[15,18] + w[16,18]
   + w[17,18] - w[18,1] - w[18,2] - w[18,3] - w[18,4] - w[18,5] - w[18,6]
   - w[18,7] - w[18,8] - w[18,9] - w[18,10] - w[18,11] - w[18,12]
   - w[18,13] - w[18,14] - w[18,15] - w[18,16] - w[18,17] - w[18,19]
   - w[18,20] - w[18,21] - w[18,22] - w[18,23] - w[18,24] - w[18,25]
   + w[19,18] + w[20,18] + w[21,18] + w[22,18] + w[23,18] + w[24,18] = 0
 FlowConservationRobots[19]: w[0,19] + w[1,19] + w[2,19] + w[3,19]
   + w[4,19] + w[5,19] + w[6,19] + w[7,19] + w[8,19] + w[9,19] + w[10,19]
   + w[11,19] + w[12,19] + w[13,19] + w[14,19] + w[15,19] + w[16,19]
   + w[17,19] + w[18,19] - w[19,1] - w[19,2] - w[19,3] - w[19,4] - w[19,5]
   - w[19,6] - w[19,7] - w[19,8] - w[19,9] - w[19,10] - w[19,11] - w[19,12]
   - w[19,13] - w[19,14] - w[19,15] - w[19,16] - w[19,17] - w[19,18]
   - w[19,20] - w[19,21] - w[19,22] - w[19,23] - w[19,24] - w[19,25]
   + w[20,19] + w[21,19] + w[22,19] + w[23,19] + w[24,19] = 0
 FlowConservationRobots[20]: w[0,20] + w[1,20] + w[2,20] + w[3,20]
   + w[4,20] + w[5,20] + w[6,20] + w[7,20] + w[8,20] + w[9,20] + w[10,20]
   + w[11,20] + w[12,20] + w[13,20] + w[14,20] + w[15,20] + w[16,20]
   + w[17,20] + w[18,20] + w[19,20] - w[20,1] - w[20,2] - w[20,3] - w[20,4]
   - w[20,5] - w[20,6] - w[20,7] - w[20,8] - w[20,9] - w[20,10] - w[20,11]
   - w[20,12] - w[20,13] - w[20,14] - w[20,15] - w[20,16] - w[20,17]
   - w[20,18] - w[20,19] - w[20,21] - w[20,22] - w[20,23] - w[20,24]
   - w[20,25] + w[21,20] + w[22,20] + w[23,20] + w[24,20] = 0
 FlowConservationRobots[21]: w[0,21] + w[1,21] + w[2,21] + w[3,21]
   + w[4,21] + w[5,21] + w[6,21] + w[7,21] + w[8,21] + w[9,21] + w[10,21]
   + w[11,21] + w[12,21] + w[13,21] + w[14,21] + w[15,21] + w[16,21]
   + w[17,21] + w[18,21] + w[19,21] + w[20,21] - w[21,1] - w[21,2]
   - w[21,3] - w[21,4] - w[21,5] - w[21,6] - w[21,7] - w[21,8] - w[21,9]
   - w[21,10] - w[21,11] - w[21,12] - w[21,13] - w[21,14] - w[21,15]
   - w[21,16] - w[21,17] - w[21,18] - w[21,19] - w[21,20] - w[21,22]
   - w[21,23] - w[21,24] - w[21,25] + w[22,21] + w[23,21] + w[24,21] = 0
 FlowConservationRobots[22]: w[0,22] + w[1,22] + w[2,22] + w[3,22]
   + w[4,22] + w[5,22] + w[6,22] + w[7,22] + w[8,22] + w[9,22] + w[10,22]
   + w[11,22] + w[12,22] + w[13,22] + w[14,22] + w[15,22] + w[16,22]
   + w[17,22] + w[18,22] + w[19,22] + w[20,22] + w[21,22] - w[22,1]
   - w[22,2] - w[22,3] - w[22,4] - w[22,5] - w[22,6] - w[22,7] - w[22,8]
   - w[22,9] - w[22,10] - w[22,11] - w[22,12] - w[22,13] - w[22,14]
   - w[22,15] - w[22,16] - w[22,17] - w[22,18] - w[22,19] - w[22,20]
   - w[22,21] - w[22,23] - w[22,24] - w[22,25] + w[23,22] + w[24,22] = 0
 MaxNumberRobotsOnArc[0,1]: w[0,1] - 2 v[0,1] <= 0
 MaxNumberRobotsOnArc[0,2]: w[0,2] - 2 v[0,2] <= 0
 MaxNumberRobotsOnArc[0,3]: w[0,3] - 2 v[0,3] <= 0
 MaxNumberRobotsOnArc[0,4]: w[0,4] - 2 v[0,4] <= 0
 MaxNumberRobotsOnArc[0,5]: w[0,5] - 2 v[0,5] <= 0
 MaxNumberRobotsOnArc[0,6]: w[0,6] - 2 v[0,6] <= 0
 MaxNumberRobotsOnArc[0,7]: w[0,7] - 2 v[0,7] <= 0
 MaxNumberRobotsOnArc[0,8]: w[0,8] - 2 v[0,8] <= 0
 MaxNumberRobotsOnArc[0,9]: w[0,9] - 2 v[0,9] <= 0
 MaxNumberRobotsOnArc[0,10]: w[0,10] - 2 v[0,10] <= 0
 MaxNumberRobotsOnArc[0,11]: w[0,11] - 2 v[0,11] <= 0
 MaxNumberRobotsOnArc[0,12]: w[0,12] - 2 v[0,12] <= 0
 MaxNumberRobotsOnArc[0,13]: w[0,13] - 2 v[0,13] <= 0
 MaxNumberRobotsOnArc[0,14]: w[0,14] - 2 v[0,14] <= 0
 MaxNumberRobotsOnArc[0,15]: w[0,15] - 2 v[0,15] <= 0
 MaxNumberRobotsOnArc[0,16]: w[0,16] - 2 v[0,16] <= 0
 MaxNumberRobotsOnArc[0,17]: w[0,17] - 2 v[0,17] <= 0
 MaxNumberRobotsOnArc[0,18]: w[0,18] - 2 v[0,18] <= 0
 MaxNumberRobotsOnArc[0,19]: w[0,19] - 2 v[0,19] <= 0
 MaxNumberRobotsOnArc[0,20]: w[0,20] - 2 v[0,20] <= 0
 MaxNumberRobotsOnArc[0,21]: w[0,21] - 2 v[0,21] <= 0
 MaxNumberRobotsOnArc[0,22]: w[0,22] - 2 v[0,22] <= 0
 MaxNumberRobotsOnArc[0,23]: w[0,23] - 2 v[0,23] <= 0
 MaxNumberRobotsOnArc[0,24]: w[0,24] - 2 v[0,24] <= 0
 MaxNumberRobotsOnArc[0,25]: w[0,25] - 2 v[0,25] <= 0
 MaxNumberRobotsOnArc[1,1]: w[1,1] - 2 v[1,1] <= 0
 MaxNumberRobotsOnArc[1,2]: w[1,2] - 2 v[1,2] <= 0
 MaxNumberRobotsOnArc[1,3]: w[1,3] - 2 v[1,3] <= 0
 MaxNumberRobotsOnArc[1,4]: w[1,4] - 2 v[1,4] <= 0
 MaxNumberRobotsOnArc[1,5]: w[1,5] - 2 v[1,5] <= 0
 MaxNumberRobotsOnArc[1,6]: w[1,6] - 2 v[1,6] <= 0
 MaxNumberRobotsOnArc[1,7]: w[1,7] - 2 v[1,7] <= 0
 MaxNumberRobotsOnArc[1,8]: w[1,8] - 2 v[1,8] <= 0
 MaxNumberRobotsOnArc[1,9]: w[1,9] - 2 v[1,9] <= 0
 MaxNumberRobotsOnArc[1,10]: w[1,10] - 2 v[1,10] <= 0
 MaxNumberRobotsOnArc[1,11]: w[1,11] - 2 v[1,11] <= 0
 MaxNumberRobotsOnArc[1,12]: w[1,12] - 2 v[1,12] <= 0
 MaxNumberRobotsOnArc[1,13]: w[1,13] - 2 v[1,13] <= 0
 MaxNumberRobotsOnArc[1,14]: w[1,14] - 2 v[1,14] <= 0
 MaxNumberRobotsOnArc[1,15]: w[1,15] - 2 v[1,15] <= 0
 MaxNumberRobotsOnArc[1,16]: w[1,16] - 2 v[1,16] <= 0
 MaxNumberRobotsOnArc[1,17]: w[1,17] - 2 v[1,17] <= 0
 MaxNumberRobotsOnArc[1,18]: w[1,18] - 2 v[1,18] <= 0
 MaxNumberRobotsOnArc[1,19]: w[1,19] - 2 v[1,19] <= 0
 MaxNumberRobotsOnArc[1,20]: w[1,20] - 2 v[1,20] <= 0
 MaxNumberRobotsOnArc[1,21]: w[1,21] - 2 v[1,21] <= 0
 MaxNumberRobotsOnArc[1,22]: w[1,22] - 2 v[1,22] <= 0
 MaxNumberRobotsOnArc[1,23]: w[1,23] - 2 v[1,23] <= 0
 MaxNumberRobotsOnArc[1,24]: w[1,24] - 2 v[1,24] <= 0
 MaxNumberRobotsOnArc[1,25]: w[1,25] - 2 v[1,25] <= 0
 MaxNumberRobotsOnArc[2,1]: w[2,1] - 2 v[2,1] <= 0
 MaxNumberRobotsOnArc[2,2]: w[2,2] - 2 v[2,2] <= 0
 MaxNumberRobotsOnArc[2,3]: w[2,3] - 2 v[2,3] <= 0
 MaxNumberRobotsOnArc[2,4]: w[2,4] - 2 v[2,4] <= 0
 MaxNumberRobotsOnArc[2,5]: w[2,5] - 2 v[2,5] <= 0
 MaxNumberRobotsOnArc[2,6]: w[2,6] - 2 v[2,6] <= 0
 MaxNumberRobotsOnArc[2,7]: w[2,7] - 2 v[2,7] <= 0
 MaxNumberRobotsOnArc[2,8]: w[2,8] - 2 v[2,8] <= 0
 MaxNumberRobotsOnArc[2,9]: w[2,9] - 2 v[2,9] <= 0
 MaxNumberRobotsOnArc[2,10]: w[2,10] - 2 v[2,10] <= 0
 MaxNumberRobotsOnArc[2,11]: w[2,11] - 2 v[2,11] <= 0
 MaxNumberRobotsOnArc[2,12]: w[2,12] - 2 v[2,12] <= 0
 MaxNumberRobotsOnArc[2,13]: w[2,13] - 2 v[2,13] <= 0
 MaxNumberRobotsOnArc[2,14]: w[2,14] - 2 v[2,14] <= 0
 MaxNumberRobotsOnArc[2,15]: w[2,15] - 2 v[2,15] <= 0
 MaxNumberRobotsOnArc[2,16]: w[2,16] - 2 v[2,16] <= 0
 MaxNumberRobotsOnArc[2,17]: w[2,17] - 2 v[2,17] <= 0
 MaxNumberRobotsOnArc[2,18]: w[2,18] - 2 v[2,18] <= 0
 MaxNumberRobotsOnArc[2,19]: w[2,19] - 2 v[2,19] <= 0
 MaxNumberRobotsOnArc[2,20]: w[2,20] - 2 v[2,20] <= 0
 MaxNumberRobotsOnArc[2,21]: w[2,21] - 2 v[2,21] <= 0
 MaxNumberRobotsOnArc[2,22]: w[2,22] - 2 v[2,22] <= 0
 MaxNumberRobotsOnArc[2,23]: w[2,23] - 2 v[2,23] <= 0
 MaxNumberRobotsOnArc[2,24]: w[2,24] - 2 v[2,24] <= 0
 MaxNumberRobotsOnArc[2,25]: w[2,25] - 2 v[2,25] <= 0
 MaxNumberRobotsOnArc[3,1]: w[3,1] - 2 v[3,1] <= 0
 MaxNumberRobotsOnArc[3,2]: w[3,2] - 2 v[3,2] <= 0
 MaxNumberRobotsOnArc[3,3]: w[3,3] - 2 v[3,3] <= 0
 MaxNumberRobotsOnArc[3,4]: w[3,4] - 2 v[3,4] <= 0
 MaxNumberRobotsOnArc[3,5]: w[3,5] - 2 v[3,5] <= 0
 MaxNumberRobotsOnArc[3,6]: w[3,6] - 2 v[3,6] <= 0
 MaxNumberRobotsOnArc[3,7]: w[3,7] - 2 v[3,7] <= 0
 MaxNumberRobotsOnArc[3,8]: w[3,8] - 2 v[3,8] <= 0
 MaxNumberRobotsOnArc[3,9]: w[3,9] - 2 v[3,9] <= 0
 MaxNumberRobotsOnArc[3,10]: w[3,10] - 2 v[3,10] <= 0
 MaxNumberRobotsOnArc[3,11]: w[3,11] - 2 v[3,11] <= 0
 MaxNumberRobotsOnArc[3,12]: w[3,12] - 2 v[3,12] <= 0
 MaxNumberRobotsOnArc[3,13]: w[3,13] - 2 v[3,13] <= 0
 MaxNumberRobotsOnArc[3,14]: w[3,14] - 2 v[3,14] <= 0
 MaxNumberRobotsOnArc[3,15]: w[3,15] - 2 v[3,15] <= 0
 MaxNumberRobotsOnArc[3,16]: w[3,16] - 2 v[3,16] <= 0
 MaxNumberRobotsOnArc[3,17]: w[3,17] - 2 v[3,17] <= 0
 MaxNumberRobotsOnArc[3,18]: w[3,18] - 2 v[3,18] <= 0
 MaxNumberRobotsOnArc[3,19]: w[3,19] - 2 v[3,19] <= 0
 MaxNumberRobotsOnArc[3,20]: w[3,20] - 2 v[3,20] <= 0
 MaxNumberRobotsOnArc[3,21]: w[3,21] - 2 v[3,21] <= 0
 MaxNumberRobotsOnArc[3,22]: w[3,22] - 2 v[3,22] <= 0
 MaxNumberRobotsOnArc[3,23]: w[3,23] - 2 v[3,23] <= 0
 MaxNumberRobotsOnArc[3,24]: w[3,24] - 2 v[3,24] <= 0
 MaxNumberRobotsOnArc[3,25]: w[3,25] - 2 v[3,25] <= 0
 MaxNumberRobotsOnArc[4,1]: w[4,1] - 2 v[4,1] <= 0
 MaxNumberRobotsOnArc[4,2]: w[4,2] - 2 v[4,2] <= 0
 MaxNumberRobotsOnArc[4,3]: w[4,3] - 2 v[4,3] <= 0
 MaxNumberRobotsOnArc[4,4]: w[4,4] - 2 v[4,4] <= 0
 MaxNumberRobotsOnArc[4,5]: w[4,5] - 2 v[4,5] <= 0
 MaxNumberRobotsOnArc[4,6]: w[4,6] - 2 v[4,6] <= 0
 MaxNumberRobotsOnArc[4,7]: w[4,7] - 2 v[4,7] <= 0
 MaxNumberRobotsOnArc[4,8]: w[4,8] - 2 v[4,8] <= 0
 MaxNumberRobotsOnArc[4,9]: w[4,9] - 2 v[4,9] <= 0
 MaxNumberRobotsOnArc[4,10]: w[4,10] - 2 v[4,10] <= 0
 MaxNumberRobotsOnArc[4,11]: w[4,11] - 2 v[4,11] <= 0
 MaxNumberRobotsOnArc[4,12]: w[4,12] - 2 v[4,12] <= 0
 MaxNumberRobotsOnArc[4,13]: w[4,13] - 2 v[4,13] <= 0
 MaxNumberRobotsOnArc[4,14]: w[4,14] - 2 v[4,14] <= 0
 MaxNumberRobotsOnArc[4,15]: w[4,15] - 2 v[4,15] <= 0
 MaxNumberRobotsOnArc[4,16]: w[4,16] - 2 v[4,16] <= 0
 MaxNumberRobotsOnArc[4,17]: w[4,17] - 2 v[4,17] <= 0
 MaxNumberRobotsOnArc[4,18]: w[4,18] - 2 v[4,18] <= 0
 MaxNumberRobotsOnArc[4,19]: w[4,19] - 2 v[4,19] <= 0
 MaxNumberRobotsOnArc[4,20]: w[4,20] - 2 v[4,20] <= 0
 MaxNumberRobotsOnArc[4,21]: w[4,21] - 2 v[4,21] <= 0
 MaxNumberRobotsOnArc[4,22]: w[4,22] - 2 v[4,22] <= 0
 MaxNumberRobotsOnArc[4,23]: w[4,23] - 2 v[4,23] <= 0
 MaxNumberRobotsOnArc[4,24]: w[4,24] - 2 v[4,24] <= 0
 MaxNumberRobotsOnArc[4,25]: w[4,25] - 2 v[4,25] <= 0
 MaxNumberRobotsOnArc[5,1]: w[5,1] - 2 v[5,1] <= 0
 MaxNumberRobotsOnArc[5,2]: w[5,2] - 2 v[5,2] <= 0
 MaxNumberRobotsOnArc[5,3]: w[5,3] - 2 v[5,3] <= 0
 MaxNumberRobotsOnArc[5,4]: w[5,4] - 2 v[5,4] <= 0
 MaxNumberRobotsOnArc[5,5]: w[5,5] - 2 v[5,5] <= 0
 MaxNumberRobotsOnArc[5,6]: w[5,6] - 2 v[5,6] <= 0
 MaxNumberRobotsOnArc[5,7]: w[5,7] - 2 v[5,7] <= 0
 MaxNumberRobotsOnArc[5,8]: w[5,8] - 2 v[5,8] <= 0
 MaxNumberRobotsOnArc[5,9]: w[5,9] - 2 v[5,9] <= 0
 MaxNumberRobotsOnArc[5,10]: w[5,10] - 2 v[5,10] <= 0
 MaxNumberRobotsOnArc[5,11]: w[5,11] - 2 v[5,11] <= 0
 MaxNumberRobotsOnArc[5,12]: w[5,12] - 2 v[5,12] <= 0
 MaxNumberRobotsOnArc[5,13]: w[5,13] - 2 v[5,13] <= 0
 MaxNumberRobotsOnArc[5,14]: w[5,14] - 2 v[5,14] <= 0
 MaxNumberRobotsOnArc[5,15]: w[5,15] - 2 v[5,15] <= 0
 MaxNumberRobotsOnArc[5,16]: w[5,16] - 2 v[5,16] <= 0
 MaxNumberRobotsOnArc[5,17]: w[5,17] - 2 v[5,17] <= 0
 MaxNumberRobotsOnArc[5,18]: w[5,18] - 2 v[5,18] <= 0
 MaxNumberRobotsOnArc[5,19]: w[5,19] - 2 v[5,19] <= 0
 MaxNumberRobotsOnArc[5,20]: w[5,20] - 2 v[5,20] <= 0
 MaxNumberRobotsOnArc[5,21]: w[5,21] - 2 v[5,21] <= 0
 MaxNumberRobotsOnArc[5,22]: w[5,22] - 2 v[5,22] <= 0
 MaxNumberRobotsOnArc[5,23]: w[5,23] - 2 v[5,23] <= 0
 MaxNumberRobotsOnArc[5,24]: w[5,24] - 2 v[5,24] <= 0
 MaxNumberRobotsOnArc[5,25]: w[5,25] - 2 v[5,25] <= 0
 MaxNumberRobotsOnArc[6,1]: w[6,1] - 2 v[6,1] <= 0
 MaxNumberRobotsOnArc[6,2]: w[6,2] - 2 v[6,2] <= 0
 MaxNumberRobotsOnArc[6,3]: w[6,3] - 2 v[6,3] <= 0
 MaxNumberRobotsOnArc[6,4]: w[6,4] - 2 v[6,4] <= 0
 MaxNumberRobotsOnArc[6,5]: w[6,5] - 2 v[6,5] <= 0
 MaxNumberRobotsOnArc[6,6]: w[6,6] - 2 v[6,6] <= 0
 MaxNumberRobotsOnArc[6,7]: w[6,7] - 2 v[6,7] <= 0
 MaxNumberRobotsOnArc[6,8]: w[6,8] - 2 v[6,8] <= 0
 MaxNumberRobotsOnArc[6,9]: w[6,9] - 2 v[6,9] <= 0
 MaxNumberRobotsOnArc[6,10]: w[6,10] - 2 v[6,10] <= 0
 MaxNumberRobotsOnArc[6,11]: w[6,11] - 2 v[6,11] <= 0
 MaxNumberRobotsOnArc[6,12]: w[6,12] - 2 v[6,12] <= 0
 MaxNumberRobotsOnArc[6,13]: w[6,13] - 2 v[6,13] <= 0
 MaxNumberRobotsOnArc[6,14]: w[6,14] - 2 v[6,14] <= 0
 MaxNumberRobotsOnArc[6,15]: w[6,15] - 2 v[6,15] <= 0
 MaxNumberRobotsOnArc[6,16]: w[6,16] - 2 v[6,16] <= 0
 MaxNumberRobotsOnArc[6,17]: w[6,17] - 2 v[6,17] <= 0
 MaxNumberRobotsOnArc[6,18]: w[6,18] - 2 v[6,18] <= 0
 MaxNumberRobotsOnArc[6,19]: w[6,19] - 2 v[6,19] <= 0
 MaxNumberRobotsOnArc[6,20]: w[6,20] - 2 v[6,20] <= 0
 MaxNumberRobotsOnArc[6,21]: w[6,21] - 2 v[6,21] <= 0
 MaxNumberRobotsOnArc[6,22]: w[6,22] - 2 v[6,22] <= 0
 MaxNumberRobotsOnArc[6,23]: w[6,23] - 2 v[6,23] <= 0
 MaxNumberRobotsOnArc[6,24]: w[6,24] - 2 v[6,24] <= 0
 MaxNumberRobotsOnArc[6,25]: w[6,25] - 2 v[6,25] <= 0
 MaxNumberRobotsOnArc[7,1]: w[7,1] - 2 v[7,1] <= 0
 MaxNumberRobotsOnArc[7,2]: w[7,2] - 2 v[7,2] <= 0
 MaxNumberRobotsOnArc[7,3]: w[7,3] - 2 v[7,3] <= 0
 MaxNumberRobotsOnArc[7,4]: w[7,4] - 2 v[7,4] <= 0
 MaxNumberRobotsOnArc[7,5]: w[7,5] - 2 v[7,5] <= 0
 MaxNumberRobotsOnArc[7,6]: w[7,6] - 2 v[7,6] <= 0
 MaxNumberRobotsOnArc[7,7]: w[7,7] - 2 v[7,7] <= 0
 MaxNumberRobotsOnArc[7,8]: w[7,8] - 2 v[7,8] <= 0
 MaxNumberRobotsOnArc[7,9]: w[7,9] - 2 v[7,9] <= 0
 MaxNumberRobotsOnArc[7,10]: w[7,10] - 2 v[7,10] <= 0
 MaxNumberRobotsOnArc[7,11]: w[7,11] - 2 v[7,11] <= 0
 MaxNumberRobotsOnArc[7,12]: w[7,12] - 2 v[7,12] <= 0
 MaxNumberRobotsOnArc[7,13]: w[7,13] - 2 v[7,13] <= 0
 MaxNumberRobotsOnArc[7,14]: w[7,14] - 2 v[7,14] <= 0
 MaxNumberRobotsOnArc[7,15]: w[7,15] - 2 v[7,15] <= 0
 MaxNumberRobotsOnArc[7,16]: w[7,16] - 2 v[7,16] <= 0
 MaxNumberRobotsOnArc[7,17]: w[7,17] - 2 v[7,17] <= 0
 MaxNumberRobotsOnArc[7,18]: w[7,18] - 2 v[7,18] <= 0
 MaxNumberRobotsOnArc[7,19]: w[7,19] - 2 v[7,19] <= 0
 MaxNumberRobotsOnArc[7,20]: w[7,20] - 2 v[7,20] <= 0
 MaxNumberRobotsOnArc[7,21]: w[7,21] - 2 v[7,21] <= 0
 MaxNumberRobotsOnArc[7,22]: w[7,22] - 2 v[7,22] <= 0
 MaxNumberRobotsOnArc[7,23]: w[7,23] - 2 v[7,23] <= 0
 MaxNumberRobotsOnArc[7,24]: w[7,24] - 2 v[7,24] <= 0
 MaxNumberRobotsOnArc[7,25]: w[7,25] - 2 v[7,25] <= 0
 MaxNumberRobotsOnArc[8,1]: w[8,1] - 2 v[8,1] <= 0
 MaxNumberRobotsOnArc[8,2]: w[8,2] - 2 v[8,2] <= 0
 MaxNumberRobotsOnArc[8,3]: w[8,3] - 2 v[8,3] <= 0
 MaxNumberRobotsOnArc[8,4]: w[8,4] - 2 v[8,4] <= 0
 MaxNumberRobotsOnArc[8,5]: w[8,5] - 2 v[8,5] <= 0
 MaxNumberRobotsOnArc[8,6]: w[8,6] - 2 v[8,6] <= 0
 MaxNumberRobotsOnArc[8,7]: w[8,7] - 2 v[8,7] <= 0
 MaxNumberRobotsOnArc[8,8]: w[8,8] - 2 v[8,8] <= 0
 MaxNumberRobotsOnArc[8,9]: w[8,9] - 2 v[8,9] <= 0
 MaxNumberRobotsOnArc[8,10]: w[8,10] - 2 v[8,10] <= 0
 MaxNumberRobotsOnArc[8,11]: w[8,11] - 2 v[8,11] <= 0
 MaxNumberRobotsOnArc[8,12]: w[8,12] - 2 v[8,12] <= 0
 MaxNumberRobotsOnArc[8,13]: w[8,13] - 2 v[8,13] <= 0
 MaxNumberRobotsOnArc[8,14]: w[8,14] - 2 v[8,14] <= 0
 MaxNumberRobotsOnArc[8,15]: w[8,15] - 2 v[8,15] <= 0
 MaxNumberRobotsOnArc[8,16]: w[8,16] - 2 v[8,16] <= 0
 MaxNumberRobotsOnArc[8,17]: w[8,17] - 2 v[8,17] <= 0
 MaxNumberRobotsOnArc[8,18]: w[8,18] - 2 v[8,18] <= 0
 MaxNumberRobotsOnArc[8,19]: w[8,19] - 2 v[8,19] <= 0
 MaxNumberRobotsOnArc[8,20]: w[8,20] - 2 v[8,20] <= 0
 MaxNumberRobotsOnArc[8,21]: w[8,21] - 2 v[8,21] <= 0
 MaxNumberRobotsOnArc[8,22]: w[8,22] - 2 v[8,22] <= 0
 MaxNumberRobotsOnArc[8,23]: w[8,23] - 2 v[8,23] <= 0
 MaxNumberRobotsOnArc[8,24]: w[8,24] - 2 v[8,24] <= 0
 MaxNumberRobotsOnArc[8,25]: w[8,25] - 2 v[8,25] <= 0
 MaxNumberRobotsOnArc[9,1]: w[9,1] - 2 v[9,1] <= 0
 MaxNumberRobotsOnArc[9,2]: w[9,2] - 2 v[9,2] <= 0
 MaxNumberRobotsOnArc[9,3]: w[9,3] - 2 v[9,3] <= 0
 MaxNumberRobotsOnArc[9,4]: w[9,4] - 2 v[9,4] <= 0
 MaxNumberRobotsOnArc[9,5]: w[9,5] - 2 v[9,5] <= 0
 MaxNumberRobotsOnArc[9,6]: w[9,6] - 2 v[9,6] <= 0
 MaxNumberRobotsOnArc[9,7]: w[9,7] - 2 v[9,7] <= 0
 MaxNumberRobotsOnArc[9,8]: w[9,8] - 2 v[9,8] <= 0
 MaxNumberRobotsOnArc[9,9]: w[9,9] - 2 v[9,9] <= 0
 MaxNumberRobotsOnArc[9,10]: w[9,10] - 2 v[9,10] <= 0
 MaxNumberRobotsOnArc[9,11]: w[9,11] - 2 v[9,11] <= 0
 MaxNumberRobotsOnArc[9,12]: w[9,12] - 2 v[9,12] <= 0
 MaxNumberRobotsOnArc[9,13]: w[9,13] - 2 v[9,13] <= 0
 MaxNumberRobotsOnArc[9,14]: w[9,14] - 2 v[9,14] <= 0
 MaxNumberRobotsOnArc[9,15]: w[9,15] - 2 v[9,15] <= 0
 MaxNumberRobotsOnArc[9,16]: w[9,16] - 2 v[9,16] <= 0
 MaxNumberRobotsOnArc[9,17]: w[9,17] - 2 v[9,17] <= 0
 MaxNumberRobotsOnArc[9,18]: w[9,18] - 2 v[9,18] <= 0
 MaxNumberRobotsOnArc[9,19]: w[9,19] - 2 v[9,19] <= 0
 MaxNumberRobotsOnArc[9,20]: w[9,20] - 2 v[9,20] <= 0
 MaxNumberRobotsOnArc[9,21]: w[9,21] - 2 v[9,21] <= 0
 MaxNumberRobotsOnArc[9,22]: w[9,22] - 2 v[9,22] <= 0
 MaxNumberRobotsOnArc[9,23]: w[9,23] - 2 v[9,23] <= 0
 MaxNumberRobotsOnArc[9,24]: w[9,24] - 2 v[9,24] <= 0
 MaxNumberRobotsOnArc[9,25]: w[9,25] - 2 v[9,25] <= 0
 MaxNumberRobotsOnArc[10,1]: w[10,1] - 2 v[10,1] <= 0
 MaxNumberRobotsOnArc[10,2]: w[10,2] - 2 v[10,2] <= 0
 MaxNumberRobotsOnArc[10,3]: w[10,3] - 2 v[10,3] <= 0
 MaxNumberRobotsOnArc[10,4]: w[10,4] - 2 v[10,4] <= 0
 MaxNumberRobotsOnArc[10,5]: w[10,5] - 2 v[10,5] <= 0
 MaxNumberRobotsOnArc[10,6]: w[10,6] - 2 v[10,6] <= 0
 MaxNumberRobotsOnArc[10,7]: w[10,7] - 2 v[10,7] <= 0
 MaxNumberRobotsOnArc[10,8]: w[10,8] - 2 v[10,8] <= 0
 MaxNumberRobotsOnArc[10,9]: w[10,9] - 2 v[10,9] <= 0
 MaxNumberRobotsOnArc[10,10]: w[10,10] - 2 v[10,10] <= 0
 MaxNumberRobotsOnArc[10,11]: w[10,11] - 2 v[10,11] <= 0
 MaxNumberRobotsOnArc[10,12]: w[10,12] - 2 v[10,12] <= 0
 MaxNumberRobotsOnArc[10,13]: w[10,13] - 2 v[10,13] <= 0
 MaxNumberRobotsOnArc[10,14]: w[10,14] - 2 v[10,14] <= 0
 MaxNumberRobotsOnArc[10,15]: w[10,15] - 2 v[10,15] <= 0
 MaxNumberRobotsOnArc[10,16]: w[10,16] - 2 v[10,16] <= 0
 MaxNumberRobotsOnArc[10,17]: w[10,17] - 2 v[10,17] <= 0
 MaxNumberRobotsOnArc[10,18]: w[10,18] - 2 v[10,18] <= 0
 MaxNumberRobotsOnArc[10,19]: w[10,19] - 2 v[10,19] <= 0
 MaxNumberRobotsOnArc[10,20]: w[10,20] - 2 v[10,20] <= 0
 MaxNumberRobotsOnArc[10,21]: w[10,21] - 2 v[10,21] <= 0
 MaxNumberRobotsOnArc[10,22]: w[10,22] - 2 v[10,22] <= 0
 MaxNumberRobotsOnArc[10,23]: w[10,23] - 2 v[10,23] <= 0
 MaxNumberRobotsOnArc[10,24]: w[10,24] - 2 v[10,24] <= 0
 MaxNumberRobotsOnArc[10,25]: w[10,25] - 2 v[10,25] <= 0
 MaxNumberRobotsOnArc[11,1]: w[11,1] - 2 v[11,1] <= 0
 MaxNumberRobotsOnArc[11,2]: w[11,2] - 2 v[11,2] <= 0
 MaxNumberRobotsOnArc[11,3]: w[11,3] - 2 v[11,3] <= 0
 MaxNumberRobotsOnArc[11,4]: w[11,4] - 2 v[11,4] <= 0
 MaxNumberRobotsOnArc[11,5]: w[11,5] - 2 v[11,5] <= 0
 MaxNumberRobotsOnArc[11,6]: w[11,6] - 2 v[11,6] <= 0
 MaxNumberRobotsOnArc[11,7]: w[11,7] - 2 v[11,7] <= 0
 MaxNumberRobotsOnArc[11,8]: w[11,8] - 2 v[11,8] <= 0
 MaxNumberRobotsOnArc[11,9]: w[11,9] - 2 v[11,9] <= 0
 MaxNumberRobotsOnArc[11,10]: w[11,10] - 2 v[11,10] <= 0
 MaxNumberRobotsOnArc[11,11]: w[11,11] - 2 v[11,11] <= 0
 MaxNumberRobotsOnArc[11,12]: w[11,12] - 2 v[11,12] <= 0
 MaxNumberRobotsOnArc[11,13]: w[11,13] - 2 v[11,13] <= 0
 MaxNumberRobotsOnArc[11,14]: w[11,14] - 2 v[11,14] <= 0
 MaxNumberRobotsOnArc[11,15]: w[11,15] - 2 v[11,15] <= 0
 MaxNumberRobotsOnArc[11,16]: w[11,16] - 2 v[11,16] <= 0
 MaxNumberRobotsOnArc[11,17]: w[11,17] - 2 v[11,17] <= 0
 MaxNumberRobotsOnArc[11,18]: w[11,18] - 2 v[11,18] <= 0
 MaxNumberRobotsOnArc[11,19]: w[11,19] - 2 v[11,19] <= 0
 MaxNumberRobotsOnArc[11,20]: w[11,20] - 2 v[11,20] <= 0
 MaxNumberRobotsOnArc[11,21]: w[11,21] - 2 v[11,21] <= 0
 MaxNumberRobotsOnArc[11,22]: w[11,22] - 2 v[11,22] <= 0
 MaxNumberRobotsOnArc[11,23]: w[11,23] - 2 v[11,23] <= 0
 MaxNumberRobotsOnArc[11,24]: w[11,24] - 2 v[11,24] <= 0
 MaxNumberRobotsOnArc[11,25]: w[11,25] - 2 v[11,25] <= 0
 MaxNumberRobotsOnArc[12,1]: w[12,1] - 2 v[12,1] <= 0
 MaxNumberRobotsOnArc[12,2]: w[12,2] - 2 v[12,2] <= 0
 MaxNumberRobotsOnArc[12,3]: w[12,3] - 2 v[12,3] <= 0
 MaxNumberRobotsOnArc[12,4]: w[12,4] - 2 v[12,4] <= 0
 MaxNumberRobotsOnArc[12,5]: w[12,5] - 2 v[12,5] <= 0
 MaxNumberRobotsOnArc[12,6]: w[12,6] - 2 v[12,6] <= 0
 MaxNumberRobotsOnArc[12,7]: w[12,7] - 2 v[12,7] <= 0
 MaxNumberRobotsOnArc[12,8]: w[12,8] - 2 v[12,8] <= 0
 MaxNumberRobotsOnArc[12,9]: w[12,9] - 2 v[12,9] <= 0
 MaxNumberRobotsOnArc[12,10]: w[12,10] - 2 v[12,10] <= 0
 MaxNumberRobotsOnArc[12,11]: w[12,11] - 2 v[12,11] <= 0
 MaxNumberRobotsOnArc[12,12]: w[12,12] - 2 v[12,12] <= 0
 MaxNumberRobotsOnArc[12,13]: w[12,13] - 2 v[12,13] <= 0
 MaxNumberRobotsOnArc[12,14]: w[12,14] - 2 v[12,14] <= 0
 MaxNumberRobotsOnArc[12,15]: w[12,15] - 2 v[12,15] <= 0
 MaxNumberRobotsOnArc[12,16]: w[12,16] - 2 v[12,16] <= 0
 MaxNumberRobotsOnArc[12,17]: w[12,17] - 2 v[12,17] <= 0
 MaxNumberRobotsOnArc[12,18]: w[12,18] - 2 v[12,18] <= 0
 MaxNumberRobotsOnArc[12,19]: w[12,19] - 2 v[12,19] <= 0
 MaxNumberRobotsOnArc[12,20]: w[12,20] - 2 v[12,20] <= 0
 MaxNumberRobotsOnArc[12,21]: w[12,21] - 2 v[12,21] <= 0
 MaxNumberRobotsOnArc[12,22]: w[12,22] - 2 v[12,22] <= 0
 MaxNumberRobotsOnArc[12,23]: w[12,23] - 2 v[12,23] <= 0
 MaxNumberRobotsOnArc[12,24]: w[12,24] - 2 v[12,24] <= 0
 MaxNumberRobotsOnArc[12,25]: w[12,25] - 2 v[12,25] <= 0
 MaxNumberRobotsOnArc[13,1]: w[13,1] - 2 v[13,1] <= 0
 MaxNumberRobotsOnArc[13,2]: w[13,2] - 2 v[13,2] <= 0
 MaxNumberRobotsOnArc[13,3]: w[13,3] - 2 v[13,3] <= 0
 MaxNumberRobotsOnArc[13,4]: w[13,4] - 2 v[13,4] <= 0
 MaxNumberRobotsOnArc[13,5]: w[13,5] - 2 v[13,5] <= 0
 MaxNumberRobotsOnArc[13,6]: w[13,6] - 2 v[13,6] <= 0
 MaxNumberRobotsOnArc[13,7]: w[13,7] - 2 v[13,7] <= 0
 MaxNumberRobotsOnArc[13,8]: w[13,8] - 2 v[13,8] <= 0
 MaxNumberRobotsOnArc[13,9]: w[13,9] - 2 v[13,9] <= 0
 MaxNumberRobotsOnArc[13,10]: w[13,10] - 2 v[13,10] <= 0
 MaxNumberRobotsOnArc[13,11]: w[13,11] - 2 v[13,11] <= 0
 MaxNumberRobotsOnArc[13,12]: w[13,12] - 2 v[13,12] <= 0
 MaxNumberRobotsOnArc[13,13]: w[13,13] - 2 v[13,13] <= 0
 MaxNumberRobotsOnArc[13,14]: w[13,14] - 2 v[13,14] <= 0
 MaxNumberRobotsOnArc[13,15]: w[13,15] - 2 v[13,15] <= 0
 MaxNumberRobotsOnArc[13,16]: w[13,16] - 2 v[13,16] <= 0
 MaxNumberRobotsOnArc[13,17]: w[13,17] - 2 v[13,17] <= 0
 MaxNumberRobotsOnArc[13,18]: w[13,18] - 2 v[13,18] <= 0
 MaxNumberRobotsOnArc[13,19]: w[13,19] - 2 v[13,19] <= 0
 MaxNumberRobotsOnArc[13,20]: w[13,20] - 2 v[13,20] <= 0
 MaxNumberRobotsOnArc[13,21]: w[13,21] - 2 v[13,21] <= 0
 MaxNumberRobotsOnArc[13,22]: w[13,22] - 2 v[13,22] <= 0
 MaxNumberRobotsOnArc[13,23]: w[13,23] - 2 v[13,23] <= 0
 MaxNumberRobotsOnArc[13,24]: w[13,24] - 2 v[13,24] <= 0
 MaxNumberRobotsOnArc[13,25]: w[13,25] - 2 v[13,25] <= 0
 MaxNumberRobotsOnArc[14,1]: w[14,1] - 2 v[14,1] <= 0
 MaxNumberRobotsOnArc[14,2]: w[14,2] - 2 v[14,2] <= 0
 MaxNumberRobotsOnArc[14,3]: w[14,3] - 2 v[14,3] <= 0
 MaxNumberRobotsOnArc[14,4]: w[14,4] - 2 v[14,4] <= 0
 MaxNumberRobotsOnArc[14,5]: w[14,5] - 2 v[14,5] <= 0
 MaxNumberRobotsOnArc[14,6]: w[14,6] - 2 v[14,6] <= 0
 MaxNumberRobotsOnArc[14,7]: w[14,7] - 2 v[14,7] <= 0
 MaxNumberRobotsOnArc[14,8]: w[14,8] - 2 v[14,8] <= 0
 MaxNumberRobotsOnArc[14,9]: w[14,9] - 2 v[14,9] <= 0
 MaxNumberRobotsOnArc[14,10]: w[14,10] - 2 v[14,10] <= 0
 MaxNumberRobotsOnArc[14,11]: w[14,11] - 2 v[14,11] <= 0
 MaxNumberRobotsOnArc[14,12]: w[14,12] - 2 v[14,12] <= 0
 MaxNumberRobotsOnArc[14,13]: w[14,13] - 2 v[14,13] <= 0
 MaxNumberRobotsOnArc[14,14]: w[14,14] - 2 v[14,14] <= 0
 MaxNumberRobotsOnArc[14,15]: w[14,15] - 2 v[14,15] <= 0
 MaxNumberRobotsOnArc[14,16]: w[14,16] - 2 v[14,16] <= 0
 MaxNumberRobotsOnArc[14,17]: w[14,17] - 2 v[14,17] <= 0
 MaxNumberRobotsOnArc[14,18]: w[14,18] - 2 v[14,18] <= 0
 MaxNumberRobotsOnArc[14,19]: w[14,19] - 2 v[14,19] <= 0
 MaxNumberRobotsOnArc[14,20]: w[14,20] - 2 v[14,20] <= 0
 MaxNumberRobotsOnArc[14,21]: w[14,21] - 2 v[14,21] <= 0
 MaxNumberRobotsOnArc[14,22]: w[14,22] - 2 v[14,22] <= 0
 MaxNumberRobotsOnArc[14,23]: w[14,23] - 2 v[14,23] <= 0
 MaxNumberRobotsOnArc[14,24]: w[14,24] - 2 v[14,24] <= 0
 MaxNumberRobotsOnArc[14,25]: w[14,25] - 2 v[14,25] <= 0
 MaxNumberRobotsOnArc[15,1]: w[15,1] - 2 v[15,1] <= 0
 MaxNumberRobotsOnArc[15,2]: w[15,2] - 2 v[15,2] <= 0
 MaxNumberRobotsOnArc[15,3]: w[15,3] - 2 v[15,3] <= 0
 MaxNumberRobotsOnArc[15,4]: w[15,4] - 2 v[15,4] <= 0
 MaxNumberRobotsOnArc[15,5]: w[15,5] - 2 v[15,5] <= 0
 MaxNumberRobotsOnArc[15,6]: w[15,6] - 2 v[15,6] <= 0
 MaxNumberRobotsOnArc[15,7]: w[15,7] - 2 v[15,7] <= 0
 MaxNumberRobotsOnArc[15,8]: w[15,8] - 2 v[15,8] <= 0
 MaxNumberRobotsOnArc[15,9]: w[15,9] - 2 v[15,9] <= 0
 MaxNumberRobotsOnArc[15,10]: w[15,10] - 2 v[15,10] <= 0
 MaxNumberRobotsOnArc[15,11]: w[15,11] - 2 v[15,11] <= 0
 MaxNumberRobotsOnArc[15,12]: w[15,12] - 2 v[15,12] <= 0
 MaxNumberRobotsOnArc[15,13]: w[15,13] - 2 v[15,13] <= 0
 MaxNumberRobotsOnArc[15,14]: w[15,14] - 2 v[15,14] <= 0
 MaxNumberRobotsOnArc[15,15]: w[15,15] - 2 v[15,15] <= 0
 MaxNumberRobotsOnArc[15,16]: w[15,16] - 2 v[15,16] <= 0
 MaxNumberRobotsOnArc[15,17]: w[15,17] - 2 v[15,17] <= 0
 MaxNumberRobotsOnArc[15,18]: w[15,18] - 2 v[15,18] <= 0
 MaxNumberRobotsOnArc[15,19]: w[15,19] - 2 v[15,19] <= 0
 MaxNumberRobotsOnArc[15,20]: w[15,20] - 2 v[15,20] <= 0
 MaxNumberRobotsOnArc[15,21]: w[15,21] - 2 v[15,21] <= 0
 MaxNumberRobotsOnArc[15,22]: w[15,22] - 2 v[15,22] <= 0
 MaxNumberRobotsOnArc[15,23]: w[15,23] - 2 v[15,23] <= 0
 MaxNumberRobotsOnArc[15,24]: w[15,24] - 2 v[15,24] <= 0
 MaxNumberRobotsOnArc[15,25]: w[15,25] - 2 v[15,25] <= 0
 MaxNumberRobotsOnArc[16,1]: w[16,1] - 2 v[16,1] <= 0
 MaxNumberRobotsOnArc[16,2]: w[16,2] - 2 v[16,2] <= 0
 MaxNumberRobotsOnArc[16,3]: w[16,3] - 2 v[16,3] <= 0
 MaxNumberRobotsOnArc[16,4]: w[16,4] - 2 v[16,4] <= 0
 MaxNumberRobotsOnArc[16,5]: w[16,5] - 2 v[16,5] <= 0
 MaxNumberRobotsOnArc[16,6]: w[16,6] - 2 v[16,6] <= 0
 MaxNumberRobotsOnArc[16,7]: w[16,7] - 2 v[16,7] <= 0
 MaxNumberRobotsOnArc[16,8]: w[16,8] - 2 v[16,8] <= 0
 MaxNumberRobotsOnArc[16,9]: w[16,9] - 2 v[16,9] <= 0
 MaxNumberRobotsOnArc[16,10]: w[16,10] - 2 v[16,10] <= 0
 MaxNumberRobotsOnArc[16,11]: w[16,11] - 2 v[16,11] <= 0
 MaxNumberRobotsOnArc[16,12]: w[16,12] - 2 v[16,12] <= 0
 MaxNumberRobotsOnArc[16,13]: w[16,13] - 2 v[16,13] <= 0
 MaxNumberRobotsOnArc[16,14]: w[16,14] - 2 v[16,14] <= 0
 MaxNumberRobotsOnArc[16,15]: w[16,15] - 2 v[16,15] <= 0
 MaxNumberRobotsOnArc[16,16]: w[16,16] - 2 v[16,16] <= 0
 MaxNumberRobotsOnArc[16,17]: w[16,17] - 2 v[16,17] <= 0
 MaxNumberRobotsOnArc[16,18]: w[16,18] - 2 v[16,18] <= 0
 MaxNumberRobotsOnArc[16,19]: w[16,19] - 2 v[16,19] <= 0
 MaxNumberRobotsOnArc[16,20]: w[16,20] - 2 v[16,20] <= 0
 MaxNumberRobotsOnArc[16,21]: w[16,21] - 2 v[16,21] <= 0
 MaxNumberRobotsOnArc[16,22]: w[16,22] - 2 v[16,22] <= 0
 MaxNumberRobotsOnArc[16,23]: w[16,23] - 2 v[16,23] <= 0
 MaxNumberRobotsOnArc[16,24]: w[16,24] - 2 v[16,24] <= 0
 MaxNumberRobotsOnArc[16,25]: w[16,25] - 2 v[16,25] <= 0
 MaxNumberRobotsOnArc[17,1]: w[17,1] - 2 v[17,1] <= 0
 MaxNumberRobotsOnArc[17,2]: w[17,2] - 2 v[17,2] <= 0
 MaxNumberRobotsOnArc[17,3]: w[17,3] - 2 v[17,3] <= 0
 MaxNumberRobotsOnArc[17,4]: w[17,4] - 2 v[17,4] <= 0
 MaxNumberRobotsOnArc[17,5]: w[17,5] - 2 v[17,5] <= 0
 MaxNumberRobotsOnArc[17,6]: w[17,6] - 2 v[17,6] <= 0
 MaxNumberRobotsOnArc[17,7]: w[17,7] - 2 v[17,7] <= 0
 MaxNumberRobotsOnArc[17,8]: w[17,8] - 2 v[17,8] <= 0
 MaxNumberRobotsOnArc[17,9]: w[17,9] - 2 v[17,9] <= 0
 MaxNumberRobotsOnArc[17,10]: w[17,10] - 2 v[17,10] <= 0
 MaxNumberRobotsOnArc[17,11]: w[17,11] - 2 v[17,11] <= 0
 MaxNumberRobotsOnArc[17,12]: w[17,12] - 2 v[17,12] <= 0
 MaxNumberRobotsOnArc[17,13]: w[17,13] - 2 v[17,13] <= 0
 MaxNumberRobotsOnArc[17,14]: w[17,14] - 2 v[17,14] <= 0
 MaxNumberRobotsOnArc[17,15]: w[17,15] - 2 v[17,15] <= 0
 MaxNumberRobotsOnArc[17,16]: w[17,16] - 2 v[17,16] <= 0
 MaxNumberRobotsOnArc[17,17]: w[17,17] - 2 v[17,17] <= 0
 MaxNumberRobotsOnArc[17,18]: w[17,18] - 2 v[17,18] <= 0
 MaxNumberRobotsOnArc[17,19]: w[17,19] - 2 v[17,19] <= 0
 MaxNumberRobotsOnArc[17,20]: w[17,20] - 2 v[17,20] <= 0
 MaxNumberRobotsOnArc[17,21]: w[17,21] - 2 v[17,21] <= 0
 MaxNumberRobotsOnArc[17,22]: w[17,22] - 2 v[17,22] <= 0
 MaxNumberRobotsOnArc[17,23]: w[17,23] - 2 v[17,23] <= 0
 MaxNumberRobotsOnArc[17,24]: w[17,24] - 2 v[17,24] <= 0
 MaxNumberRobotsOnArc[17,25]: w[17,25] - 2 v[17,25] <= 0
 MaxNumberRobotsOnArc[18,1]: w[18,1] - 2 v[18,1] <= 0
 MaxNumberRobotsOnArc[18,2]: w[18,2] - 2 v[18,2] <= 0
 MaxNumberRobotsOnArc[18,3]: w[18,3] - 2 v[18,3] <= 0
 MaxNumberRobotsOnArc[18,4]: w[18,4] - 2 v[18,4] <= 0
 MaxNumberRobotsOnArc[18,5]: w[18,5] - 2 v[18,5] <= 0
 MaxNumberRobotsOnArc[18,6]: w[18,6] - 2 v[18,6] <= 0
 MaxNumberRobotsOnArc[18,7]: w[18,7] - 2 v[18,7] <= 0
 MaxNumberRobotsOnArc[18,8]: w[18,8] - 2 v[18,8] <= 0
 MaxNumberRobotsOnArc[18,9]: w[18,9] - 2 v[18,9] <= 0
 MaxNumberRobotsOnArc[18,10]: w[18,10] - 2 v[18,10] <= 0
 MaxNumberRobotsOnArc[18,11]: w[18,11] - 2 v[18,11] <= 0
 MaxNumberRobotsOnArc[18,12]: w[18,12] - 2 v[18,12] <= 0
 MaxNumberRobotsOnArc[18,13]: w[18,13] - 2 v[18,13] <= 0
 MaxNumberRobotsOnArc[18,14]: w[18,14] - 2 v[18,14] <= 0
 MaxNumberRobotsOnArc[18,15]: w[18,15] - 2 v[18,15] <= 0
 MaxNumberRobotsOnArc[18,16]: w[18,16] - 2 v[18,16] <= 0
 MaxNumberRobotsOnArc[18,17]: w[18,17] - 2 v[18,17] <= 0
 MaxNumberRobotsOnArc[18,18]: w[18,18] - 2 v[18,18] <= 0
 MaxNumberRobotsOnArc[18,19]: w[18,19] - 2 v[18,19] <= 0
 MaxNumberRobotsOnArc[18,20]: w[18,20] - 2 v[18,20] <= 0
 MaxNumberRobotsOnArc[18,21]: w[18,21] - 2 v[18,21] <= 0
 MaxNumberRobotsOnArc[18,22]: w[18,22] - 2 v[18,22] <= 0
 MaxNumberRobotsOnArc[18,23]: w[18,23] - 2 v[18,23] <= 0
 MaxNumberRobotsOnArc[18,24]: w[18,24] - 2 v[18,24] <= 0
 MaxNumberRobotsOnArc[18,25]: w[18,25] - 2 v[18,25] <= 0
 MaxNumberRobotsOnArc[19,1]: w[19,1] - 2 v[19,1] <= 0
 MaxNumberRobotsOnArc[19,2]: w[19,2] - 2 v[19,2] <= 0
 MaxNumberRobotsOnArc[19,3]: w[19,3] - 2 v[19,3] <= 0
 MaxNumberRobotsOnArc[19,4]: w[19,4] - 2 v[19,4] <= 0
 MaxNumberRobotsOnArc[19,5]: w[19,5] - 2 v[19,5] <= 0
 MaxNumberRobotsOnArc[19,6]: w[19,6] - 2 v[19,6] <= 0
 MaxNumberRobotsOnArc[19,7]: w[19,7] - 2 v[19,7] <= 0
 MaxNumberRobotsOnArc[19,8]: w[19,8] - 2 v[19,8] <= 0
 MaxNumberRobotsOnArc[19,9]: w[19,9] - 2 v[19,9] <= 0
 MaxNumberRobotsOnArc[19,10]: w[19,10] - 2 v[19,10] <= 0
 MaxNumberRobotsOnArc[19,11]: w[19,11] - 2 v[19,11] <= 0
 MaxNumberRobotsOnArc[19,12]: w[19,12] - 2 v[19,12] <= 0
 MaxNumberRobotsOnArc[19,13]: w[19,13] - 2 v[19,13] <= 0
 MaxNumberRobotsOnArc[19,14]: w[19,14] - 2 v[19,14] <= 0
 MaxNumberRobotsOnArc[19,15]: w[19,15] - 2 v[19,15] <= 0
 MaxNumberRobotsOnArc[19,16]: w[19,16] - 2 v[19,16] <= 0
 MaxNumberRobotsOnArc[19,17]: w[19,17] - 2 v[19,17] <= 0
 MaxNumberRobotsOnArc[19,18]: w[19,18] - 2 v[19,18] <= 0
 MaxNumberRobotsOnArc[19,19]: w[19,19] - 2 v[19,19] <= 0
 MaxNumberRobotsOnArc[19,20]: w[19,20] - 2 v[19,20] <= 0
 MaxNumberRobotsOnArc[19,21]: w[19,21] - 2 v[19,21] <= 0
 MaxNumberRobotsOnArc[19,22]: w[19,22] - 2 v[19,22] <= 0
 MaxNumberRobotsOnArc[19,23]: w[19,23] - 2 v[19,23] <= 0
 MaxNumberRobotsOnArc[19,24]: w[19,24] - 2 v[19,24] <= 0
 MaxNumberRobotsOnArc[19,25]: w[19,25] - 2 v[19,25] <= 0
 MaxNumberRobotsOnArc[20,1]: w[20,1] - 2 v[20,1] <= 0
 MaxNumberRobotsOnArc[20,2]: w[20,2] - 2 v[20,2] <= 0
 MaxNumberRobotsOnArc[20,3]: w[20,3] - 2 v[20,3] <= 0
 MaxNumberRobotsOnArc[20,4]: w[20,4] - 2 v[20,4] <= 0
 MaxNumberRobotsOnArc[20,5]: w[20,5] - 2 v[20,5] <= 0
 MaxNumberRobotsOnArc[20,6]: w[20,6] - 2 v[20,6] <= 0
 MaxNumberRobotsOnArc[20,7]: w[20,7] - 2 v[20,7] <= 0
 MaxNumberRobotsOnArc[20,8]: w[20,8] - 2 v[20,8] <= 0
 MaxNumberRobotsOnArc[20,9]: w[20,9] - 2 v[20,9] <= 0
 MaxNumberRobotsOnArc[20,10]: w[20,10] - 2 v[20,10] <= 0
 MaxNumberRobotsOnArc[20,11]: w[20,11] - 2 v[20,11] <= 0
 MaxNumberRobotsOnArc[20,12]: w[20,12] - 2 v[20,12] <= 0
 MaxNumberRobotsOnArc[20,13]: w[20,13] - 2 v[20,13] <= 0
 MaxNumberRobotsOnArc[20,14]: w[20,14] - 2 v[20,14] <= 0
 MaxNumberRobotsOnArc[20,15]: w[20,15] - 2 v[20,15] <= 0
 MaxNumberRobotsOnArc[20,16]: w[20,16] - 2 v[20,16] <= 0
 MaxNumberRobotsOnArc[20,17]: w[20,17] - 2 v[20,17] <= 0
 MaxNumberRobotsOnArc[20,18]: w[20,18] - 2 v[20,18] <= 0
 MaxNumberRobotsOnArc[20,19]: w[20,19] - 2 v[20,19] <= 0
 MaxNumberRobotsOnArc[20,20]: w[20,20] - 2 v[20,20] <= 0
 MaxNumberRobotsOnArc[20,21]: w[20,21] - 2 v[20,21] <= 0
 MaxNumberRobotsOnArc[20,22]: w[20,22] - 2 v[20,22] <= 0
 MaxNumberRobotsOnArc[20,23]: w[20,23] - 2 v[20,23] <= 0
 MaxNumberRobotsOnArc[20,24]: w[20,24] - 2 v[20,24] <= 0
 MaxNumberRobotsOnArc[20,25]: w[20,25] - 2 v[20,25] <= 0
 MaxNumberRobotsOnArc[21,1]: w[21,1] - 2 v[21,1] <= 0
 MaxNumberRobotsOnArc[21,2]: w[21,2] - 2 v[21,2] <= 0
 MaxNumberRobotsOnArc[21,3]: w[21,3] - 2 v[21,3] <= 0
 MaxNumberRobotsOnArc[21,4]: w[21,4] - 2 v[21,4] <= 0
 MaxNumberRobotsOnArc[21,5]: w[21,5] - 2 v[21,5] <= 0
 MaxNumberRobotsOnArc[21,6]: w[21,6] - 2 v[21,6] <= 0
 MaxNumberRobotsOnArc[21,7]: w[21,7] - 2 v[21,7] <= 0
 MaxNumberRobotsOnArc[21,8]: w[21,8] - 2 v[21,8] <= 0
 MaxNumberRobotsOnArc[21,9]: w[21,9] - 2 v[21,9] <= 0
 MaxNumberRobotsOnArc[21,10]: w[21,10] - 2 v[21,10] <= 0
 MaxNumberRobotsOnArc[21,11]: w[21,11] - 2 v[21,11] <= 0
 MaxNumberRobotsOnArc[21,12]: w[21,12] - 2 v[21,12] <= 0
 MaxNumberRobotsOnArc[21,13]: w[21,13] - 2 v[21,13] <= 0
 MaxNumberRobotsOnArc[21,14]: w[21,14] - 2 v[21,14] <= 0
 MaxNumberRobotsOnArc[21,15]: w[21,15] - 2 v[21,15] <= 0
 MaxNumberRobotsOnArc[21,16]: w[21,16] - 2 v[21,16] <= 0
 MaxNumberRobotsOnArc[21,17]: w[21,17] - 2 v[21,17] <= 0
 MaxNumberRobotsOnArc[21,18]: w[21,18] - 2 v[21,18] <= 0
 MaxNumberRobotsOnArc[21,19]: w[21,19] - 2 v[21,19] <= 0
 MaxNumberRobotsOnArc[21,20]: w[21,20] - 2 v[21,20] <= 0
 MaxNumberRobotsOnArc[21,21]: w[21,21] - 2 v[21,21] <= 0
 MaxNumberRobotsOnArc[21,22]: w[21,22] - 2 v[21,22] <= 0
 MaxNumberRobotsOnArc[21,23]: w[21,23] - 2 v[21,23] <= 0
 MaxNumberRobotsOnArc[21,24]: w[21,24] - 2 v[21,24] <= 0
 MaxNumberRobotsOnArc[21,25]: w[21,25] - 2 v[21,25] <= 0
 MaxNumberRobotsOnArc[22,1]: w[22,1] - 2 v[22,1] <= 0
 MaxNumberRobotsOnArc[22,2]: w[22,2] - 2 v[22,2] <= 0
 MaxNumberRobotsOnArc[22,3]: w[22,3] - 2 v[22,3] <= 0
 MaxNumberRobotsOnArc[22,4]: w[22,4] - 2 v[22,4] <= 0
 MaxNumberRobotsOnArc[22,5]: w[22,5] - 2 v[22,5] <= 0
 MaxNumberRobotsOnArc[22,6]: w[22,6] - 2 v[22,6] <= 0
 MaxNumberRobotsOnArc[22,7]: w[22,7] - 2 v[22,7] <= 0
 MaxNumberRobotsOnArc[22,8]: w[22,8] - 2 v[22,8] <= 0
 MaxNumberRobotsOnArc[22,9]: w[22,9] - 2 v[22,9] <= 0
 MaxNumberRobotsOnArc[22,10]: w[22,10] - 2 v[22,10] <= 0
 MaxNumberRobotsOnArc[22,11]: w[22,11] - 2 v[22,11] <= 0
 MaxNumberRobotsOnArc[22,12]: w[22,12] - 2 v[22,12] <= 0
 MaxNumberRobotsOnArc[22,13]: w[22,13] - 2 v[22,13] <= 0
 MaxNumberRobotsOnArc[22,14]: w[22,14] - 2 v[22,14] <= 0
 MaxNumberRobotsOnArc[22,15]: w[22,15] - 2 v[22,15] <= 0
 MaxNumberRobotsOnArc[22,16]: w[22,16] - 2 v[22,16] <= 0
 MaxNumberRobotsOnArc[22,17]: w[22,17] - 2 v[22,17] <= 0
 MaxNumberRobotsOnArc[22,18]: w[22,18] - 2 v[22,18] <= 0
 MaxNumberRobotsOnArc[22,19]: w[22,19] - 2 v[22,19] <= 0
 MaxNumberRobotsOnArc[22,20]: w[22,20] - 2 v[22,20] <= 0
 MaxNumberRobotsOnArc[22,21]: w[22,21] - 2 v[22,21] <= 0
 MaxNumberRobotsOnArc[22,22]: w[22,22] - 2 v[22,22] <= 0
 MaxNumberRobotsOnArc[22,23]: w[22,23] - 2 v[22,23] <= 0
 MaxNumberRobotsOnArc[22,24]: w[22,24] - 2 v[22,24] <= 0
 MaxNumberRobotsOnArc[22,25]: w[22,25] - 2 v[22,25] <= 0
 MaxNumberRobotsOnArc[23,1]: w[23,1] - 2 v[23,1] <= 0
 MaxNumberRobotsOnArc[23,2]: w[23,2] - 2 v[23,2] <= 0
 MaxNumberRobotsOnArc[23,3]: w[23,3] - 2 v[23,3] <= 0
 MaxNumberRobotsOnArc[23,4]: w[23,4] - 2 v[23,4] <= 0
 MaxNumberRobotsOnArc[23,5]: w[23,5] - 2 v[23,5] <= 0
 MaxNumberRobotsOnArc[23,6]: w[23,6] - 2 v[23,6] <= 0
 MaxNumberRobotsOnArc[23,7]: w[23,7] - 2 v[23,7] <= 0
 MaxNumberRobotsOnArc[23,8]: w[23,8] - 2 v[23,8] <= 0
 MaxNumberRobotsOnArc[23,9]: w[23,9] - 2 v[23,9] <= 0
 MaxNumberRobotsOnArc[23,10]: w[23,10] - 2 v[23,10] <= 0
 MaxNumberRobotsOnArc[23,11]: w[23,11] - 2 v[23,11] <= 0
 MaxNumberRobotsOnArc[23,12]: w[23,12] - 2 v[23,12] <= 0
 MaxNumberRobotsOnArc[23,13]: w[23,13] - 2 v[23,13] <= 0
 MaxNumberRobotsOnArc[23,14]: w[23,14] - 2 v[23,14] <= 0
 MaxNumberRobotsOnArc[23,15]: w[23,15] - 2 v[23,15] <= 0
 MaxNumberRobotsOnArc[23,16]: w[23,16] - 2 v[23,16] <= 0
 MaxNumberRobotsOnArc[23,17]: w[23,17] - 2 v[23,17] <= 0
 MaxNumberRobotsOnArc[23,18]: w[23,18] - 2 v[23,18] <= 0
 MaxNumberRobotsOnArc[23,19]: w[23,19] - 2 v[23,19] <= 0
 MaxNumberRobotsOnArc[23,20]: w[23,20] - 2 v[23,20] <= 0
 MaxNumberRobotsOnArc[23,21]: w[23,21] - 2 v[23,21] <= 0
 MaxNumberRobotsOnArc[23,22]: w[23,22] - 2 v[23,22] <= 0
 MaxNumberRobotsOnArc[23,23]: w[23,23] - 2 v[23,23] <= 0
 MaxNumberRobotsOnArc[23,24]: w[23,24] - 2 v[23,24] <= 0
 MaxNumberRobotsOnArc[23,25]: w[23,25] - 2 v[23,25] <= 0
 MaxNumberRobotsOnArc[24,1]: w[24,1] - 2 v[24,1] <= 0
 MaxNumberRobotsOnArc[24,2]: w[24,2] - 2 v[24,2] <= 0
 MaxNumberRobotsOnArc[24,3]: w[24,3] - 2 v[24,3] <= 0
 MaxNumberRobotsOnArc[24,4]: w[24,4] - 2 v[24,4] <= 0
 MaxNumberRobotsOnArc[24,5]: w[24,5] - 2 v[24,5] <= 0
 MaxNumberRobotsOnArc[24,6]: w[24,6] - 2 v[24,6] <= 0
 MaxNumberRobotsOnArc[24,7]: w[24,7] - 2 v[24,7] <= 0
 MaxNumberRobotsOnArc[24,8]: w[24,8] - 2 v[24,8] <= 0
 MaxNumberRobotsOnArc[24,9]: w[24,9] - 2 v[24,9] <= 0
 MaxNumberRobotsOnArc[24,10]: w[24,10] - 2 v[24,10] <= 0
 MaxNumberRobotsOnArc[24,11]: w[24,11] - 2 v[24,11] <= 0
 MaxNumberRobotsOnArc[24,12]: w[24,12] - 2 v[24,12] <= 0
 MaxNumberRobotsOnArc[24,13]: w[24,13] - 2 v[24,13] <= 0
 MaxNumberRobotsOnArc[24,14]: w[24,14] - 2 v[24,14] <= 0
 MaxNumberRobotsOnArc[24,15]: w[24,15] - 2 v[24,15] <= 0
 MaxNumberRobotsOnArc[24,16]: w[24,16] - 2 v[24,16] <= 0
 MaxNumberRobotsOnArc[24,17]: w[24,17] - 2 v[24,17] <= 0
 MaxNumberRobotsOnArc[24,18]: w[24,18] - 2 v[24,18] <= 0
 MaxNumberRobotsOnArc[24,19]: w[24,19] - 2 v[24,19] <= 0
 MaxNumberRobotsOnArc[24,20]: w[24,20] - 2 v[24,20] <= 0
 MaxNumberRobotsOnArc[24,21]: w[24,21] - 2 v[24,21] <= 0
 MaxNumberRobotsOnArc[24,22]: w[24,22] - 2 v[24,22] <= 0
 MaxNumberRobotsOnArc[24,23]: w[24,23] - 2 v[24,23] <= 0
 MaxNumberRobotsOnArc[24,24]: w[24,24] - 2 v[24,24] <= 0
 MaxNumberRobotsOnArc[24,25]: w[24,25] - 2 v[24,25] <= 0
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
 ArcUsedByRobot[0,24]: w[0,24] - v[0,24] >= 0
 ArcUsedByRobot[0,25]: w[0,25] - v[0,25] >= 0
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
 ArcUsedByRobot[1,24]: w[1,24] - v[1,24] >= 0
 ArcUsedByRobot[1,25]: w[1,25] - v[1,25] >= 0
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
 ArcUsedByRobot[2,24]: w[2,24] - v[2,24] >= 0
 ArcUsedByRobot[2,25]: w[2,25] - v[2,25] >= 0
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
 ArcUsedByRobot[3,24]: w[3,24] - v[3,24] >= 0
 ArcUsedByRobot[3,25]: w[3,25] - v[3,25] >= 0
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
 ArcUsedByRobot[4,24]: w[4,24] - v[4,24] >= 0
 ArcUsedByRobot[4,25]: w[4,25] - v[4,25] >= 0
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
 ArcUsedByRobot[5,24]: w[5,24] - v[5,24] >= 0
 ArcUsedByRobot[5,25]: w[5,25] - v[5,25] >= 0
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
 ArcUsedByRobot[6,24]: w[6,24] - v[6,24] >= 0
 ArcUsedByRobot[6,25]: w[6,25] - v[6,25] >= 0
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
 ArcUsedByRobot[7,24]: w[7,24] - v[7,24] >= 0
 ArcUsedByRobot[7,25]: w[7,25] - v[7,25] >= 0
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
 ArcUsedByRobot[8,24]: w[8,24] - v[8,24] >= 0
 ArcUsedByRobot[8,25]: w[8,25] - v[8,25] >= 0
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
 ArcUsedByRobot[9,24]: w[9,24] - v[9,24] >= 0
 ArcUsedByRobot[9,25]: w[9,25] - v[9,25] >= 0
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
 ArcUsedByRobot[10,24]: w[10,24] - v[10,24] >= 0
 ArcUsedByRobot[10,25]: w[10,25] - v[10,25] >= 0
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
 ArcUsedByRobot[11,24]: w[11,24] - v[11,24] >= 0
 ArcUsedByRobot[11,25]: w[11,25] - v[11,25] >= 0
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
 ArcUsedByRobot[12,24]: w[12,24] - v[12,24] >= 0
 ArcUsedByRobot[12,25]: w[12,25] - v[12,25] >= 0
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
 ArcUsedByRobot[13,24]: w[13,24] - v[13,24] >= 0
 ArcUsedByRobot[13,25]: w[13,25] - v[13,25] >= 0
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
 ArcUsedByRobot[14,24]: w[14,24] - v[14,24] >= 0
 ArcUsedByRobot[14,25]: w[14,25] - v[14,25] >= 0
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
 ArcUsedByRobot[15,24]: w[15,24] - v[15,24] >= 0
 ArcUsedByRobot[15,25]: w[15,25] - v[15,25] >= 0
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
 ArcUsedByRobot[16,24]: w[16,24] - v[16,24] >= 0
 ArcUsedByRobot[16,25]: w[16,25] - v[16,25] >= 0
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
 ArcUsedByRobot[17,24]: w[17,24] - v[17,24] >= 0
 ArcUsedByRobot[17,25]: w[17,25] - v[17,25] >= 0
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
 ArcUsedByRobot[18,24]: w[18,24] - v[18,24] >= 0
 ArcUsedByRobot[18,25]: w[18,25] - v[18,25] >= 0
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
 ArcUsedByRobot[19,24]: w[19,24] - v[19,24] >= 0
 ArcUsedByRobot[19,25]: w[19,25] - v[19,25] >= 0
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
 ArcUsedByRobot[20,24]: w[20,24] - v[20,24] >= 0
 ArcUsedByRobot[20,25]: w[20,25] - v[20,25] >= 0
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
 ArcUsedByRobot[21,24]: w[21,24] - v[21,24] >= 0
 ArcUsedByRobot[21,25]: w[21,25] - v[21,25] >= 0
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
 ArcUsedByRobot[22,24]: w[22,24] - v[22,24] >= 0
 ArcUsedByRobot[22,25]: w[22,25] - v[22,25] >= 0
 ArcUsedByRobot[23,1]: w[23,1] - v[23,1] >= 0
 ArcUsedByRobot[23,2]: w[23,2] - v[23,2] >= 0
 ArcUsedByRobot[23,3]: w[23,3] - v[23,3] >= 0
 ArcUsedByRobot[23,4]: w[23,4] - v[23,4] >= 0
 ArcUsedByRobot[23,5]: w[23,5] - v[23,5] >= 0
 ArcUsedByRobot[23,6]: w[23,6] - v[23,6] >= 0
 ArcUsedByRobot[23,7]: w[23,7] - v[23,7] >= 0
 ArcUsedByRobot[23,8]: w[23,8] - v[23,8] >= 0
 ArcUsedByRobot[23,9]: w[23,9] - v[23,9] >= 0
 ArcUsedByRobot[23,10]: w[23,10] - v[23,10] >= 0
 ArcUsedByRobot[23,11]: w[23,11] - v[23,11] >= 0
 ArcUsedByRobot[23,12]: w[23,12] - v[23,12] >= 0
 ArcUsedByRobot[23,13]: w[23,13] - v[23,13] >= 0
 ArcUsedByRobot[23,14]: w[23,14] - v[23,14] >= 0
 ArcUsedByRobot[23,15]: w[23,15] - v[23,15] >= 0
 ArcUsedByRobot[23,16]: w[23,16] - v[23,16] >= 0
 ArcUsedByRobot[23,17]: w[23,17] - v[23,17] >= 0
 ArcUsedByRobot[23,18]: w[23,18] - v[23,18] >= 0
 ArcUsedByRobot[23,19]: w[23,19] - v[23,19] >= 0
 ArcUsedByRobot[23,20]: w[23,20] - v[23,20] >= 0
 ArcUsedByRobot[23,21]: w[23,21] - v[23,21] >= 0
 ArcUsedByRobot[23,22]: w[23,22] - v[23,22] >= 0
 ArcUsedByRobot[23,23]: w[23,23] - v[23,23] >= 0
 ArcUsedByRobot[23,24]: w[23,24] - v[23,24] >= 0
 ArcUsedByRobot[23,25]: w[23,25] - v[23,25] >= 0
 ArcUsedByRobot[24,1]: w[24,1] - v[24,1] >= 0
 ArcUsedByRobot[24,2]: w[24,2] - v[24,2] >= 0
 ArcUsedByRobot[24,3]: w[24,3] - v[24,3] >= 0
 ArcUsedByRobot[24,4]: w[24,4] - v[24,4] >= 0
 ArcUsedByRobot[24,5]: w[24,5] - v[24,5] >= 0
 ArcUsedByRobot[24,6]: w[24,6] - v[24,6] >= 0
 ArcUsedByRobot[24,7]: w[24,7] - v[24,7] >= 0
 ArcUsedByRobot[24,8]: w[24,8] - v[24,8] >= 0
 ArcUsedByRobot[24,9]: w[24,9] - v[24,9] >= 0
 ArcUsedByRobot[24,10]: w[24,10] - v[24,10] >= 0
 ArcUsedByRobot[24,11]: w[24,11] - v[24,11] >= 0
 ArcUsedByRobot[24,12]: w[24,12] - v[24,12] >= 0
 ArcUsedByRobot[24,13]: w[24,13] - v[24,13] >= 0
 ArcUsedByRobot[24,14]: w[24,14] - v[24,14] >= 0
 ArcUsedByRobot[24,15]: w[24,15] - v[24,15] >= 0
 ArcUsedByRobot[24,16]: w[24,16] - v[24,16] >= 0
 ArcUsedByRobot[24,17]: w[24,17] - v[24,17] >= 0
 ArcUsedByRobot[24,18]: w[24,18] - v[24,18] >= 0
 ArcUsedByRobot[24,19]: w[24,19] - v[24,19] >= 0
 ArcUsedByRobot[24,20]: w[24,20] - v[24,20] >= 0
 ArcUsedByRobot[24,21]: w[24,21] - v[24,21] >= 0
 ArcUsedByRobot[24,22]: w[24,22] - v[24,22] >= 0
 ArcUsedByRobot[24,23]: w[24,23] - v[24,23] >= 0
 ArcUsedByRobot[24,24]: w[24,24] - v[24,24] >= 0
 ArcUsedByRobot[24,25]: w[24,25] - v[24,25] >= 0
 StartServiceRobots[0,1]: - 999999.01 v[0,1] - start_service[0]
   + start_service[1] >= -999999
 StartServiceRobots[0,2]: - 999999.01 v[0,2] - start_service[0]
   + start_service[2] >= -999999
 StartServiceRobots[0,3]: - 1.0000538900827599e+06 v[0,3]
   - start_service[0] + start_service[3] >= -999999
 StartServiceRobots[0,4]: - 1.0000315560591351e+06 v[0,4]
   - start_service[0] + start_service[4] >= -999999
 StartServiceRobots[0,5]: - 1.0000086606162182e+06 v[0,5]
   - start_service[0] + start_service[5] >= -999999
 StartServiceRobots[0,6]: - 1.0000880721248598e+06 v[0,6]
   - start_service[0] + start_service[6] >= -999999
 StartServiceRobots[0,7]: - 1.0000120359099368e+06 v[0,7]
   - start_service[0] + start_service[7] >= -999999
 StartServiceRobots[0,8]: - 1.0000538900827599e+06 v[0,8]
   - start_service[0] + start_service[8] >= -999999
 StartServiceRobots[0,9]: - 1.0000315560591351e+06 v[0,9]
   - start_service[0] + start_service[9] >= -999999
 StartServiceRobots[0,10]: - 1.0000086606162182e+06 v[0,10]
   - start_service[0] + start_service[10] >= -999999
 StartServiceRobots[0,11]: - 1.0000880721248598e+06 v[0,11]
   - start_service[0] + start_service[11] >= -999999
 StartServiceRobots[0,12]: - 1.0000120359099368e+06 v[0,12]
   - start_service[0] + start_service[12] >= -999999
 StartServiceRobots[0,13]: - 1.0000538900827599e+06 v[0,13]
   - start_service[0] + start_service[13] >= -999999
 StartServiceRobots[0,14]: - 1.0000315560591351e+06 v[0,14]
   - start_service[0] + start_service[14] >= -999999
 StartServiceRobots[0,15]: - 1.0000086606162182e+06 v[0,15]
   - start_service[0] + start_service[15] >= -999999
 StartServiceRobots[0,16]: - 1.0000880721248598e+06 v[0,16]
   - start_service[0] + start_service[16] >= -999999
 StartServiceRobots[0,17]: - 1.0000120359099368e+06 v[0,17]
   - start_service[0] + start_service[17] >= -999999
 StartServiceRobots[0,18]: - 1.0000538900827599e+06 v[0,18]
   - start_service[0] + start_service[18] >= -999999
 StartServiceRobots[0,19]: - 1.0000315560591351e+06 v[0,19]
   - start_service[0] + start_service[19] >= -999999
 StartServiceRobots[0,20]: - 1.0000086606162182e+06 v[0,20]
   - start_service[0] + start_service[20] >= -999999
 StartServiceRobots[0,21]: - 1.0000880721248598e+06 v[0,21]
   - start_service[0] + start_service[21] >= -999999
 StartServiceRobots[0,22]: - 1.0000120359099368e+06 v[0,22]
   - start_service[0] + start_service[22] >= -999999
 StartServiceRobots[0,23]: - 999999.01 v[0,23] - start_service[0]
   + start_service[23] >= -999999
 StartServiceRobots[0,24]: - 999999.01 v[0,24] - start_service[0]
   + start_service[24] >= -999999
 StartServiceRobots[0,25]: - 999999.01 v[0,25] - start_service[0]
   + start_service[25] >= -999999
 StartServiceRobots[1,1]: - 999999.01 v[1,1] >= -999999
 StartServiceRobots[1,2]: - 999999.01 v[1,2] - start_service[1]
   + start_service[2] >= -999999
 StartServiceRobots[1,3]: - 1.0000538900827599e+06 v[1,3]
   - start_service[1] + start_service[3] >= -999999
 StartServiceRobots[1,4]: - 1.0000315560591351e+06 v[1,4]
   - start_service[1] + start_service[4] >= -999999
 StartServiceRobots[1,5]: - 1.0000086606162182e+06 v[1,5]
   - start_service[1] + start_service[5] >= -999999
 StartServiceRobots[1,6]: - 1.0000880721248598e+06 v[1,6]
   - start_service[1] + start_service[6] >= -999999
 StartServiceRobots[1,7]: - 1.0000120359099368e+06 v[1,7]
   - start_service[1] + start_service[7] >= -999999
 StartServiceRobots[1,8]: - 1.0000538900827599e+06 v[1,8]
   - start_service[1] + start_service[8] >= -999999
 StartServiceRobots[1,9]: - 1.0000315560591351e+06 v[1,9]
   - start_service[1] + start_service[9] >= -999999
 StartServiceRobots[1,10]: - 1.0000086606162182e+06 v[1,10]
   - start_service[1] + start_service[10] >= -999999
 StartServiceRobots[1,11]: - 1.0000880721248598e+06 v[1,11]
   - start_service[1] + start_service[11] >= -999999
 StartServiceRobots[1,12]: - 1.0000120359099368e+06 v[1,12]
   - start_service[1] + start_service[12] >= -999999
 StartServiceRobots[1,13]: - 1.0000538900827599e+06 v[1,13]
   - start_service[1] + start_service[13] >= -999999
 StartServiceRobots[1,14]: - 1.0000315560591351e+06 v[1,14]
   - start_service[1] + start_service[14] >= -999999
 StartServiceRobots[1,15]: - 1.0000086606162182e+06 v[1,15]
   - start_service[1] + start_service[15] >= -999999
 StartServiceRobots[1,16]: - 1.0000880721248598e+06 v[1,16]
   - start_service[1] + start_service[16] >= -999999
 StartServiceRobots[1,17]: - 1.0000120359099368e+06 v[1,17]
   - start_service[1] + start_service[17] >= -999999
 StartServiceRobots[1,18]: - 1.0000538900827599e+06 v[1,18]
   - start_service[1] + start_service[18] >= -999999
 StartServiceRobots[1,19]: - 1.0000315560591351e+06 v[1,19]
   - start_service[1] + start_service[19] >= -999999
 StartServiceRobots[1,20]: - 1.0000086606162182e+06 v[1,20]
   - start_service[1] + start_service[20] >= -999999
 StartServiceRobots[1,21]: - 1.0000880721248598e+06 v[1,21]
   - start_service[1] + start_service[21] >= -999999
 StartServiceRobots[1,22]: - 1.0000120359099368e+06 v[1,22]
   - start_service[1] + start_service[22] >= -999999
 StartServiceRobots[1,23]: - 999999.01 v[1,23] - start_service[1]
   + start_service[23] >= -999999
 StartServiceRobots[1,24]: - 999999.01 v[1,24] - start_service[1]
   + start_service[24] >= -999999
 StartServiceRobots[1,25]: - 999999.01 v[1,25] - start_service[1]
   + start_service[25] >= -999999
 StartServiceRobots[2,1]: - 999999.01 v[2,1] + start_service[1]
   - start_service[2] >= -999999
 StartServiceRobots[2,2]: - 999999.01 v[2,2] >= -999999
 StartServiceRobots[2,3]: - 1.0000538900827599e+06 v[2,3]
   - start_service[2] + start_service[3] >= -999999
 StartServiceRobots[2,4]: - 1.0000315560591351e+06 v[2,4]
   - start_service[2] + start_service[4] >= -999999
 StartServiceRobots[2,5]: - 1.0000086606162182e+06 v[2,5]
   - start_service[2] + start_service[5] >= -999999
 StartServiceRobots[2,6]: - 1.0000880721248598e+06 v[2,6]
   - start_service[2] + start_service[6] >= -999999
 StartServiceRobots[2,7]: - 1.0000120359099368e+06 v[2,7]
   - start_service[2] + start_service[7] >= -999999
 StartServiceRobots[2,8]: - 1.0000538900827599e+06 v[2,8]
   - start_service[2] + start_service[8] >= -999999
 StartServiceRobots[2,9]: - 1.0000315560591351e+06 v[2,9]
   - start_service[2] + start_service[9] >= -999999
 StartServiceRobots[2,10]: - 1.0000086606162182e+06 v[2,10]
   - start_service[2] + start_service[10] >= -999999
 StartServiceRobots[2,11]: - 1.0000880721248598e+06 v[2,11]
   - start_service[2] + start_service[11] >= -999999
 StartServiceRobots[2,12]: - 1.0000120359099368e+06 v[2,12]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 1.0000538900827599e+06 v[2,13]
   - start_service[2] + start_service[13] >= -999999
 StartServiceRobots[2,14]: - 1.0000315560591351e+06 v[2,14]
   - start_service[2] + start_service[14] >= -999999
 StartServiceRobots[2,15]: - 1.0000086606162182e+06 v[2,15]
   - start_service[2] + start_service[15] >= -999999
 StartServiceRobots[2,16]: - 1.0000880721248598e+06 v[2,16]
   - start_service[2] + start_service[16] >= -999999
 StartServiceRobots[2,17]: - 1.0000120359099368e+06 v[2,17]
   - start_service[2] + start_service[17] >= -999999
 StartServiceRobots[2,18]: - 1.0000538900827599e+06 v[2,18]
   - start_service[2] + start_service[18] >= -999999
 StartServiceRobots[2,19]: - 1.0000315560591351e+06 v[2,19]
   - start_service[2] + start_service[19] >= -999999
 StartServiceRobots[2,20]: - 1.0000086606162182e+06 v[2,20]
   - start_service[2] + start_service[20] >= -999999
 StartServiceRobots[2,21]: - 1.0000880721248598e+06 v[2,21]
   - start_service[2] + start_service[21] >= -999999
 StartServiceRobots[2,22]: - 1.0000120359099368e+06 v[2,22]
   - start_service[2] + start_service[22] >= -999999
 StartServiceRobots[2,23]: - 999999.01 v[2,23] - start_service[2]
   + start_service[23] >= -999999
 StartServiceRobots[2,24]: - 999999.01 v[2,24] - start_service[2]
   + start_service[24] >= -999999
 StartServiceRobots[2,25]: - 999999.01 v[2,25] - start_service[2]
   + start_service[25] >= -999999
 StartServiceRobots[3,1]: - 1.0000538900827599e+06 v[3,1]
   + start_service[1] - start_service[3] >= -999999
 StartServiceRobots[3,2]: - 1.0000538900827599e+06 v[3,2]
   + start_service[2] - start_service[3] >= -999999
 StartServiceRobots[3,3]: - 999999.01 v[3,3] >= -999999
 StartServiceRobots[3,4]: - 1.0000399322151314e+06 v[3,4]
   - start_service[3] + start_service[4] >= -999999
 StartServiceRobots[3,5]: - 1.0000562206776621e+06 v[3,5]
   - start_service[3] + start_service[5] >= -999999
 StartServiceRobots[3,6]: - 1.0000706995536003e+06 v[3,6]
   - start_service[3] + start_service[6] >= -999999
 StartServiceRobots[3,7]: - 1.0000408655636616e+06 v[3,7]
   - start_service[3] + start_service[7] >= -999999
 StartServiceRobots[3,8]: - 999999.01 v[3,8] - start_service[3]
   + start_service[8] >= -999999
 StartServiceRobots[3,9]: - 1.0000399322151314e+06 v[3,9]
   - start_service[3] + start_service[9] >= -999999
 StartServiceRobots[3,10]: - 1.0000562206776621e+06 v[3,10]
   - start_service[3] + start_service[10] >= -999999
 StartServiceRobots[3,11]: - 1.0000706995536003e+06 v[3,11]
   - start_service[3] + start_service[11] >= -999999
 StartServiceRobots[3,12]: - 1.0000408655636616e+06 v[3,12]
   - start_service[3] + start_service[12] >= -999999
 StartServiceRobots[3,13]: - 999999.01 v[3,13] - start_service[3]
   + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 1.0000399322151314e+06 v[3,14]
   - start_service[3] + start_service[14] >= -999999
 StartServiceRobots[3,15]: - 1.0000562206776621e+06 v[3,15]
   - start_service[3] + start_service[15] >= -999999
 StartServiceRobots[3,16]: - 1.0000706995536003e+06 v[3,16]
   - start_service[3] + start_service[16] >= -999999
 StartServiceRobots[3,17]: - 1.0000408655636616e+06 v[3,17]
   - start_service[3] + start_service[17] >= -999999
 StartServiceRobots[3,18]: - 999999.01 v[3,18] - start_service[3]
   + start_service[18] >= -999999
 StartServiceRobots[3,19]: - 1.0000399322151314e+06 v[3,19]
   - start_service[3] + start_service[19] >= -999999
 StartServiceRobots[3,20]: - 1.0000562206776621e+06 v[3,20]
   - start_service[3] + start_service[20] >= -999999
 StartServiceRobots[3,21]: - 1.0000706995536003e+06 v[3,21]
   - start_service[3] + start_service[21] >= -999999
 StartServiceRobots[3,22]: - 1.0000408655636616e+06 v[3,22]
   - start_service[3] + start_service[22] >= -999999
 StartServiceRobots[3,23]: - 1.0000538900827599e+06 v[3,23]
   - start_service[3] + start_service[23] >= -999999
 StartServiceRobots[3,24]: - 1.0000538900827599e+06 v[3,24]
   - start_service[3] + start_service[24] >= -999999
 StartServiceRobots[3,25]: - 1.0000538900827599e+06 v[3,25]
   - start_service[3] + start_service[25] >= -999999
 StartServiceRobots[4,1]: - 1.0000315560591351e+06 v[4,1]
   + start_service[1] - start_service[4] >= -999999
 StartServiceRobots[4,2]: - 1.0000315560591351e+06 v[4,2]
   + start_service[2] - start_service[4] >= -999999
 StartServiceRobots[4,3]: - 1.0000399322151314e+06 v[4,3]
   + start_service[3] - start_service[4] >= -999999
 StartServiceRobots[4,4]: - 999999.01 v[4,4] >= -999999
 StartServiceRobots[4,5]: - 1.0000399960725273e+06 v[4,5]
   - start_service[4] + start_service[5] >= -999999
 StartServiceRobots[4,6]: - 1.0000557694957714e+06 v[4,6]
   - start_service[4] + start_service[6] >= -999999
 StartServiceRobots[4,7]: - 1.0000242562644748e+06 v[4,7]
   - start_service[4] + start_service[7] >= -999999
 StartServiceRobots[4,8]: - 1.0000399322151314e+06 v[4,8]
   - start_service[4] + start_service[8] >= -999999
 StartServiceRobots[4,9]: - 999999.01 v[4,9] - start_service[4]
   + start_service[9] >= -999999
 StartServiceRobots[4,10]: - 1.0000399960725273e+06 v[4,10]
   - start_service[4] + start_service[10] >= -999999
 StartServiceRobots[4,11]: - 1.0000557694957714e+06 v[4,11]
   - start_service[4] + start_service[11] >= -999999
 StartServiceRobots[4,12]: - 1.0000242562644748e+06 v[4,12]
   - start_service[4] + start_service[12] >= -999999
 StartServiceRobots[4,13]: - 1.0000399322151314e+06 v[4,13]
   - start_service[4] + start_service[13] >= -999999
 StartServiceRobots[4,14]: - 999999.01 v[4,14] - start_service[4]
   + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 1.0000399960725273e+06 v[4,15]
   - start_service[4] + start_service[15] >= -999999
 StartServiceRobots[4,16]: - 1.0000557694957714e+06 v[4,16]
   - start_service[4] + start_service[16] >= -999999
 StartServiceRobots[4,17]: - 1.0000242562644748e+06 v[4,17]
   - start_service[4] + start_service[17] >= -999999
 StartServiceRobots[4,18]: - 1.0000399322151314e+06 v[4,18]
   - start_service[4] + start_service[18] >= -999999
 StartServiceRobots[4,19]: - 999999.01 v[4,19] - start_service[4]
   + start_service[19] >= -999999
 StartServiceRobots[4,20]: - 1.0000399960725273e+06 v[4,20]
   - start_service[4] + start_service[20] >= -999999
 StartServiceRobots[4,21]: - 1.0000557694957714e+06 v[4,21]
   - start_service[4] + start_service[21] >= -999999
 StartServiceRobots[4,22]: - 1.0000242562644748e+06 v[4,22]
   - start_service[4] + start_service[22] >= -999999
 StartServiceRobots[4,23]: - 1.0000315560591351e+06 v[4,23]
   - start_service[4] + start_service[23] >= -999999
 StartServiceRobots[4,24]: - 1.0000315560591351e+06 v[4,24]
   - start_service[4] + start_service[24] >= -999999
 StartServiceRobots[4,25]: - 1.0000315560591351e+06 v[4,25]
   - start_service[4] + start_service[25] >= -999999
 StartServiceRobots[5,1]: - 1.0000086606162182e+06 v[5,1]
   + start_service[1] - start_service[5] >= -999999
 StartServiceRobots[5,2]: - 1.0000086606162182e+06 v[5,2]
   + start_service[2] - start_service[5] >= -999999
 StartServiceRobots[5,3]: - 1.0000562206776621e+06 v[5,3]
   + start_service[3] - start_service[5] >= -999999
 StartServiceRobots[5,4]: - 1.0000399960725273e+06 v[5,4]
   + start_service[4] - start_service[5] >= -999999
 StartServiceRobots[5,5]: - 999999.01 v[5,5] >= -999999
 StartServiceRobots[5,6]: - 1.0000967563598768e+06 v[5,6]
   - start_service[5] + start_service[6] >= -999999
 StartServiceRobots[5,7]: - 1.0000166692290857e+06 v[5,7]
   - start_service[5] + start_service[7] >= -999999
 StartServiceRobots[5,8]: - 1.0000562206776621e+06 v[5,8]
   - start_service[5] + start_service[8] >= -999999
 StartServiceRobots[5,9]: - 1.0000399960725273e+06 v[5,9]
   - start_service[5] + start_service[9] >= -999999
 StartServiceRobots[5,10]: - 999999.01 v[5,10] - start_service[5]
   + start_service[10] >= -999999
 StartServiceRobots[5,11]: - 1.0000967563598768e+06 v[5,11]
   - start_service[5] + start_service[11] >= -999999
 StartServiceRobots[5,12]: - 1.0000166692290857e+06 v[5,12]
   - start_service[5] + start_service[12] >= -999999
 StartServiceRobots[5,13]: - 1.0000562206776621e+06 v[5,13]
   - start_service[5] + start_service[13] >= -999999
 StartServiceRobots[5,14]: - 1.0000399960725273e+06 v[5,14]
   - start_service[5] + start_service[14] >= -999999
 StartServiceRobots[5,15]: - 999999.01 v[5,15] - start_service[5]
   + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 1.0000967563598768e+06 v[5,16]
   - start_service[5] + start_service[16] >= -999999
 StartServiceRobots[5,17]: - 1.0000166692290857e+06 v[5,17]
   - start_service[5] + start_service[17] >= -999999
 StartServiceRobots[5,18]: - 1.0000562206776621e+06 v[5,18]
   - start_service[5] + start_service[18] >= -999999
 StartServiceRobots[5,19]: - 1.0000399960725273e+06 v[5,19]
   - start_service[5] + start_service[19] >= -999999
 StartServiceRobots[5,20]: - 999999.01 v[5,20] - start_service[5]
   + start_service[20] >= -999999
 StartServiceRobots[5,21]: - 1.0000967563598768e+06 v[5,21]
   - start_service[5] + start_service[21] >= -999999
 StartServiceRobots[5,22]: - 1.0000166692290857e+06 v[5,22]
   - start_service[5] + start_service[22] >= -999999
 StartServiceRobots[5,23]: - 1.0000086606162182e+06 v[5,23]
   - start_service[5] + start_service[23] >= -999999
 StartServiceRobots[5,24]: - 1.0000086606162182e+06 v[5,24]
   - start_service[5] + start_service[24] >= -999999
 StartServiceRobots[5,25]: - 1.0000086606162182e+06 v[5,25]
   - start_service[5] + start_service[25] >= -999999
 StartServiceRobots[6,1]: - 1.0000880721248598e+06 v[6,1]
   + start_service[1] - start_service[6] >= -999999
 StartServiceRobots[6,2]: - 1.0000880721248598e+06 v[6,2]
   + start_service[2] - start_service[6] >= -999999
 StartServiceRobots[6,3]: - 1.0000706995536003e+06 v[6,3]
   + start_service[3] - start_service[6] >= -999999
 StartServiceRobots[6,4]: - 1.0000557694957714e+06 v[6,4]
   + start_service[4] - start_service[6] >= -999999
 StartServiceRobots[6,5]: - 1.0000967563598768e+06 v[6,5]
   + start_service[5] - start_service[6] >= -999999
 StartServiceRobots[6,6]: - 999999.01 v[6,6] >= -999999
 StartServiceRobots[6,7]: - 1.0000805943163345e+06 v[6,7]
   - start_service[6] + start_service[7] >= -999999
 StartServiceRobots[6,8]: - 1.0000706995536003e+06 v[6,8]
   - start_service[6] + start_service[8] >= -999999
 StartServiceRobots[6,9]: - 1.0000557694957714e+06 v[6,9]
   - start_service[6] + start_service[9] >= -999999
 StartServiceRobots[6,10]: - 1.0000967563598768e+06 v[6,10]
   - start_service[6] + start_service[10] >= -999999
 StartServiceRobots[6,11]: - 999999.01 v[6,11] - start_service[6]
   + start_service[11] >= -999999
 StartServiceRobots[6,12]: - 1.0000805943163345e+06 v[6,12]
   - start_service[6] + start_service[12] >= -999999
 StartServiceRobots[6,13]: - 1.0000706995536003e+06 v[6,13]
   - start_service[6] + start_service[13] >= -999999
 StartServiceRobots[6,14]: - 1.0000557694957714e+06 v[6,14]
   - start_service[6] + start_service[14] >= -999999
 StartServiceRobots[6,15]: - 1.0000967563598768e+06 v[6,15]
   - start_service[6] + start_service[15] >= -999999
 StartServiceRobots[6,16]: - 999999.01 v[6,16] - start_service[6]
   + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 1.0000805943163345e+06 v[6,17]
   - start_service[6] + start_service[17] >= -999999
 StartServiceRobots[6,18]: - 1.0000706995536003e+06 v[6,18]
   - start_service[6] + start_service[18] >= -999999
 StartServiceRobots[6,19]: - 1.0000557694957714e+06 v[6,19]
   - start_service[6] + start_service[19] >= -999999
 StartServiceRobots[6,20]: - 1.0000967563598768e+06 v[6,20]
   - start_service[6] + start_service[20] >= -999999
 StartServiceRobots[6,21]: - 999999.01 v[6,21] - start_service[6]
   + start_service[21] >= -999999
 StartServiceRobots[6,22]: - 1.0000805943163345e+06 v[6,22]
   - start_service[6] + start_service[22] >= -999999
 StartServiceRobots[6,23]: - 1.0000880721248598e+06 v[6,23]
   - start_service[6] + start_service[23] >= -999999
 StartServiceRobots[6,24]: - 1.0000880721248598e+06 v[6,24]
   - start_service[6] + start_service[24] >= -999999
 StartServiceRobots[6,25]: - 1.0000880721248598e+06 v[6,25]
   - start_service[6] + start_service[25] >= -999999
 StartServiceRobots[7,1]: - 1.0000120359099368e+06 v[7,1]
   + start_service[1] - start_service[7] >= -999999
 StartServiceRobots[7,2]: - 1.0000120359099368e+06 v[7,2]
   + start_service[2] - start_service[7] >= -999999
 StartServiceRobots[7,3]: - 1.0000408655636616e+06 v[7,3]
   + start_service[3] - start_service[7] >= -999999
 StartServiceRobots[7,4]: - 1.0000242562644748e+06 v[7,4]
   + start_service[4] - start_service[7] >= -999999
 StartServiceRobots[7,5]: - 1.0000166692290857e+06 v[7,5]
   + start_service[5] - start_service[7] >= -999999
 StartServiceRobots[7,6]: - 1.0000805943163345e+06 v[7,6]
   + start_service[6] - start_service[7] >= -999999
 StartServiceRobots[7,7]: - 999999.01 v[7,7] >= -999999
 StartServiceRobots[7,8]: - 1.0000408655636616e+06 v[7,8]
   - start_service[7] + start_service[8] >= -999999
 StartServiceRobots[7,9]: - 1.0000242562644748e+06 v[7,9]
   - start_service[7] + start_service[9] >= -999999
 StartServiceRobots[7,10]: - 1.0000166692290857e+06 v[7,10]
   - start_service[7] + start_service[10] >= -999999
 StartServiceRobots[7,11]: - 1.0000805943163345e+06 v[7,11]
   - start_service[7] + start_service[11] >= -999999
 StartServiceRobots[7,12]: - 999999.01 v[7,12] - start_service[7]
   + start_service[12] >= -999999
 StartServiceRobots[7,13]: - 1.0000408655636616e+06 v[7,13]
   - start_service[7] + start_service[13] >= -999999
 StartServiceRobots[7,14]: - 1.0000242562644748e+06 v[7,14]
   - start_service[7] + start_service[14] >= -999999
 StartServiceRobots[7,15]: - 1.0000166692290857e+06 v[7,15]
   - start_service[7] + start_service[15] >= -999999
 StartServiceRobots[7,16]: - 1.0000805943163345e+06 v[7,16]
   - start_service[7] + start_service[16] >= -999999
 StartServiceRobots[7,17]: - 999999.01 v[7,17] - start_service[7]
   + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 1.0000408655636616e+06 v[7,18]
   - start_service[7] + start_service[18] >= -999999
 StartServiceRobots[7,19]: - 1.0000242562644748e+06 v[7,19]
   - start_service[7] + start_service[19] >= -999999
 StartServiceRobots[7,20]: - 1.0000166692290857e+06 v[7,20]
   - start_service[7] + start_service[20] >= -999999
 StartServiceRobots[7,21]: - 1.0000805943163345e+06 v[7,21]
   - start_service[7] + start_service[21] >= -999999
 StartServiceRobots[7,22]: - 999999.01 v[7,22] - start_service[7]
   + start_service[22] >= -999999
 StartServiceRobots[7,23]: - 1.0000120359099368e+06 v[7,23]
   - start_service[7] + start_service[23] >= -999999
 StartServiceRobots[7,24]: - 1.0000120359099368e+06 v[7,24]
   - start_service[7] + start_service[24] >= -999999
 StartServiceRobots[7,25]: - 1.0000120359099368e+06 v[7,25]
   - start_service[7] + start_service[25] >= -999999
 StartServiceRobots[8,1]: - 1.0000538900827599e+06 v[8,1]
   + start_service[1] - start_service[8] >= -999999
 StartServiceRobots[8,2]: - 1.0000538900827599e+06 v[8,2]
   + start_service[2] - start_service[8] >= -999999
 StartServiceRobots[8,3]: - 999999.01 v[8,3] + start_service[3]
   - start_service[8] >= -999999
 StartServiceRobots[8,4]: - 1.0000399322151314e+06 v[8,4]
   + start_service[4] - start_service[8] >= -999999
 StartServiceRobots[8,5]: - 1.0000562206776621e+06 v[8,5]
   + start_service[5] - start_service[8] >= -999999
 StartServiceRobots[8,6]: - 1.0000706995536003e+06 v[8,6]
   + start_service[6] - start_service[8] >= -999999
 StartServiceRobots[8,7]: - 1.0000408655636616e+06 v[8,7]
   + start_service[7] - start_service[8] >= -999999
 StartServiceRobots[8,8]: - 999999.01 v[8,8] >= -999999
 StartServiceRobots[8,9]: - 1.0000399322151314e+06 v[8,9]
   - start_service[8] + start_service[9] >= -999999
 StartServiceRobots[8,10]: - 1.0000562206776621e+06 v[8,10]
   - start_service[8] + start_service[10] >= -999999
 StartServiceRobots[8,11]: - 1.0000706995536003e+06 v[8,11]
   - start_service[8] + start_service[11] >= -999999
 StartServiceRobots[8,12]: - 1.0000408655636616e+06 v[8,12]
   - start_service[8] + start_service[12] >= -999999
 StartServiceRobots[8,13]: - 999999.01 v[8,13] - start_service[8]
   + start_service[13] >= -999999
 StartServiceRobots[8,14]: - 1.0000399322151314e+06 v[8,14]
   - start_service[8] + start_service[14] >= -999999
 StartServiceRobots[8,15]: - 1.0000562206776621e+06 v[8,15]
   - start_service[8] + start_service[15] >= -999999
 StartServiceRobots[8,16]: - 1.0000706995536003e+06 v[8,16]
   - start_service[8] + start_service[16] >= -999999
 StartServiceRobots[8,17]: - 1.0000408655636616e+06 v[8,17]
   - start_service[8] + start_service[17] >= -999999
 StartServiceRobots[8,18]: - 999999.01 v[8,18] - start_service[8]
   + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 1.0000399322151314e+06 v[8,19]
   - start_service[8] + start_service[19] >= -999999
 StartServiceRobots[8,20]: - 1.0000562206776621e+06 v[8,20]
   - start_service[8] + start_service[20] >= -999999
 StartServiceRobots[8,21]: - 1.0000706995536003e+06 v[8,21]
   - start_service[8] + start_service[21] >= -999999
 StartServiceRobots[8,22]: - 1.0000408655636616e+06 v[8,22]
   - start_service[8] + start_service[22] >= -999999
 StartServiceRobots[8,23]: - 1.0000538900827599e+06 v[8,23]
   - start_service[8] + start_service[23] >= -999999
 StartServiceRobots[8,24]: - 1.0000538900827599e+06 v[8,24]
   - start_service[8] + start_service[24] >= -999999
 StartServiceRobots[8,25]: - 1.0000538900827599e+06 v[8,25]
   - start_service[8] + start_service[25] >= -999999
 StartServiceRobots[9,1]: - 1.0000315560591351e+06 v[9,1]
   + start_service[1] - start_service[9] >= -999999
 StartServiceRobots[9,2]: - 1.0000315560591351e+06 v[9,2]
   + start_service[2] - start_service[9] >= -999999
 StartServiceRobots[9,3]: - 1.0000399322151314e+06 v[9,3]
   + start_service[3] - start_service[9] >= -999999
 StartServiceRobots[9,4]: - 999999.01 v[9,4] + start_service[4]
   - start_service[9] >= -999999
 StartServiceRobots[9,5]: - 1.0000399960725273e+06 v[9,5]
   + start_service[5] - start_service[9] >= -999999
 StartServiceRobots[9,6]: - 1.0000557694957714e+06 v[9,6]
   + start_service[6] - start_service[9] >= -999999
 StartServiceRobots[9,7]: - 1.0000242562644748e+06 v[9,7]
   + start_service[7] - start_service[9] >= -999999
 StartServiceRobots[9,8]: - 1.0000399322151314e+06 v[9,8]
   + start_service[8] - start_service[9] >= -999999
 StartServiceRobots[9,9]: - 999999.01 v[9,9] >= -999999
 StartServiceRobots[9,10]: - 1.0000399960725273e+06 v[9,10]
   - start_service[9] + start_service[10] >= -999999
 StartServiceRobots[9,11]: - 1.0000557694957714e+06 v[9,11]
   - start_service[9] + start_service[11] >= -999999
 StartServiceRobots[9,12]: - 1.0000242562644748e+06 v[9,12]
   - start_service[9] + start_service[12] >= -999999
 StartServiceRobots[9,13]: - 1.0000399322151314e+06 v[9,13]
   - start_service[9] + start_service[13] >= -999999
 StartServiceRobots[9,14]: - 999999.01 v[9,14] - start_service[9]
   + start_service[14] >= -999999
 StartServiceRobots[9,15]: - 1.0000399960725273e+06 v[9,15]
   - start_service[9] + start_service[15] >= -999999
 StartServiceRobots[9,16]: - 1.0000557694957714e+06 v[9,16]
   - start_service[9] + start_service[16] >= -999999
 StartServiceRobots[9,17]: - 1.0000242562644748e+06 v[9,17]
   - start_service[9] + start_service[17] >= -999999
 StartServiceRobots[9,18]: - 1.0000399322151314e+06 v[9,18]
   - start_service[9] + start_service[18] >= -999999
 StartServiceRobots[9,19]: - 999999.01 v[9,19] - start_service[9]
   + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 1.0000399960725273e+06 v[9,20]
   - start_service[9] + start_service[20] >= -999999
 StartServiceRobots[9,21]: - 1.0000557694957714e+06 v[9,21]
   - start_service[9] + start_service[21] >= -999999
 StartServiceRobots[9,22]: - 1.0000242562644748e+06 v[9,22]
   - start_service[9] + start_service[22] >= -999999
 StartServiceRobots[9,23]: - 1.0000315560591351e+06 v[9,23]
   - start_service[9] + start_service[23] >= -999999
 StartServiceRobots[9,24]: - 1.0000315560591351e+06 v[9,24]
   - start_service[9] + start_service[24] >= -999999
 StartServiceRobots[9,25]: - 1.0000315560591351e+06 v[9,25]
   - start_service[9] + start_service[25] >= -999999
 StartServiceRobots[10,1]: - 1.0000086606162182e+06 v[10,1]
   + start_service[1] - start_service[10] >= -999999
 StartServiceRobots[10,2]: - 1.0000086606162182e+06 v[10,2]
   + start_service[2] - start_service[10] >= -999999
 StartServiceRobots[10,3]: - 1.0000562206776621e+06 v[10,3]
   + start_service[3] - start_service[10] >= -999999
 StartServiceRobots[10,4]: - 1.0000399960725273e+06 v[10,4]
   + start_service[4] - start_service[10] >= -999999
 StartServiceRobots[10,5]: - 999999.01 v[10,5] + start_service[5]
   - start_service[10] >= -999999
 StartServiceRobots[10,6]: - 1.0000967563598768e+06 v[10,6]
   + start_service[6] - start_service[10] >= -999999
 StartServiceRobots[10,7]: - 1.0000166692290857e+06 v[10,7]
   + start_service[7] - start_service[10] >= -999999
 StartServiceRobots[10,8]: - 1.0000562206776621e+06 v[10,8]
   + start_service[8] - start_service[10] >= -999999
 StartServiceRobots[10,9]: - 1.0000399960725273e+06 v[10,9]
   + start_service[9] - start_service[10] >= -999999
 StartServiceRobots[10,10]: - 999999.01 v[10,10] >= -999999
 StartServiceRobots[10,11]: - 1.0000967563598768e+06 v[10,11]
   - start_service[10] + start_service[11] >= -999999
 StartServiceRobots[10,12]: - 1.0000166692290857e+06 v[10,12]
   - start_service[10] + start_service[12] >= -999999
 StartServiceRobots[10,13]: - 1.0000562206776621e+06 v[10,13]
   - start_service[10] + start_service[13] >= -999999
 StartServiceRobots[10,14]: - 1.0000399960725273e+06 v[10,14]
   - start_service[10] + start_service[14] >= -999999
 StartServiceRobots[10,15]: - 999999.01 v[10,15] - start_service[10]
   + start_service[15] >= -999999
 StartServiceRobots[10,16]: - 1.0000967563598768e+06 v[10,16]
   - start_service[10] + start_service[16] >= -999999
 StartServiceRobots[10,17]: - 1.0000166692290857e+06 v[10,17]
   - start_service[10] + start_service[17] >= -999999
 StartServiceRobots[10,18]: - 1.0000562206776621e+06 v[10,18]
   - start_service[10] + start_service[18] >= -999999
 StartServiceRobots[10,19]: - 1.0000399960725273e+06 v[10,19]
   - start_service[10] + start_service[19] >= -999999
 StartServiceRobots[10,20]: - 999999.01 v[10,20] - start_service[10]
   + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 1.0000967563598768e+06 v[10,21]
   - start_service[10] + start_service[21] >= -999999
 StartServiceRobots[10,22]: - 1.0000166692290857e+06 v[10,22]
   - start_service[10] + start_service[22] >= -999999
 StartServiceRobots[10,23]: - 1.0000086606162182e+06 v[10,23]
   - start_service[10] + start_service[23] >= -999999
 StartServiceRobots[10,24]: - 1.0000086606162182e+06 v[10,24]
   - start_service[10] + start_service[24] >= -999999
 StartServiceRobots[10,25]: - 1.0000086606162182e+06 v[10,25]
   - start_service[10] + start_service[25] >= -999999
 StartServiceRobots[11,1]: - 1.0000880721248598e+06 v[11,1]
   + start_service[1] - start_service[11] >= -999999
 StartServiceRobots[11,2]: - 1.0000880721248598e+06 v[11,2]
   + start_service[2] - start_service[11] >= -999999
 StartServiceRobots[11,3]: - 1.0000706995536003e+06 v[11,3]
   + start_service[3] - start_service[11] >= -999999
 StartServiceRobots[11,4]: - 1.0000557694957714e+06 v[11,4]
   + start_service[4] - start_service[11] >= -999999
 StartServiceRobots[11,5]: - 1.0000967563598768e+06 v[11,5]
   + start_service[5] - start_service[11] >= -999999
 StartServiceRobots[11,6]: - 999999.01 v[11,6] + start_service[6]
   - start_service[11] >= -999999
 StartServiceRobots[11,7]: - 1.0000805943163345e+06 v[11,7]
   + start_service[7] - start_service[11] >= -999999
 StartServiceRobots[11,8]: - 1.0000706995536003e+06 v[11,8]
   + start_service[8] - start_service[11] >= -999999
 StartServiceRobots[11,9]: - 1.0000557694957714e+06 v[11,9]
   + start_service[9] - start_service[11] >= -999999
 StartServiceRobots[11,10]: - 1.0000967563598768e+06 v[11,10]
   + start_service[10] - start_service[11] >= -999999
 StartServiceRobots[11,11]: - 999999.01 v[11,11] >= -999999
 StartServiceRobots[11,12]: - 1.0000805943163345e+06 v[11,12]
   - start_service[11] + start_service[12] >= -999999
 StartServiceRobots[11,13]: - 1.0000706995536003e+06 v[11,13]
   - start_service[11] + start_service[13] >= -999999
 StartServiceRobots[11,14]: - 1.0000557694957714e+06 v[11,14]
   - start_service[11] + start_service[14] >= -999999
 StartServiceRobots[11,15]: - 1.0000967563598768e+06 v[11,15]
   - start_service[11] + start_service[15] >= -999999
 StartServiceRobots[11,16]: - 999999.01 v[11,16] - start_service[11]
   + start_service[16] >= -999999
 StartServiceRobots[11,17]: - 1.0000805943163345e+06 v[11,17]
   - start_service[11] + start_service[17] >= -999999
 StartServiceRobots[11,18]: - 1.0000706995536003e+06 v[11,18]
   - start_service[11] + start_service[18] >= -999999
 StartServiceRobots[11,19]: - 1.0000557694957714e+06 v[11,19]
   - start_service[11] + start_service[19] >= -999999
 StartServiceRobots[11,20]: - 1.0000967563598768e+06 v[11,20]
   - start_service[11] + start_service[20] >= -999999
 StartServiceRobots[11,21]: - 999999.01 v[11,21] - start_service[11]
   + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 1.0000805943163345e+06 v[11,22]
   - start_service[11] + start_service[22] >= -999999
 StartServiceRobots[11,23]: - 1.0000880721248598e+06 v[11,23]
   - start_service[11] + start_service[23] >= -999999
 StartServiceRobots[11,24]: - 1.0000880721248598e+06 v[11,24]
   - start_service[11] + start_service[24] >= -999999
 StartServiceRobots[11,25]: - 1.0000880721248598e+06 v[11,25]
   - start_service[11] + start_service[25] >= -999999
 StartServiceRobots[12,1]: - 1.0000120359099368e+06 v[12,1]
   + start_service[1] - start_service[12] >= -999999
 StartServiceRobots[12,2]: - 1.0000120359099368e+06 v[12,2]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 1.0000408655636616e+06 v[12,3]
   + start_service[3] - start_service[12] >= -999999
 StartServiceRobots[12,4]: - 1.0000242562644748e+06 v[12,4]
   + start_service[4] - start_service[12] >= -999999
 StartServiceRobots[12,5]: - 1.0000166692290857e+06 v[12,5]
   + start_service[5] - start_service[12] >= -999999
 StartServiceRobots[12,6]: - 1.0000805943163345e+06 v[12,6]
   + start_service[6] - start_service[12] >= -999999
 StartServiceRobots[12,7]: - 999999.01 v[12,7] + start_service[7]
   - start_service[12] >= -999999
 StartServiceRobots[12,8]: - 1.0000408655636616e+06 v[12,8]
   + start_service[8] - start_service[12] >= -999999
 StartServiceRobots[12,9]: - 1.0000242562644748e+06 v[12,9]
   + start_service[9] - start_service[12] >= -999999
 StartServiceRobots[12,10]: - 1.0000166692290857e+06 v[12,10]
   + start_service[10] - start_service[12] >= -999999
 StartServiceRobots[12,11]: - 1.0000805943163345e+06 v[12,11]
   + start_service[11] - start_service[12] >= -999999
 StartServiceRobots[12,12]: - 999999.01 v[12,12] >= -999999
 StartServiceRobots[12,13]: - 1.0000408655636616e+06 v[12,13]
   - start_service[12] + start_service[13] >= -999999
 StartServiceRobots[12,14]: - 1.0000242562644748e+06 v[12,14]
   - start_service[12] + start_service[14] >= -999999
 StartServiceRobots[12,15]: - 1.0000166692290857e+06 v[12,15]
   - start_service[12] + start_service[15] >= -999999
 StartServiceRobots[12,16]: - 1.0000805943163345e+06 v[12,16]
   - start_service[12] + start_service[16] >= -999999
 StartServiceRobots[12,17]: - 999999.01 v[12,17] - start_service[12]
   + start_service[17] >= -999999
 StartServiceRobots[12,18]: - 1.0000408655636616e+06 v[12,18]
   - start_service[12] + start_service[18] >= -999999
 StartServiceRobots[12,19]: - 1.0000242562644748e+06 v[12,19]
   - start_service[12] + start_service[19] >= -999999
 StartServiceRobots[12,20]: - 1.0000166692290857e+06 v[12,20]
   - start_service[12] + start_service[20] >= -999999
 StartServiceRobots[12,21]: - 1.0000805943163345e+06 v[12,21]
   - start_service[12] + start_service[21] >= -999999
 StartServiceRobots[12,22]: - 999999.01 v[12,22] - start_service[12]
   + start_service[22] >= -999999
 StartServiceRobots[12,23]: - 1.0000120359099368e+06 v[12,23]
   - start_service[12] + start_service[23] >= -999999
 StartServiceRobots[12,24]: - 1.0000120359099368e+06 v[12,24]
   - start_service[12] + start_service[24] >= -999999
 StartServiceRobots[12,25]: - 1.0000120359099368e+06 v[12,25]
   - start_service[12] + start_service[25] >= -999999
 StartServiceRobots[13,1]: - 1.0000538900827599e+06 v[13,1]
   + start_service[1] - start_service[13] >= -999999
 StartServiceRobots[13,2]: - 1.0000538900827599e+06 v[13,2]
   + start_service[2] - start_service[13] >= -999999
 StartServiceRobots[13,3]: - 999999.01 v[13,3] + start_service[3]
   - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 1.0000399322151314e+06 v[13,4]
   + start_service[4] - start_service[13] >= -999999
 StartServiceRobots[13,5]: - 1.0000562206776621e+06 v[13,5]
   + start_service[5] - start_service[13] >= -999999
 StartServiceRobots[13,6]: - 1.0000706995536003e+06 v[13,6]
   + start_service[6] - start_service[13] >= -999999
 StartServiceRobots[13,7]: - 1.0000408655636616e+06 v[13,7]
   + start_service[7] - start_service[13] >= -999999
 StartServiceRobots[13,8]: - 999999.01 v[13,8] + start_service[8]
   - start_service[13] >= -999999
 StartServiceRobots[13,9]: - 1.0000399322151314e+06 v[13,9]
   + start_service[9] - start_service[13] >= -999999
 StartServiceRobots[13,10]: - 1.0000562206776621e+06 v[13,10]
   + start_service[10] - start_service[13] >= -999999
 StartServiceRobots[13,11]: - 1.0000706995536003e+06 v[13,11]
   + start_service[11] - start_service[13] >= -999999
 StartServiceRobots[13,12]: - 1.0000408655636616e+06 v[13,12]
   + start_service[12] - start_service[13] >= -999999
 StartServiceRobots[13,13]: - 999999.01 v[13,13] >= -999999
 StartServiceRobots[13,14]: - 1.0000399322151314e+06 v[13,14]
   - start_service[13] + start_service[14] >= -999999
 StartServiceRobots[13,15]: - 1.0000562206776621e+06 v[13,15]
   - start_service[13] + start_service[15] >= -999999
 StartServiceRobots[13,16]: - 1.0000706995536003e+06 v[13,16]
   - start_service[13] + start_service[16] >= -999999
 StartServiceRobots[13,17]: - 1.0000408655636616e+06 v[13,17]
   - start_service[13] + start_service[17] >= -999999
 StartServiceRobots[13,18]: - 999999.01 v[13,18] - start_service[13]
   + start_service[18] >= -999999
 StartServiceRobots[13,19]: - 1.0000399322151314e+06 v[13,19]
   - start_service[13] + start_service[19] >= -999999
 StartServiceRobots[13,20]: - 1.0000562206776621e+06 v[13,20]
   - start_service[13] + start_service[20] >= -999999
 StartServiceRobots[13,21]: - 1.0000706995536003e+06 v[13,21]
   - start_service[13] + start_service[21] >= -999999
 StartServiceRobots[13,22]: - 1.0000408655636616e+06 v[13,22]
   - start_service[13] + start_service[22] >= -999999
 StartServiceRobots[13,23]: - 1.0000538900827599e+06 v[13,23]
   - start_service[13] + start_service[23] >= -999999
 StartServiceRobots[13,24]: - 1.0000538900827599e+06 v[13,24]
   - start_service[13] + start_service[24] >= -999999
 StartServiceRobots[13,25]: - 1.0000538900827599e+06 v[13,25]
   - start_service[13] + start_service[25] >= -999999
 StartServiceRobots[14,1]: - 1.0000315560591351e+06 v[14,1]
   + start_service[1] - start_service[14] >= -999999
 StartServiceRobots[14,2]: - 1.0000315560591351e+06 v[14,2]
   + start_service[2] - start_service[14] >= -999999
 StartServiceRobots[14,3]: - 1.0000399322151314e+06 v[14,3]
   + start_service[3] - start_service[14] >= -999999
 StartServiceRobots[14,4]: - 999999.01 v[14,4] + start_service[4]
   - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 1.0000399960725273e+06 v[14,5]
   + start_service[5] - start_service[14] >= -999999
 StartServiceRobots[14,6]: - 1.0000557694957714e+06 v[14,6]
   + start_service[6] - start_service[14] >= -999999
 StartServiceRobots[14,7]: - 1.0000242562644748e+06 v[14,7]
   + start_service[7] - start_service[14] >= -999999
 StartServiceRobots[14,8]: - 1.0000399322151314e+06 v[14,8]
   + start_service[8] - start_service[14] >= -999999
 StartServiceRobots[14,9]: - 999999.01 v[14,9] + start_service[9]
   - start_service[14] >= -999999
 StartServiceRobots[14,10]: - 1.0000399960725273e+06 v[14,10]
   + start_service[10] - start_service[14] >= -999999
 StartServiceRobots[14,11]: - 1.0000557694957714e+06 v[14,11]
   + start_service[11] - start_service[14] >= -999999
 StartServiceRobots[14,12]: - 1.0000242562644748e+06 v[14,12]
   + start_service[12] - start_service[14] >= -999999
 StartServiceRobots[14,13]: - 1.0000399322151314e+06 v[14,13]
   + start_service[13] - start_service[14] >= -999999
 StartServiceRobots[14,14]: - 999999.01 v[14,14] >= -999999
 StartServiceRobots[14,15]: - 1.0000399960725273e+06 v[14,15]
   - start_service[14] + start_service[15] >= -999999
 StartServiceRobots[14,16]: - 1.0000557694957714e+06 v[14,16]
   - start_service[14] + start_service[16] >= -999999
 StartServiceRobots[14,17]: - 1.0000242562644748e+06 v[14,17]
   - start_service[14] + start_service[17] >= -999999
 StartServiceRobots[14,18]: - 1.0000399322151314e+06 v[14,18]
   - start_service[14] + start_service[18] >= -999999
 StartServiceRobots[14,19]: - 999999.01 v[14,19] - start_service[14]
   + start_service[19] >= -999999
 StartServiceRobots[14,20]: - 1.0000399960725273e+06 v[14,20]
   - start_service[14] + start_service[20] >= -999999
 StartServiceRobots[14,21]: - 1.0000557694957714e+06 v[14,21]
   - start_service[14] + start_service[21] >= -999999
 StartServiceRobots[14,22]: - 1.0000242562644748e+06 v[14,22]
   - start_service[14] + start_service[22] >= -999999
 StartServiceRobots[14,23]: - 1.0000315560591351e+06 v[14,23]
   - start_service[14] + start_service[23] >= -999999
 StartServiceRobots[14,24]: - 1.0000315560591351e+06 v[14,24]
   - start_service[14] + start_service[24] >= -999999
 StartServiceRobots[14,25]: - 1.0000315560591351e+06 v[14,25]
   - start_service[14] + start_service[25] >= -999999
 StartServiceRobots[15,1]: - 1.0000086606162182e+06 v[15,1]
   + start_service[1] - start_service[15] >= -999999
 StartServiceRobots[15,2]: - 1.0000086606162182e+06 v[15,2]
   + start_service[2] - start_service[15] >= -999999
 StartServiceRobots[15,3]: - 1.0000562206776621e+06 v[15,3]
   + start_service[3] - start_service[15] >= -999999
 StartServiceRobots[15,4]: - 1.0000399960725273e+06 v[15,4]
   + start_service[4] - start_service[15] >= -999999
 StartServiceRobots[15,5]: - 999999.01 v[15,5] + start_service[5]
   - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 1.0000967563598768e+06 v[15,6]
   + start_service[6] - start_service[15] >= -999999
 StartServiceRobots[15,7]: - 1.0000166692290857e+06 v[15,7]
   + start_service[7] - start_service[15] >= -999999
 StartServiceRobots[15,8]: - 1.0000562206776621e+06 v[15,8]
   + start_service[8] - start_service[15] >= -999999
 StartServiceRobots[15,9]: - 1.0000399960725273e+06 v[15,9]
   + start_service[9] - start_service[15] >= -999999
 StartServiceRobots[15,10]: - 999999.01 v[15,10] + start_service[10]
   - start_service[15] >= -999999
 StartServiceRobots[15,11]: - 1.0000967563598768e+06 v[15,11]
   + start_service[11] - start_service[15] >= -999999
 StartServiceRobots[15,12]: - 1.0000166692290857e+06 v[15,12]
   + start_service[12] - start_service[15] >= -999999
 StartServiceRobots[15,13]: - 1.0000562206776621e+06 v[15,13]
   + start_service[13] - start_service[15] >= -999999
 StartServiceRobots[15,14]: - 1.0000399960725273e+06 v[15,14]
   + start_service[14] - start_service[15] >= -999999
 StartServiceRobots[15,15]: - 999999.01 v[15,15] >= -999999
 StartServiceRobots[15,16]: - 1.0000967563598768e+06 v[15,16]
   - start_service[15] + start_service[16] >= -999999
 StartServiceRobots[15,17]: - 1.0000166692290857e+06 v[15,17]
   - start_service[15] + start_service[17] >= -999999
 StartServiceRobots[15,18]: - 1.0000562206776621e+06 v[15,18]
   - start_service[15] + start_service[18] >= -999999
 StartServiceRobots[15,19]: - 1.0000399960725273e+06 v[15,19]
   - start_service[15] + start_service[19] >= -999999
 StartServiceRobots[15,20]: - 999999.01 v[15,20] - start_service[15]
   + start_service[20] >= -999999
 StartServiceRobots[15,21]: - 1.0000967563598768e+06 v[15,21]
   - start_service[15] + start_service[21] >= -999999
 StartServiceRobots[15,22]: - 1.0000166692290857e+06 v[15,22]
   - start_service[15] + start_service[22] >= -999999
 StartServiceRobots[15,23]: - 1.0000086606162182e+06 v[15,23]
   - start_service[15] + start_service[23] >= -999999
 StartServiceRobots[15,24]: - 1.0000086606162182e+06 v[15,24]
   - start_service[15] + start_service[24] >= -999999
 StartServiceRobots[15,25]: - 1.0000086606162182e+06 v[15,25]
   - start_service[15] + start_service[25] >= -999999
 StartServiceRobots[16,1]: - 1.0000880721248598e+06 v[16,1]
   + start_service[1] - start_service[16] >= -999999
 StartServiceRobots[16,2]: - 1.0000880721248598e+06 v[16,2]
   + start_service[2] - start_service[16] >= -999999
 StartServiceRobots[16,3]: - 1.0000706995536003e+06 v[16,3]
   + start_service[3] - start_service[16] >= -999999
 StartServiceRobots[16,4]: - 1.0000557694957714e+06 v[16,4]
   + start_service[4] - start_service[16] >= -999999
 StartServiceRobots[16,5]: - 1.0000967563598768e+06 v[16,5]
   + start_service[5] - start_service[16] >= -999999
 StartServiceRobots[16,6]: - 999999.01 v[16,6] + start_service[6]
   - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 1.0000805943163345e+06 v[16,7]
   + start_service[7] - start_service[16] >= -999999
 StartServiceRobots[16,8]: - 1.0000706995536003e+06 v[16,8]
   + start_service[8] - start_service[16] >= -999999
 StartServiceRobots[16,9]: - 1.0000557694957714e+06 v[16,9]
   + start_service[9] - start_service[16] >= -999999
 StartServiceRobots[16,10]: - 1.0000967563598768e+06 v[16,10]
   + start_service[10] - start_service[16] >= -999999
 StartServiceRobots[16,11]: - 999999.01 v[16,11] + start_service[11]
   - start_service[16] >= -999999
 StartServiceRobots[16,12]: - 1.0000805943163345e+06 v[16,12]
   + start_service[12] - start_service[16] >= -999999
 StartServiceRobots[16,13]: - 1.0000706995536003e+06 v[16,13]
   + start_service[13] - start_service[16] >= -999999
 StartServiceRobots[16,14]: - 1.0000557694957714e+06 v[16,14]
   + start_service[14] - start_service[16] >= -999999
 StartServiceRobots[16,15]: - 1.0000967563598768e+06 v[16,15]
   + start_service[15] - start_service[16] >= -999999
 StartServiceRobots[16,16]: - 999999.01 v[16,16] >= -999999
 StartServiceRobots[16,17]: - 1.0000805943163345e+06 v[16,17]
   - start_service[16] + start_service[17] >= -999999
 StartServiceRobots[16,18]: - 1.0000706995536003e+06 v[16,18]
   - start_service[16] + start_service[18] >= -999999
 StartServiceRobots[16,19]: - 1.0000557694957714e+06 v[16,19]
   - start_service[16] + start_service[19] >= -999999
 StartServiceRobots[16,20]: - 1.0000967563598768e+06 v[16,20]
   - start_service[16] + start_service[20] >= -999999
 StartServiceRobots[16,21]: - 999999.01 v[16,21] - start_service[16]
   + start_service[21] >= -999999
 StartServiceRobots[16,22]: - 1.0000805943163345e+06 v[16,22]
   - start_service[16] + start_service[22] >= -999999
 StartServiceRobots[16,23]: - 1.0000880721248598e+06 v[16,23]
   - start_service[16] + start_service[23] >= -999999
 StartServiceRobots[16,24]: - 1.0000880721248598e+06 v[16,24]
   - start_service[16] + start_service[24] >= -999999
 StartServiceRobots[16,25]: - 1.0000880721248598e+06 v[16,25]
   - start_service[16] + start_service[25] >= -999999
 StartServiceRobots[17,1]: - 1.0000120359099368e+06 v[17,1]
   + start_service[1] - start_service[17] >= -999999
 StartServiceRobots[17,2]: - 1.0000120359099368e+06 v[17,2]
   + start_service[2] - start_service[17] >= -999999
 StartServiceRobots[17,3]: - 1.0000408655636616e+06 v[17,3]
   + start_service[3] - start_service[17] >= -999999
 StartServiceRobots[17,4]: - 1.0000242562644748e+06 v[17,4]
   + start_service[4] - start_service[17] >= -999999
 StartServiceRobots[17,5]: - 1.0000166692290857e+06 v[17,5]
   + start_service[5] - start_service[17] >= -999999
 StartServiceRobots[17,6]: - 1.0000805943163345e+06 v[17,6]
   + start_service[6] - start_service[17] >= -999999
 StartServiceRobots[17,7]: - 999999.01 v[17,7] + start_service[7]
   - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 1.0000408655636616e+06 v[17,8]
   + start_service[8] - start_service[17] >= -999999
 StartServiceRobots[17,9]: - 1.0000242562644748e+06 v[17,9]
   + start_service[9] - start_service[17] >= -999999
 StartServiceRobots[17,10]: - 1.0000166692290857e+06 v[17,10]
   + start_service[10] - start_service[17] >= -999999
 StartServiceRobots[17,11]: - 1.0000805943163345e+06 v[17,11]
   + start_service[11] - start_service[17] >= -999999
 StartServiceRobots[17,12]: - 999999.01 v[17,12] + start_service[12]
   - start_service[17] >= -999999
 StartServiceRobots[17,13]: - 1.0000408655636616e+06 v[17,13]
   + start_service[13] - start_service[17] >= -999999
 StartServiceRobots[17,14]: - 1.0000242562644748e+06 v[17,14]
   + start_service[14] - start_service[17] >= -999999
 StartServiceRobots[17,15]: - 1.0000166692290857e+06 v[17,15]
   + start_service[15] - start_service[17] >= -999999
 StartServiceRobots[17,16]: - 1.0000805943163345e+06 v[17,16]
   + start_service[16] - start_service[17] >= -999999
 StartServiceRobots[17,17]: - 999999.01 v[17,17] >= -999999
 StartServiceRobots[17,18]: - 1.0000408655636616e+06 v[17,18]
   - start_service[17] + start_service[18] >= -999999
 StartServiceRobots[17,19]: - 1.0000242562644748e+06 v[17,19]
   - start_service[17] + start_service[19] >= -999999
 StartServiceRobots[17,20]: - 1.0000166692290857e+06 v[17,20]
   - start_service[17] + start_service[20] >= -999999
 StartServiceRobots[17,21]: - 1.0000805943163345e+06 v[17,21]
   - start_service[17] + start_service[21] >= -999999
 StartServiceRobots[17,22]: - 999999.01 v[17,22] - start_service[17]
   + start_service[22] >= -999999
 StartServiceRobots[17,23]: - 1.0000120359099368e+06 v[17,23]
   - start_service[17] + start_service[23] >= -999999
 StartServiceRobots[17,24]: - 1.0000120359099368e+06 v[17,24]
   - start_service[17] + start_service[24] >= -999999
 StartServiceRobots[17,25]: - 1.0000120359099368e+06 v[17,25]
   - start_service[17] + start_service[25] >= -999999
 StartServiceRobots[18,1]: - 1.0000538900827599e+06 v[18,1]
   + start_service[1] - start_service[18] >= -999999
 StartServiceRobots[18,2]: - 1.0000538900827599e+06 v[18,2]
   + start_service[2] - start_service[18] >= -999999
 StartServiceRobots[18,3]: - 999999.01 v[18,3] + start_service[3]
   - start_service[18] >= -999999
 StartServiceRobots[18,4]: - 1.0000399322151314e+06 v[18,4]
   + start_service[4] - start_service[18] >= -999999
 StartServiceRobots[18,5]: - 1.0000562206776621e+06 v[18,5]
   + start_service[5] - start_service[18] >= -999999
 StartServiceRobots[18,6]: - 1.0000706995536003e+06 v[18,6]
   + start_service[6] - start_service[18] >= -999999
 StartServiceRobots[18,7]: - 1.0000408655636616e+06 v[18,7]
   + start_service[7] - start_service[18] >= -999999
 StartServiceRobots[18,8]: - 999999.01 v[18,8] + start_service[8]
   - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 1.0000399322151314e+06 v[18,9]
   + start_service[9] - start_service[18] >= -999999
 StartServiceRobots[18,10]: - 1.0000562206776621e+06 v[18,10]
   + start_service[10] - start_service[18] >= -999999
 StartServiceRobots[18,11]: - 1.0000706995536003e+06 v[18,11]
   + start_service[11] - start_service[18] >= -999999
 StartServiceRobots[18,12]: - 1.0000408655636616e+06 v[18,12]
   + start_service[12] - start_service[18] >= -999999
 StartServiceRobots[18,13]: - 999999.01 v[18,13] + start_service[13]
   - start_service[18] >= -999999
 StartServiceRobots[18,14]: - 1.0000399322151314e+06 v[18,14]
   + start_service[14] - start_service[18] >= -999999
 StartServiceRobots[18,15]: - 1.0000562206776621e+06 v[18,15]
   + start_service[15] - start_service[18] >= -999999
 StartServiceRobots[18,16]: - 1.0000706995536003e+06 v[18,16]
   + start_service[16] - start_service[18] >= -999999
 StartServiceRobots[18,17]: - 1.0000408655636616e+06 v[18,17]
   + start_service[17] - start_service[18] >= -999999
 StartServiceRobots[18,18]: - 999999.01 v[18,18] >= -999999
 StartServiceRobots[18,19]: - 1.0000399322151314e+06 v[18,19]
   - start_service[18] + start_service[19] >= -999999
 StartServiceRobots[18,20]: - 1.0000562206776621e+06 v[18,20]
   - start_service[18] + start_service[20] >= -999999
 StartServiceRobots[18,21]: - 1.0000706995536003e+06 v[18,21]
   - start_service[18] + start_service[21] >= -999999
 StartServiceRobots[18,22]: - 1.0000408655636616e+06 v[18,22]
   - start_service[18] + start_service[22] >= -999999
 StartServiceRobots[18,23]: - 1.0000538900827599e+06 v[18,23]
   - start_service[18] + start_service[23] >= -999999
 StartServiceRobots[18,24]: - 1.0000538900827599e+06 v[18,24]
   - start_service[18] + start_service[24] >= -999999
 StartServiceRobots[18,25]: - 1.0000538900827599e+06 v[18,25]
   - start_service[18] + start_service[25] >= -999999
 StartServiceRobots[19,1]: - 1.0000315560591351e+06 v[19,1]
   + start_service[1] - start_service[19] >= -999999
 StartServiceRobots[19,2]: - 1.0000315560591351e+06 v[19,2]
   + start_service[2] - start_service[19] >= -999999
 StartServiceRobots[19,3]: - 1.0000399322151314e+06 v[19,3]
   + start_service[3] - start_service[19] >= -999999
 StartServiceRobots[19,4]: - 999999.01 v[19,4] + start_service[4]
   - start_service[19] >= -999999
 StartServiceRobots[19,5]: - 1.0000399960725273e+06 v[19,5]
   + start_service[5] - start_service[19] >= -999999
 StartServiceRobots[19,6]: - 1.0000557694957714e+06 v[19,6]
   + start_service[6] - start_service[19] >= -999999
 StartServiceRobots[19,7]: - 1.0000242562644748e+06 v[19,7]
   + start_service[7] - start_service[19] >= -999999
 StartServiceRobots[19,8]: - 1.0000399322151314e+06 v[19,8]
   + start_service[8] - start_service[19] >= -999999
 StartServiceRobots[19,9]: - 999999.01 v[19,9] + start_service[9]
   - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 1.0000399960725273e+06 v[19,10]
   + start_service[10] - start_service[19] >= -999999
 StartServiceRobots[19,11]: - 1.0000557694957714e+06 v[19,11]
   + start_service[11] - start_service[19] >= -999999
 StartServiceRobots[19,12]: - 1.0000242562644748e+06 v[19,12]
   + start_service[12] - start_service[19] >= -999999
 StartServiceRobots[19,13]: - 1.0000399322151314e+06 v[19,13]
   + start_service[13] - start_service[19] >= -999999
 StartServiceRobots[19,14]: - 999999.01 v[19,14] + start_service[14]
   - start_service[19] >= -999999
 StartServiceRobots[19,15]: - 1.0000399960725273e+06 v[19,15]
   + start_service[15] - start_service[19] >= -999999
 StartServiceRobots[19,16]: - 1.0000557694957714e+06 v[19,16]
   + start_service[16] - start_service[19] >= -999999
 StartServiceRobots[19,17]: - 1.0000242562644748e+06 v[19,17]
   + start_service[17] - start_service[19] >= -999999
 StartServiceRobots[19,18]: - 1.0000399322151314e+06 v[19,18]
   + start_service[18] - start_service[19] >= -999999
 StartServiceRobots[19,19]: - 999999.01 v[19,19] >= -999999
 StartServiceRobots[19,20]: - 1.0000399960725273e+06 v[19,20]
   - start_service[19] + start_service[20] >= -999999
 StartServiceRobots[19,21]: - 1.0000557694957714e+06 v[19,21]
   - start_service[19] + start_service[21] >= -999999
 StartServiceRobots[19,22]: - 1.0000242562644748e+06 v[19,22]
   - start_service[19] + start_service[22] >= -999999
 StartServiceRobots[19,23]: - 1.0000315560591351e+06 v[19,23]
   - start_service[19] + start_service[23] >= -999999
 StartServiceRobots[19,24]: - 1.0000315560591351e+06 v[19,24]
   - start_service[19] + start_service[24] >= -999999
 StartServiceRobots[19,25]: - 1.0000315560591351e+06 v[19,25]
   - start_service[19] + start_service[25] >= -999999
 StartServiceRobots[20,1]: - 1.0000086606162182e+06 v[20,1]
   + start_service[1] - start_service[20] >= -999999
 StartServiceRobots[20,2]: - 1.0000086606162182e+06 v[20,2]
   + start_service[2] - start_service[20] >= -999999
 StartServiceRobots[20,3]: - 1.0000562206776621e+06 v[20,3]
   + start_service[3] - start_service[20] >= -999999
 StartServiceRobots[20,4]: - 1.0000399960725273e+06 v[20,4]
   + start_service[4] - start_service[20] >= -999999
 StartServiceRobots[20,5]: - 999999.01 v[20,5] + start_service[5]
   - start_service[20] >= -999999
 StartServiceRobots[20,6]: - 1.0000967563598768e+06 v[20,6]
   + start_service[6] - start_service[20] >= -999999
 StartServiceRobots[20,7]: - 1.0000166692290857e+06 v[20,7]
   + start_service[7] - start_service[20] >= -999999
 StartServiceRobots[20,8]: - 1.0000562206776621e+06 v[20,8]
   + start_service[8] - start_service[20] >= -999999
 StartServiceRobots[20,9]: - 1.0000399960725273e+06 v[20,9]
   + start_service[9] - start_service[20] >= -999999
 StartServiceRobots[20,10]: - 999999.01 v[20,10] + start_service[10]
   - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 1.0000967563598768e+06 v[20,11]
   + start_service[11] - start_service[20] >= -999999
 StartServiceRobots[20,12]: - 1.0000166692290857e+06 v[20,12]
   + start_service[12] - start_service[20] >= -999999
 StartServiceRobots[20,13]: - 1.0000562206776621e+06 v[20,13]
   + start_service[13] - start_service[20] >= -999999
 StartServiceRobots[20,14]: - 1.0000399960725273e+06 v[20,14]
   + start_service[14] - start_service[20] >= -999999
 StartServiceRobots[20,15]: - 999999.01 v[20,15] + start_service[15]
   - start_service[20] >= -999999
 StartServiceRobots[20,16]: - 1.0000967563598768e+06 v[20,16]
   + start_service[16] - start_service[20] >= -999999
 StartServiceRobots[20,17]: - 1.0000166692290857e+06 v[20,17]
   + start_service[17] - start_service[20] >= -999999
 StartServiceRobots[20,18]: - 1.0000562206776621e+06 v[20,18]
   + start_service[18] - start_service[20] >= -999999
 StartServiceRobots[20,19]: - 1.0000399960725273e+06 v[20,19]
   + start_service[19] - start_service[20] >= -999999
 StartServiceRobots[20,20]: - 999999.01 v[20,20] >= -999999
 StartServiceRobots[20,21]: - 1.0000967563598768e+06 v[20,21]
   - start_service[20] + start_service[21] >= -999999
 StartServiceRobots[20,22]: - 1.0000166692290857e+06 v[20,22]
   - start_service[20] + start_service[22] >= -999999
 StartServiceRobots[20,23]: - 1.0000086606162182e+06 v[20,23]
   - start_service[20] + start_service[23] >= -999999
 StartServiceRobots[20,24]: - 1.0000086606162182e+06 v[20,24]
   - start_service[20] + start_service[24] >= -999999
 StartServiceRobots[20,25]: - 1.0000086606162182e+06 v[20,25]
   - start_service[20] + start_service[25] >= -999999
 StartServiceRobots[21,1]: - 1.0000880721248598e+06 v[21,1]
   + start_service[1] - start_service[21] >= -999999
 StartServiceRobots[21,2]: - 1.0000880721248598e+06 v[21,2]
   + start_service[2] - start_service[21] >= -999999
 StartServiceRobots[21,3]: - 1.0000706995536003e+06 v[21,3]
   + start_service[3] - start_service[21] >= -999999
 StartServiceRobots[21,4]: - 1.0000557694957714e+06 v[21,4]
   + start_service[4] - start_service[21] >= -999999
 StartServiceRobots[21,5]: - 1.0000967563598768e+06 v[21,5]
   + start_service[5] - start_service[21] >= -999999
 StartServiceRobots[21,6]: - 999999.01 v[21,6] + start_service[6]
   - start_service[21] >= -999999
 StartServiceRobots[21,7]: - 1.0000805943163345e+06 v[21,7]
   + start_service[7] - start_service[21] >= -999999
 StartServiceRobots[21,8]: - 1.0000706995536003e+06 v[21,8]
   + start_service[8] - start_service[21] >= -999999
 StartServiceRobots[21,9]: - 1.0000557694957714e+06 v[21,9]
   + start_service[9] - start_service[21] >= -999999
 StartServiceRobots[21,10]: - 1.0000967563598768e+06 v[21,10]
   + start_service[10] - start_service[21] >= -999999
 StartServiceRobots[21,11]: - 999999.01 v[21,11] + start_service[11]
   - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 1.0000805943163345e+06 v[21,12]
   + start_service[12] - start_service[21] >= -999999
 StartServiceRobots[21,13]: - 1.0000706995536003e+06 v[21,13]
   + start_service[13] - start_service[21] >= -999999
 StartServiceRobots[21,14]: - 1.0000557694957714e+06 v[21,14]
   + start_service[14] - start_service[21] >= -999999
 StartServiceRobots[21,15]: - 1.0000967563598768e+06 v[21,15]
   + start_service[15] - start_service[21] >= -999999
 StartServiceRobots[21,16]: - 999999.01 v[21,16] + start_service[16]
   - start_service[21] >= -999999
 StartServiceRobots[21,17]: - 1.0000805943163345e+06 v[21,17]
   + start_service[17] - start_service[21] >= -999999
 StartServiceRobots[21,18]: - 1.0000706995536003e+06 v[21,18]
   + start_service[18] - start_service[21] >= -999999
 StartServiceRobots[21,19]: - 1.0000557694957714e+06 v[21,19]
   + start_service[19] - start_service[21] >= -999999
 StartServiceRobots[21,20]: - 1.0000967563598768e+06 v[21,20]
   + start_service[20] - start_service[21] >= -999999
 StartServiceRobots[21,21]: - 999999.01 v[21,21] >= -999999
 StartServiceRobots[21,22]: - 1.0000805943163345e+06 v[21,22]
   - start_service[21] + start_service[22] >= -999999
 StartServiceRobots[21,23]: - 1.0000880721248598e+06 v[21,23]
   - start_service[21] + start_service[23] >= -999999
 StartServiceRobots[21,24]: - 1.0000880721248598e+06 v[21,24]
   - start_service[21] + start_service[24] >= -999999
 StartServiceRobots[21,25]: - 1.0000880721248598e+06 v[21,25]
   - start_service[21] + start_service[25] >= -999999
 StartServiceRobots[22,1]: - 1.0000120359099368e+06 v[22,1]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 1.0000120359099368e+06 v[22,2]
   + start_service[2] - start_service[22] >= -999999
 StartServiceRobots[22,3]: - 1.0000408655636616e+06 v[22,3]
   + start_service[3] - start_service[22] >= -999999
 StartServiceRobots[22,4]: - 1.0000242562644748e+06 v[22,4]
   + start_service[4] - start_service[22] >= -999999
 StartServiceRobots[22,5]: - 1.0000166692290857e+06 v[22,5]
   + start_service[5] - start_service[22] >= -999999
 StartServiceRobots[22,6]: - 1.0000805943163345e+06 v[22,6]
   + start_service[6] - start_service[22] >= -999999
 StartServiceRobots[22,7]: - 999999.01 v[22,7] + start_service[7]
   - start_service[22] >= -999999
 StartServiceRobots[22,8]: - 1.0000408655636616e+06 v[22,8]
   + start_service[8] - start_service[22] >= -999999
 StartServiceRobots[22,9]: - 1.0000242562644748e+06 v[22,9]
   + start_service[9] - start_service[22] >= -999999
 StartServiceRobots[22,10]: - 1.0000166692290857e+06 v[22,10]
   + start_service[10] - start_service[22] >= -999999
 StartServiceRobots[22,11]: - 1.0000805943163345e+06 v[22,11]
   + start_service[11] - start_service[22] >= -999999
 StartServiceRobots[22,12]: - 999999.01 v[22,12] + start_service[12]
   - start_service[22] >= -999999
 StartServiceRobots[22,13]: - 1.0000408655636616e+06 v[22,13]
   + start_service[13] - start_service[22] >= -999999
 StartServiceRobots[22,14]: - 1.0000242562644748e+06 v[22,14]
   + start_service[14] - start_service[22] >= -999999
 StartServiceRobots[22,15]: - 1.0000166692290857e+06 v[22,15]
   + start_service[15] - start_service[22] >= -999999
 StartServiceRobots[22,16]: - 1.0000805943163345e+06 v[22,16]
   + start_service[16] - start_service[22] >= -999999
 StartServiceRobots[22,17]: - 999999.01 v[22,17] + start_service[17]
   - start_service[22] >= -999999
 StartServiceRobots[22,18]: - 1.0000408655636616e+06 v[22,18]
   + start_service[18] - start_service[22] >= -999999
 StartServiceRobots[22,19]: - 1.0000242562644748e+06 v[22,19]
   + start_service[19] - start_service[22] >= -999999
 StartServiceRobots[22,20]: - 1.0000166692290857e+06 v[22,20]
   + start_service[20] - start_service[22] >= -999999
 StartServiceRobots[22,21]: - 1.0000805943163345e+06 v[22,21]
   + start_service[21] - start_service[22] >= -999999
 StartServiceRobots[22,22]: - 999999.01 v[22,22] >= -999999
 StartServiceRobots[22,23]: - 1.0000120359099368e+06 v[22,23]
   - start_service[22] + start_service[23] >= -999999
 StartServiceRobots[22,24]: - 1.0000120359099368e+06 v[22,24]
   - start_service[22] + start_service[24] >= -999999
 StartServiceRobots[22,25]: - 1.0000120359099368e+06 v[22,25]
   - start_service[22] + start_service[25] >= -999999
 StartServiceRobots[23,1]: - 999999.01 v[23,1] + start_service[1]
   - start_service[23] >= -999999
 StartServiceRobots[23,2]: - 999999.01 v[23,2] + start_service[2]
   - start_service[23] >= -999999
 StartServiceRobots[23,3]: - 1.0000538900827599e+06 v[23,3]
   + start_service[3] - start_service[23] >= -999999
 StartServiceRobots[23,4]: - 1.0000315560591351e+06 v[23,4]
   + start_service[4] - start_service[23] >= -999999
 StartServiceRobots[23,5]: - 1.0000086606162182e+06 v[23,5]
   + start_service[5] - start_service[23] >= -999999
 StartServiceRobots[23,6]: - 1.0000880721248598e+06 v[23,6]
   + start_service[6] - start_service[23] >= -999999
 StartServiceRobots[23,7]: - 1.0000120359099368e+06 v[23,7]
   + start_service[7] - start_service[23] >= -999999
 StartServiceRobots[23,8]: - 1.0000538900827599e+06 v[23,8]
   + start_service[8] - start_service[23] >= -999999
 StartServiceRobots[23,9]: - 1.0000315560591351e+06 v[23,9]
   + start_service[9] - start_service[23] >= -999999
 StartServiceRobots[23,10]: - 1.0000086606162182e+06 v[23,10]
   + start_service[10] - start_service[23] >= -999999
 StartServiceRobots[23,11]: - 1.0000880721248598e+06 v[23,11]
   + start_service[11] - start_service[23] >= -999999
 StartServiceRobots[23,12]: - 1.0000120359099368e+06 v[23,12]
   + start_service[12] - start_service[23] >= -999999
 StartServiceRobots[23,13]: - 1.0000538900827599e+06 v[23,13]
   + start_service[13] - start_service[23] >= -999999
 StartServiceRobots[23,14]: - 1.0000315560591351e+06 v[23,14]
   + start_service[14] - start_service[23] >= -999999
 StartServiceRobots[23,15]: - 1.0000086606162182e+06 v[23,15]
   + start_service[15] - start_service[23] >= -999999
 StartServiceRobots[23,16]: - 1.0000880721248598e+06 v[23,16]
   + start_service[16] - start_service[23] >= -999999
 StartServiceRobots[23,17]: - 1.0000120359099368e+06 v[23,17]
   + start_service[17] - start_service[23] >= -999999
 StartServiceRobots[23,18]: - 1.0000538900827599e+06 v[23,18]
   + start_service[18] - start_service[23] >= -999999
 StartServiceRobots[23,19]: - 1.0000315560591351e+06 v[23,19]
   + start_service[19] - start_service[23] >= -999999
 StartServiceRobots[23,20]: - 1.0000086606162182e+06 v[23,20]
   + start_service[20] - start_service[23] >= -999999
 StartServiceRobots[23,21]: - 1.0000880721248598e+06 v[23,21]
   + start_service[21] - start_service[23] >= -999999
 StartServiceRobots[23,22]: - 1.0000120359099368e+06 v[23,22]
   + start_service[22] - start_service[23] >= -999999
 StartServiceRobots[23,23]: - 999999.01 v[23,23] >= -999999
 StartServiceRobots[23,24]: - 999999.01 v[23,24] - start_service[23]
   + start_service[24] >= -999999
 StartServiceRobots[23,25]: - 999999.01 v[23,25] - start_service[23]
   + start_service[25] >= -999999
 StartServiceRobots[24,1]: - 999999.01 v[24,1] + start_service[1]
   - start_service[24] >= -999999
 StartServiceRobots[24,2]: - 999999.01 v[24,2] + start_service[2]
   - start_service[24] >= -999999
 StartServiceRobots[24,3]: - 1.0000538900827599e+06 v[24,3]
   + start_service[3] - start_service[24] >= -999999
 StartServiceRobots[24,4]: - 1.0000315560591351e+06 v[24,4]
   + start_service[4] - start_service[24] >= -999999
 StartServiceRobots[24,5]: - 1.0000086606162182e+06 v[24,5]
   + start_service[5] - start_service[24] >= -999999
 StartServiceRobots[24,6]: - 1.0000880721248598e+06 v[24,6]
   + start_service[6] - start_service[24] >= -999999
 StartServiceRobots[24,7]: - 1.0000120359099368e+06 v[24,7]
   + start_service[7] - start_service[24] >= -999999
 StartServiceRobots[24,8]: - 1.0000538900827599e+06 v[24,8]
   + start_service[8] - start_service[24] >= -999999
 StartServiceRobots[24,9]: - 1.0000315560591351e+06 v[24,9]
   + start_service[9] - start_service[24] >= -999999
 StartServiceRobots[24,10]: - 1.0000086606162182e+06 v[24,10]
   + start_service[10] - start_service[24] >= -999999
 StartServiceRobots[24,11]: - 1.0000880721248598e+06 v[24,11]
   + start_service[11] - start_service[24] >= -999999
 StartServiceRobots[24,12]: - 1.0000120359099368e+06 v[24,12]
   + start_service[12] - start_service[24] >= -999999
 StartServiceRobots[24,13]: - 1.0000538900827599e+06 v[24,13]
   + start_service[13] - start_service[24] >= -999999
 StartServiceRobots[24,14]: - 1.0000315560591351e+06 v[24,14]
   + start_service[14] - start_service[24] >= -999999
 StartServiceRobots[24,15]: - 1.0000086606162182e+06 v[24,15]
   + start_service[15] - start_service[24] >= -999999
 StartServiceRobots[24,16]: - 1.0000880721248598e+06 v[24,16]
   + start_service[16] - start_service[24] >= -999999
 StartServiceRobots[24,17]: - 1.0000120359099368e+06 v[24,17]
   + start_service[17] - start_service[24] >= -999999
 StartServiceRobots[24,18]: - 1.0000538900827599e+06 v[24,18]
   + start_service[18] - start_service[24] >= -999999
 StartServiceRobots[24,19]: - 1.0000315560591351e+06 v[24,19]
   + start_service[19] - start_service[24] >= -999999
 StartServiceRobots[24,20]: - 1.0000086606162182e+06 v[24,20]
   + start_service[20] - start_service[24] >= -999999
 StartServiceRobots[24,21]: - 1.0000880721248598e+06 v[24,21]
   + start_service[21] - start_service[24] >= -999999
 StartServiceRobots[24,22]: - 1.0000120359099368e+06 v[24,22]
   + start_service[22] - start_service[24] >= -999999
 StartServiceRobots[24,23]: - 999999.01 v[24,23] + start_service[23]
   - start_service[24] >= -999999
 StartServiceRobots[24,24]: - 999999.01 v[24,24] >= -999999
 StartServiceRobots[24,25]: - 999999.01 v[24,25] - start_service[24]
   + start_service[25] >= -999999
 RobotsEndService[3,13]: - 999999 v[3,13] - service_time[3,13,1]
   - service_time[3,13,2] - start_service[3] + start_service[13]
   >= -999999
 RobotsEndService[3,18]: - 999999 v[3,18] - service_time[3,18,1]
   - service_time[3,18,2] - start_service[3] + start_service[18]
   >= -999999
 RobotsEndService[4,14]: - 999999 v[4,14] - service_time[4,14,1]
   - service_time[4,14,2] - start_service[4] + start_service[14]
   >= -999999
 RobotsEndService[4,19]: - 999999 v[4,19] - service_time[4,19,1]
   - service_time[4,19,2] - start_service[4] + start_service[19]
   >= -999999
 RobotsEndService[5,15]: - 999999 v[5,15] - service_time[5,15,1]
   - service_time[5,15,2] - start_service[5] + start_service[15]
   >= -999999
 RobotsEndService[5,20]: - 999999 v[5,20] - service_time[5,20,1]
   - service_time[5,20,2] - start_service[5] + start_service[20]
   >= -999999
 RobotsEndService[6,16]: - 999999 v[6,16] - service_time[6,16,1]
   - service_time[6,16,2] - start_service[6] + start_service[16]
   >= -999999
 RobotsEndService[6,21]: - 999999 v[6,21] - service_time[6,21,1]
   - service_time[6,21,2] - start_service[6] + start_service[21]
   >= -999999
 RobotsEndService[7,17]: - 999999 v[7,17] - service_time[7,17,1]
   - service_time[7,17,2] - start_service[7] + start_service[17]
   >= -999999
 RobotsEndService[7,22]: - 999999 v[7,22] - service_time[7,22,1]
   - service_time[7,22,2] - start_service[7] + start_service[22]
   >= -999999
 RobotsEndService[8,13]: - 999999 v[8,13] - service_time[8,13,1]
   - service_time[8,13,2] - start_service[8] + start_service[13]
   >= -999999
 RobotsEndService[8,18]: - 999999 v[8,18] - service_time[8,18,1]
   - service_time[8,18,2] - start_service[8] + start_service[18]
   >= -999999
 RobotsEndService[9,14]: - 999999 v[9,14] - service_time[9,14,1]
   - service_time[9,14,2] - start_service[9] + start_service[14]
   >= -999999
 RobotsEndService[9,19]: - 999999 v[9,19] - service_time[9,19,1]
   - service_time[9,19,2] - start_service[9] + start_service[19]
   >= -999999
 RobotsEndService[10,15]: - 999999 v[10,15] - service_time[10,15,1]
   - service_time[10,15,2] - start_service[10] + start_service[15]
   >= -999999
 RobotsEndService[10,20]: - 999999 v[10,20] - service_time[10,20,1]
   - service_time[10,20,2] - start_service[10] + start_service[20]
   >= -999999
 RobotsEndService[11,16]: - 999999 v[11,16] - service_time[11,16,1]
   - service_time[11,16,2] - start_service[11] + start_service[16]
   >= -999999
 RobotsEndService[11,21]: - 999999 v[11,21] - service_time[11,21,1]
   - service_time[11,21,2] - start_service[11] + start_service[21]
   >= -999999
 RobotsEndService[12,17]: - 999999 v[12,17] - service_time[12,17,1]
   - service_time[12,17,2] - start_service[12] + start_service[17]
   >= -999999
 RobotsEndService[12,22]: - 999999 v[12,22] - service_time[12,22,1]
   - service_time[12,22,2] - start_service[12] + start_service[22]
   >= -999999
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
 VehicleRobotSynchronization1[0,24]: - 2 x[0,24] + w[0,24] <= 0
 VehicleRobotSynchronization1[0,25]: - 2 x[0,25] + w[0,25] <= 0
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
 VehicleRobotSynchronization1[1,24]: - 2 x[1,24] + w[1,24] <= 0
 VehicleRobotSynchronization1[1,25]: - 2 x[1,25] + w[1,25] <= 0
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
 VehicleRobotSynchronization1[2,12]: - 2 x[2,12] + w[2,12] <= 0
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
 VehicleRobotSynchronization1[2,24]: - 2 x[2,24] + w[2,24] <= 0
 VehicleRobotSynchronization1[2,25]: - 2 x[2,25] + w[2,25] <= 0
 VehicleRobotSynchronization1[3,0]: - 2 x[3,0] + w[3,0] <= 0
 VehicleRobotSynchronization1[3,1]: - 2 x[3,1] + w[3,1] <= 0
 VehicleRobotSynchronization1[3,2]: - 2 x[3,2] + w[3,2] <= 0
 VehicleRobotSynchronization1[3,4]: - 2 x[3,4] + w[3,4] <= 0
 VehicleRobotSynchronization1[3,5]: - 2 x[3,5] + w[3,5] <= 0
 VehicleRobotSynchronization1[3,6]: - 2 x[3,6] + w[3,6] <= 0
 VehicleRobotSynchronization1[3,7]: - 2 x[3,7] + w[3,7] <= 0
 VehicleRobotSynchronization1[3,9]: - 2 x[3,9] + w[3,9] <= 0
 VehicleRobotSynchronization1[3,10]: - 2 x[3,10] + w[3,10] <= 0
 VehicleRobotSynchronization1[3,11]: - 2 x[3,11] + w[3,11] <= 0
 VehicleRobotSynchronization1[3,12]: - 2 x[3,12] + w[3,12] <= 0
 VehicleRobotSynchronization1[3,14]: - 2 x[3,14] + w[3,14] <= 0
 VehicleRobotSynchronization1[3,15]: - 2 x[3,15] + w[3,15] <= 0
 VehicleRobotSynchronization1[3,16]: - 2 x[3,16] + w[3,16] <= 0
 VehicleRobotSynchronization1[3,17]: - 2 x[3,17] + w[3,17] <= 0
 VehicleRobotSynchronization1[3,19]: - 2 x[3,19] + w[3,19] <= 0
 VehicleRobotSynchronization1[3,20]: - 2 x[3,20] + w[3,20] <= 0
 VehicleRobotSynchronization1[3,21]: - 2 x[3,21] + w[3,21] <= 0
 VehicleRobotSynchronization1[3,22]: - 2 x[3,22] + w[3,22] <= 0
 VehicleRobotSynchronization1[3,23]: - 2 x[3,23] + w[3,23] <= 0
 VehicleRobotSynchronization1[3,24]: - 2 x[3,24] + w[3,24] <= 0
 VehicleRobotSynchronization1[3,25]: - 2 x[3,25] + w[3,25] <= 0
 VehicleRobotSynchronization1[4,0]: - 2 x[4,0] + w[4,0] <= 0
 VehicleRobotSynchronization1[4,1]: - 2 x[4,1] + w[4,1] <= 0
 VehicleRobotSynchronization1[4,2]: - 2 x[4,2] + w[4,2] <= 0
 VehicleRobotSynchronization1[4,3]: - 2 x[4,3] + w[4,3] <= 0
 VehicleRobotSynchronization1[4,5]: - 2 x[4,5] + w[4,5] <= 0
 VehicleRobotSynchronization1[4,6]: - 2 x[4,6] + w[4,6] <= 0
 VehicleRobotSynchronization1[4,7]: - 2 x[4,7] + w[4,7] <= 0
 VehicleRobotSynchronization1[4,8]: - 2 x[4,8] + w[4,8] <= 0
 VehicleRobotSynchronization1[4,10]: - 2 x[4,10] + w[4,10] <= 0
 VehicleRobotSynchronization1[4,11]: - 2 x[4,11] + w[4,11] <= 0
 VehicleRobotSynchronization1[4,12]: - 2 x[4,12] + w[4,12] <= 0
 VehicleRobotSynchronization1[4,13]: - 2 x[4,13] + w[4,13] <= 0
 VehicleRobotSynchronization1[4,15]: - 2 x[4,15] + w[4,15] <= 0
 VehicleRobotSynchronization1[4,16]: - 2 x[4,16] + w[4,16] <= 0
 VehicleRobotSynchronization1[4,17]: - 2 x[4,17] + w[4,17] <= 0
 VehicleRobotSynchronization1[4,18]: - 2 x[4,18] + w[4,18] <= 0
 VehicleRobotSynchronization1[4,20]: - 2 x[4,20] + w[4,20] <= 0
 VehicleRobotSynchronization1[4,21]: - 2 x[4,21] + w[4,21] <= 0
 VehicleRobotSynchronization1[4,22]: - 2 x[4,22] + w[4,22] <= 0
 VehicleRobotSynchronization1[4,23]: - 2 x[4,23] + w[4,23] <= 0
 VehicleRobotSynchronization1[4,24]: - 2 x[4,24] + w[4,24] <= 0
 VehicleRobotSynchronization1[4,25]: - 2 x[4,25] + w[4,25] <= 0
 VehicleRobotSynchronization1[5,0]: - 2 x[5,0] + w[5,0] <= 0
 VehicleRobotSynchronization1[5,1]: - 2 x[5,1] + w[5,1] <= 0
 VehicleRobotSynchronization1[5,2]: - 2 x[5,2] + w[5,2] <= 0
 VehicleRobotSynchronization1[5,3]: - 2 x[5,3] + w[5,3] <= 0
 VehicleRobotSynchronization1[5,4]: - 2 x[5,4] + w[5,4] <= 0
 VehicleRobotSynchronization1[5,6]: - 2 x[5,6] + w[5,6] <= 0
 VehicleRobotSynchronization1[5,7]: - 2 x[5,7] + w[5,7] <= 0
 VehicleRobotSynchronization1[5,8]: - 2 x[5,8] + w[5,8] <= 0
 VehicleRobotSynchronization1[5,9]: - 2 x[5,9] + w[5,9] <= 0
 VehicleRobotSynchronization1[5,11]: - 2 x[5,11] + w[5,11] <= 0
 VehicleRobotSynchronization1[5,12]: - 2 x[5,12] + w[5,12] <= 0
 VehicleRobotSynchronization1[5,13]: - 2 x[5,13] + w[5,13] <= 0
 VehicleRobotSynchronization1[5,14]: - 2 x[5,14] + w[5,14] <= 0
 VehicleRobotSynchronization1[5,16]: - 2 x[5,16] + w[5,16] <= 0
 VehicleRobotSynchronization1[5,17]: - 2 x[5,17] + w[5,17] <= 0
 VehicleRobotSynchronization1[5,18]: - 2 x[5,18] + w[5,18] <= 0
 VehicleRobotSynchronization1[5,19]: - 2 x[5,19] + w[5,19] <= 0
 VehicleRobotSynchronization1[5,21]: - 2 x[5,21] + w[5,21] <= 0
 VehicleRobotSynchronization1[5,22]: - 2 x[5,22] + w[5,22] <= 0
 VehicleRobotSynchronization1[5,23]: - 2 x[5,23] + w[5,23] <= 0
 VehicleRobotSynchronization1[5,24]: - 2 x[5,24] + w[5,24] <= 0
 VehicleRobotSynchronization1[5,25]: - 2 x[5,25] + w[5,25] <= 0
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
 VehicleRobotSynchronization1[6,12]: - 2 x[6,12] + w[6,12] <= 0
 VehicleRobotSynchronization1[6,13]: - 2 x[6,13] + w[6,13] <= 0
 VehicleRobotSynchronization1[6,14]: - 2 x[6,14] + w[6,14] <= 0
 VehicleRobotSynchronization1[6,15]: - 2 x[6,15] + w[6,15] <= 0
 VehicleRobotSynchronization1[6,17]: - 2 x[6,17] + w[6,17] <= 0
 VehicleRobotSynchronization1[6,18]: - 2 x[6,18] + w[6,18] <= 0
 VehicleRobotSynchronization1[6,19]: - 2 x[6,19] + w[6,19] <= 0
 VehicleRobotSynchronization1[6,20]: - 2 x[6,20] + w[6,20] <= 0
 VehicleRobotSynchronization1[6,22]: - 2 x[6,22] + w[6,22] <= 0
 VehicleRobotSynchronization1[6,23]: - 2 x[6,23] + w[6,23] <= 0
 VehicleRobotSynchronization1[6,24]: - 2 x[6,24] + w[6,24] <= 0
 VehicleRobotSynchronization1[6,25]: - 2 x[6,25] + w[6,25] <= 0
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
 VehicleRobotSynchronization1[7,13]: - 2 x[7,13] + w[7,13] <= 0
 VehicleRobotSynchronization1[7,14]: - 2 x[7,14] + w[7,14] <= 0
 VehicleRobotSynchronization1[7,15]: - 2 x[7,15] + w[7,15] <= 0
 VehicleRobotSynchronization1[7,16]: - 2 x[7,16] + w[7,16] <= 0
 VehicleRobotSynchronization1[7,18]: - 2 x[7,18] + w[7,18] <= 0
 VehicleRobotSynchronization1[7,19]: - 2 x[7,19] + w[7,19] <= 0
 VehicleRobotSynchronization1[7,20]: - 2 x[7,20] + w[7,20] <= 0
 VehicleRobotSynchronization1[7,21]: - 2 x[7,21] + w[7,21] <= 0
 VehicleRobotSynchronization1[7,23]: - 2 x[7,23] + w[7,23] <= 0
 VehicleRobotSynchronization1[7,24]: - 2 x[7,24] + w[7,24] <= 0
 VehicleRobotSynchronization1[7,25]: - 2 x[7,25] + w[7,25] <= 0
 VehicleRobotSynchronization1[8,0]: - 2 x[8,0] + w[8,0] <= 0
 VehicleRobotSynchronization1[8,1]: - 2 x[8,1] + w[8,1] <= 0
 VehicleRobotSynchronization1[8,2]: - 2 x[8,2] + w[8,2] <= 0
 VehicleRobotSynchronization1[8,4]: - 2 x[8,4] + w[8,4] <= 0
 VehicleRobotSynchronization1[8,5]: - 2 x[8,5] + w[8,5] <= 0
 VehicleRobotSynchronization1[8,6]: - 2 x[8,6] + w[8,6] <= 0
 VehicleRobotSynchronization1[8,7]: - 2 x[8,7] + w[8,7] <= 0
 VehicleRobotSynchronization1[8,9]: - 2 x[8,9] + w[8,9] <= 0
 VehicleRobotSynchronization1[8,10]: - 2 x[8,10] + w[8,10] <= 0
 VehicleRobotSynchronization1[8,11]: - 2 x[8,11] + w[8,11] <= 0
 VehicleRobotSynchronization1[8,12]: - 2 x[8,12] + w[8,12] <= 0
 VehicleRobotSynchronization1[8,14]: - 2 x[8,14] + w[8,14] <= 0
 VehicleRobotSynchronization1[8,15]: - 2 x[8,15] + w[8,15] <= 0
 VehicleRobotSynchronization1[8,16]: - 2 x[8,16] + w[8,16] <= 0
 VehicleRobotSynchronization1[8,17]: - 2 x[8,17] + w[8,17] <= 0
 VehicleRobotSynchronization1[8,19]: - 2 x[8,19] + w[8,19] <= 0
 VehicleRobotSynchronization1[8,20]: - 2 x[8,20] + w[8,20] <= 0
 VehicleRobotSynchronization1[8,21]: - 2 x[8,21] + w[8,21] <= 0
 VehicleRobotSynchronization1[8,22]: - 2 x[8,22] + w[8,22] <= 0
 VehicleRobotSynchronization1[8,23]: - 2 x[8,23] + w[8,23] <= 0
 VehicleRobotSynchronization1[8,24]: - 2 x[8,24] + w[8,24] <= 0
 VehicleRobotSynchronization1[8,25]: - 2 x[8,25] + w[8,25] <= 0
 VehicleRobotSynchronization1[9,0]: - 2 x[9,0] + w[9,0] <= 0
 VehicleRobotSynchronization1[9,1]: - 2 x[9,1] + w[9,1] <= 0
 VehicleRobotSynchronization1[9,2]: - 2 x[9,2] + w[9,2] <= 0
 VehicleRobotSynchronization1[9,3]: - 2 x[9,3] + w[9,3] <= 0
 VehicleRobotSynchronization1[9,5]: - 2 x[9,5] + w[9,5] <= 0
 VehicleRobotSynchronization1[9,6]: - 2 x[9,6] + w[9,6] <= 0
 VehicleRobotSynchronization1[9,7]: - 2 x[9,7] + w[9,7] <= 0
 VehicleRobotSynchronization1[9,8]: - 2 x[9,8] + w[9,8] <= 0
 VehicleRobotSynchronization1[9,10]: - 2 x[9,10] + w[9,10] <= 0
 VehicleRobotSynchronization1[9,11]: - 2 x[9,11] + w[9,11] <= 0
 VehicleRobotSynchronization1[9,12]: - 2 x[9,12] + w[9,12] <= 0
 VehicleRobotSynchronization1[9,13]: - 2 x[9,13] + w[9,13] <= 0
 VehicleRobotSynchronization1[9,15]: - 2 x[9,15] + w[9,15] <= 0
 VehicleRobotSynchronization1[9,16]: - 2 x[9,16] + w[9,16] <= 0
 VehicleRobotSynchronization1[9,17]: - 2 x[9,17] + w[9,17] <= 0
 VehicleRobotSynchronization1[9,18]: - 2 x[9,18] + w[9,18] <= 0
 VehicleRobotSynchronization1[9,20]: - 2 x[9,20] + w[9,20] <= 0
 VehicleRobotSynchronization1[9,21]: - 2 x[9,21] + w[9,21] <= 0
 VehicleRobotSynchronization1[9,22]: - 2 x[9,22] + w[9,22] <= 0
 VehicleRobotSynchronization1[9,23]: - 2 x[9,23] + w[9,23] <= 0
 VehicleRobotSynchronization1[9,24]: - 2 x[9,24] + w[9,24] <= 0
 VehicleRobotSynchronization1[9,25]: - 2 x[9,25] + w[9,25] <= 0
 VehicleRobotSynchronization1[10,0]: - 2 x[10,0] + w[10,0] <= 0
 VehicleRobotSynchronization1[10,1]: - 2 x[10,1] + w[10,1] <= 0
 VehicleRobotSynchronization1[10,2]: - 2 x[10,2] + w[10,2] <= 0
 VehicleRobotSynchronization1[10,3]: - 2 x[10,3] + w[10,3] <= 0
 VehicleRobotSynchronization1[10,4]: - 2 x[10,4] + w[10,4] <= 0
 VehicleRobotSynchronization1[10,6]: - 2 x[10,6] + w[10,6] <= 0
 VehicleRobotSynchronization1[10,7]: - 2 x[10,7] + w[10,7] <= 0
 VehicleRobotSynchronization1[10,8]: - 2 x[10,8] + w[10,8] <= 0
 VehicleRobotSynchronization1[10,9]: - 2 x[10,9] + w[10,9] <= 0
 VehicleRobotSynchronization1[10,11]: - 2 x[10,11] + w[10,11] <= 0
 VehicleRobotSynchronization1[10,12]: - 2 x[10,12] + w[10,12] <= 0
 VehicleRobotSynchronization1[10,13]: - 2 x[10,13] + w[10,13] <= 0
 VehicleRobotSynchronization1[10,14]: - 2 x[10,14] + w[10,14] <= 0
 VehicleRobotSynchronization1[10,16]: - 2 x[10,16] + w[10,16] <= 0
 VehicleRobotSynchronization1[10,17]: - 2 x[10,17] + w[10,17] <= 0
 VehicleRobotSynchronization1[10,18]: - 2 x[10,18] + w[10,18] <= 0
 VehicleRobotSynchronization1[10,19]: - 2 x[10,19] + w[10,19] <= 0
 VehicleRobotSynchronization1[10,21]: - 2 x[10,21] + w[10,21] <= 0
 VehicleRobotSynchronization1[10,22]: - 2 x[10,22] + w[10,22] <= 0
 VehicleRobotSynchronization1[10,23]: - 2 x[10,23] + w[10,23] <= 0
 VehicleRobotSynchronization1[10,24]: - 2 x[10,24] + w[10,24] <= 0
 VehicleRobotSynchronization1[10,25]: - 2 x[10,25] + w[10,25] <= 0
 VehicleRobotSynchronization1[11,0]: - 2 x[11,0] + w[11,0] <= 0
 VehicleRobotSynchronization1[11,1]: - 2 x[11,1] + w[11,1] <= 0
 VehicleRobotSynchronization1[11,2]: - 2 x[11,2] + w[11,2] <= 0
 VehicleRobotSynchronization1[11,3]: - 2 x[11,3] + w[11,3] <= 0
 VehicleRobotSynchronization1[11,4]: - 2 x[11,4] + w[11,4] <= 0
 VehicleRobotSynchronization1[11,5]: - 2 x[11,5] + w[11,5] <= 0
 VehicleRobotSynchronization1[11,7]: - 2 x[11,7] + w[11,7] <= 0
 VehicleRobotSynchronization1[11,8]: - 2 x[11,8] + w[11,8] <= 0
 VehicleRobotSynchronization1[11,9]: - 2 x[11,9] + w[11,9] <= 0
 VehicleRobotSynchronization1[11,10]: - 2 x[11,10] + w[11,10] <= 0
 VehicleRobotSynchronization1[11,12]: - 2 x[11,12] + w[11,12] <= 0
 VehicleRobotSynchronization1[11,13]: - 2 x[11,13] + w[11,13] <= 0
 VehicleRobotSynchronization1[11,14]: - 2 x[11,14] + w[11,14] <= 0
 VehicleRobotSynchronization1[11,15]: - 2 x[11,15] + w[11,15] <= 0
 VehicleRobotSynchronization1[11,17]: - 2 x[11,17] + w[11,17] <= 0
 VehicleRobotSynchronization1[11,18]: - 2 x[11,18] + w[11,18] <= 0
 VehicleRobotSynchronization1[11,19]: - 2 x[11,19] + w[11,19] <= 0
 VehicleRobotSynchronization1[11,20]: - 2 x[11,20] + w[11,20] <= 0
 VehicleRobotSynchronization1[11,22]: - 2 x[11,22] + w[11,22] <= 0
 VehicleRobotSynchronization1[11,23]: - 2 x[11,23] + w[11,23] <= 0
 VehicleRobotSynchronization1[11,24]: - 2 x[11,24] + w[11,24] <= 0
 VehicleRobotSynchronization1[11,25]: - 2 x[11,25] + w[11,25] <= 0
 VehicleRobotSynchronization1[12,0]: - 2 x[12,0] + w[12,0] <= 0
 VehicleRobotSynchronization1[12,1]: - 2 x[12,1] + w[12,1] <= 0
 VehicleRobotSynchronization1[12,2]: - 2 x[12,2] + w[12,2] <= 0
 VehicleRobotSynchronization1[12,3]: - 2 x[12,3] + w[12,3] <= 0
 VehicleRobotSynchronization1[12,4]: - 2 x[12,4] + w[12,4] <= 0
 VehicleRobotSynchronization1[12,5]: - 2 x[12,5] + w[12,5] <= 0
 VehicleRobotSynchronization1[12,6]: - 2 x[12,6] + w[12,6] <= 0
 VehicleRobotSynchronization1[12,8]: - 2 x[12,8] + w[12,8] <= 0
 VehicleRobotSynchronization1[12,9]: - 2 x[12,9] + w[12,9] <= 0
 VehicleRobotSynchronization1[12,10]: - 2 x[12,10] + w[12,10] <= 0
 VehicleRobotSynchronization1[12,11]: - 2 x[12,11] + w[12,11] <= 0
 VehicleRobotSynchronization1[12,13]: - 2 x[12,13] + w[12,13] <= 0
 VehicleRobotSynchronization1[12,14]: - 2 x[12,14] + w[12,14] <= 0
 VehicleRobotSynchronization1[12,15]: - 2 x[12,15] + w[12,15] <= 0
 VehicleRobotSynchronization1[12,16]: - 2 x[12,16] + w[12,16] <= 0
 VehicleRobotSynchronization1[12,18]: - 2 x[12,18] + w[12,18] <= 0
 VehicleRobotSynchronization1[12,19]: - 2 x[12,19] + w[12,19] <= 0
 VehicleRobotSynchronization1[12,20]: - 2 x[12,20] + w[12,20] <= 0
 VehicleRobotSynchronization1[12,21]: - 2 x[12,21] + w[12,21] <= 0
 VehicleRobotSynchronization1[12,23]: - 2 x[12,23] + w[12,23] <= 0
 VehicleRobotSynchronization1[12,24]: - 2 x[12,24] + w[12,24] <= 0
 VehicleRobotSynchronization1[12,25]: - 2 x[12,25] + w[12,25] <= 0
 VehicleRobotSynchronization1[13,0]: - 2 x[13,0] + w[13,0] <= 0
 VehicleRobotSynchronization1[13,1]: - 2 x[13,1] + w[13,1] <= 0
 VehicleRobotSynchronization1[13,2]: - 2 x[13,2] + w[13,2] <= 0
 VehicleRobotSynchronization1[13,4]: - 2 x[13,4] + w[13,4] <= 0
 VehicleRobotSynchronization1[13,5]: - 2 x[13,5] + w[13,5] <= 0
 VehicleRobotSynchronization1[13,6]: - 2 x[13,6] + w[13,6] <= 0
 VehicleRobotSynchronization1[13,7]: - 2 x[13,7] + w[13,7] <= 0
 VehicleRobotSynchronization1[13,9]: - 2 x[13,9] + w[13,9] <= 0
 VehicleRobotSynchronization1[13,10]: - 2 x[13,10] + w[13,10] <= 0
 VehicleRobotSynchronization1[13,11]: - 2 x[13,11] + w[13,11] <= 0
 VehicleRobotSynchronization1[13,12]: - 2 x[13,12] + w[13,12] <= 0
 VehicleRobotSynchronization1[13,14]: - 2 x[13,14] + w[13,14] <= 0
 VehicleRobotSynchronization1[13,15]: - 2 x[13,15] + w[13,15] <= 0
 VehicleRobotSynchronization1[13,16]: - 2 x[13,16] + w[13,16] <= 0
 VehicleRobotSynchronization1[13,17]: - 2 x[13,17] + w[13,17] <= 0
 VehicleRobotSynchronization1[13,19]: - 2 x[13,19] + w[13,19] <= 0
 VehicleRobotSynchronization1[13,20]: - 2 x[13,20] + w[13,20] <= 0
 VehicleRobotSynchronization1[13,21]: - 2 x[13,21] + w[13,21] <= 0
 VehicleRobotSynchronization1[13,22]: - 2 x[13,22] + w[13,22] <= 0
 VehicleRobotSynchronization1[13,23]: - 2 x[13,23] + w[13,23] <= 0
 VehicleRobotSynchronization1[13,24]: - 2 x[13,24] + w[13,24] <= 0
 VehicleRobotSynchronization1[13,25]: - 2 x[13,25] + w[13,25] <= 0
 VehicleRobotSynchronization1[14,0]: - 2 x[14,0] + w[14,0] <= 0
 VehicleRobotSynchronization1[14,1]: - 2 x[14,1] + w[14,1] <= 0
 VehicleRobotSynchronization1[14,2]: - 2 x[14,2] + w[14,2] <= 0
 VehicleRobotSynchronization1[14,3]: - 2 x[14,3] + w[14,3] <= 0
 VehicleRobotSynchronization1[14,5]: - 2 x[14,5] + w[14,5] <= 0
 VehicleRobotSynchronization1[14,6]: - 2 x[14,6] + w[14,6] <= 0
 VehicleRobotSynchronization1[14,7]: - 2 x[14,7] + w[14,7] <= 0
 VehicleRobotSynchronization1[14,8]: - 2 x[14,8] + w[14,8] <= 0
 VehicleRobotSynchronization1[14,10]: - 2 x[14,10] + w[14,10] <= 0
 VehicleRobotSynchronization1[14,11]: - 2 x[14,11] + w[14,11] <= 0
 VehicleRobotSynchronization1[14,12]: - 2 x[14,12] + w[14,12] <= 0
 VehicleRobotSynchronization1[14,13]: - 2 x[14,13] + w[14,13] <= 0
 VehicleRobotSynchronization1[14,15]: - 2 x[14,15] + w[14,15] <= 0
 VehicleRobotSynchronization1[14,16]: - 2 x[14,16] + w[14,16] <= 0
 VehicleRobotSynchronization1[14,17]: - 2 x[14,17] + w[14,17] <= 0
 VehicleRobotSynchronization1[14,18]: - 2 x[14,18] + w[14,18] <= 0
 VehicleRobotSynchronization1[14,20]: - 2 x[14,20] + w[14,20] <= 0
 VehicleRobotSynchronization1[14,21]: - 2 x[14,21] + w[14,21] <= 0
 VehicleRobotSynchronization1[14,22]: - 2 x[14,22] + w[14,22] <= 0
 VehicleRobotSynchronization1[14,23]: - 2 x[14,23] + w[14,23] <= 0
 VehicleRobotSynchronization1[14,24]: - 2 x[14,24] + w[14,24] <= 0
 VehicleRobotSynchronization1[14,25]: - 2 x[14,25] + w[14,25] <= 0
 VehicleRobotSynchronization1[15,0]: - 2 x[15,0] + w[15,0] <= 0
 VehicleRobotSynchronization1[15,1]: - 2 x[15,1] + w[15,1] <= 0
 VehicleRobotSynchronization1[15,2]: - 2 x[15,2] + w[15,2] <= 0
 VehicleRobotSynchronization1[15,3]: - 2 x[15,3] + w[15,3] <= 0
 VehicleRobotSynchronization1[15,4]: - 2 x[15,4] + w[15,4] <= 0
 VehicleRobotSynchronization1[15,6]: - 2 x[15,6] + w[15,6] <= 0
 VehicleRobotSynchronization1[15,7]: - 2 x[15,7] + w[15,7] <= 0
 VehicleRobotSynchronization1[15,8]: - 2 x[15,8] + w[15,8] <= 0
 VehicleRobotSynchronization1[15,9]: - 2 x[15,9] + w[15,9] <= 0
 VehicleRobotSynchronization1[15,11]: - 2 x[15,11] + w[15,11] <= 0
 VehicleRobotSynchronization1[15,12]: - 2 x[15,12] + w[15,12] <= 0
 VehicleRobotSynchronization1[15,13]: - 2 x[15,13] + w[15,13] <= 0
 VehicleRobotSynchronization1[15,14]: - 2 x[15,14] + w[15,14] <= 0
 VehicleRobotSynchronization1[15,16]: - 2 x[15,16] + w[15,16] <= 0
 VehicleRobotSynchronization1[15,17]: - 2 x[15,17] + w[15,17] <= 0
 VehicleRobotSynchronization1[15,18]: - 2 x[15,18] + w[15,18] <= 0
 VehicleRobotSynchronization1[15,19]: - 2 x[15,19] + w[15,19] <= 0
 VehicleRobotSynchronization1[15,21]: - 2 x[15,21] + w[15,21] <= 0
 VehicleRobotSynchronization1[15,22]: - 2 x[15,22] + w[15,22] <= 0
 VehicleRobotSynchronization1[15,23]: - 2 x[15,23] + w[15,23] <= 0
 VehicleRobotSynchronization1[15,24]: - 2 x[15,24] + w[15,24] <= 0
 VehicleRobotSynchronization1[15,25]: - 2 x[15,25] + w[15,25] <= 0
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
 VehicleRobotSynchronization1[16,12]: - 2 x[16,12] + w[16,12] <= 0
 VehicleRobotSynchronization1[16,13]: - 2 x[16,13] + w[16,13] <= 0
 VehicleRobotSynchronization1[16,14]: - 2 x[16,14] + w[16,14] <= 0
 VehicleRobotSynchronization1[16,15]: - 2 x[16,15] + w[16,15] <= 0
 VehicleRobotSynchronization1[16,17]: - 2 x[16,17] + w[16,17] <= 0
 VehicleRobotSynchronization1[16,18]: - 2 x[16,18] + w[16,18] <= 0
 VehicleRobotSynchronization1[16,19]: - 2 x[16,19] + w[16,19] <= 0
 VehicleRobotSynchronization1[16,20]: - 2 x[16,20] + w[16,20] <= 0
 VehicleRobotSynchronization1[16,22]: - 2 x[16,22] + w[16,22] <= 0
 VehicleRobotSynchronization1[16,23]: - 2 x[16,23] + w[16,23] <= 0
 VehicleRobotSynchronization1[16,24]: - 2 x[16,24] + w[16,24] <= 0
 VehicleRobotSynchronization1[16,25]: - 2 x[16,25] + w[16,25] <= 0
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
 VehicleRobotSynchronization1[17,13]: - 2 x[17,13] + w[17,13] <= 0
 VehicleRobotSynchronization1[17,14]: - 2 x[17,14] + w[17,14] <= 0
 VehicleRobotSynchronization1[17,15]: - 2 x[17,15] + w[17,15] <= 0
 VehicleRobotSynchronization1[17,16]: - 2 x[17,16] + w[17,16] <= 0
 VehicleRobotSynchronization1[17,18]: - 2 x[17,18] + w[17,18] <= 0
 VehicleRobotSynchronization1[17,19]: - 2 x[17,19] + w[17,19] <= 0
 VehicleRobotSynchronization1[17,20]: - 2 x[17,20] + w[17,20] <= 0
 VehicleRobotSynchronization1[17,21]: - 2 x[17,21] + w[17,21] <= 0
 VehicleRobotSynchronization1[17,23]: - 2 x[17,23] + w[17,23] <= 0
 VehicleRobotSynchronization1[17,24]: - 2 x[17,24] + w[17,24] <= 0
 VehicleRobotSynchronization1[17,25]: - 2 x[17,25] + w[17,25] <= 0
 VehicleRobotSynchronization1[18,0]: - 2 x[18,0] + w[18,0] <= 0
 VehicleRobotSynchronization1[18,1]: - 2 x[18,1] + w[18,1] <= 0
 VehicleRobotSynchronization1[18,2]: - 2 x[18,2] + w[18,2] <= 0
 VehicleRobotSynchronization1[18,4]: - 2 x[18,4] + w[18,4] <= 0
 VehicleRobotSynchronization1[18,5]: - 2 x[18,5] + w[18,5] <= 0
 VehicleRobotSynchronization1[18,6]: - 2 x[18,6] + w[18,6] <= 0
 VehicleRobotSynchronization1[18,7]: - 2 x[18,7] + w[18,7] <= 0
 VehicleRobotSynchronization1[18,9]: - 2 x[18,9] + w[18,9] <= 0
 VehicleRobotSynchronization1[18,10]: - 2 x[18,10] + w[18,10] <= 0
 VehicleRobotSynchronization1[18,11]: - 2 x[18,11] + w[18,11] <= 0
 VehicleRobotSynchronization1[18,12]: - 2 x[18,12] + w[18,12] <= 0
 VehicleRobotSynchronization1[18,14]: - 2 x[18,14] + w[18,14] <= 0
 VehicleRobotSynchronization1[18,15]: - 2 x[18,15] + w[18,15] <= 0
 VehicleRobotSynchronization1[18,16]: - 2 x[18,16] + w[18,16] <= 0
 VehicleRobotSynchronization1[18,17]: - 2 x[18,17] + w[18,17] <= 0
 VehicleRobotSynchronization1[18,19]: - 2 x[18,19] + w[18,19] <= 0
 VehicleRobotSynchronization1[18,20]: - 2 x[18,20] + w[18,20] <= 0
 VehicleRobotSynchronization1[18,21]: - 2 x[18,21] + w[18,21] <= 0
 VehicleRobotSynchronization1[18,22]: - 2 x[18,22] + w[18,22] <= 0
 VehicleRobotSynchronization1[18,23]: - 2 x[18,23] + w[18,23] <= 0
 VehicleRobotSynchronization1[18,24]: - 2 x[18,24] + w[18,24] <= 0
 VehicleRobotSynchronization1[18,25]: - 2 x[18,25] + w[18,25] <= 0
 VehicleRobotSynchronization1[19,0]: - 2 x[19,0] + w[19,0] <= 0
 VehicleRobotSynchronization1[19,1]: - 2 x[19,1] + w[19,1] <= 0
 VehicleRobotSynchronization1[19,2]: - 2 x[19,2] + w[19,2] <= 0
 VehicleRobotSynchronization1[19,3]: - 2 x[19,3] + w[19,3] <= 0
 VehicleRobotSynchronization1[19,5]: - 2 x[19,5] + w[19,5] <= 0
 VehicleRobotSynchronization1[19,6]: - 2 x[19,6] + w[19,6] <= 0
 VehicleRobotSynchronization1[19,7]: - 2 x[19,7] + w[19,7] <= 0
 VehicleRobotSynchronization1[19,8]: - 2 x[19,8] + w[19,8] <= 0
 VehicleRobotSynchronization1[19,10]: - 2 x[19,10] + w[19,10] <= 0
 VehicleRobotSynchronization1[19,11]: - 2 x[19,11] + w[19,11] <= 0
 VehicleRobotSynchronization1[19,12]: - 2 x[19,12] + w[19,12] <= 0
 VehicleRobotSynchronization1[19,13]: - 2 x[19,13] + w[19,13] <= 0
 VehicleRobotSynchronization1[19,15]: - 2 x[19,15] + w[19,15] <= 0
 VehicleRobotSynchronization1[19,16]: - 2 x[19,16] + w[19,16] <= 0
 VehicleRobotSynchronization1[19,17]: - 2 x[19,17] + w[19,17] <= 0
 VehicleRobotSynchronization1[19,18]: - 2 x[19,18] + w[19,18] <= 0
 VehicleRobotSynchronization1[19,20]: - 2 x[19,20] + w[19,20] <= 0
 VehicleRobotSynchronization1[19,21]: - 2 x[19,21] + w[19,21] <= 0
 VehicleRobotSynchronization1[19,22]: - 2 x[19,22] + w[19,22] <= 0
 VehicleRobotSynchronization1[19,23]: - 2 x[19,23] + w[19,23] <= 0
 VehicleRobotSynchronization1[19,24]: - 2 x[19,24] + w[19,24] <= 0
 VehicleRobotSynchronization1[19,25]: - 2 x[19,25] + w[19,25] <= 0
 VehicleRobotSynchronization1[20,0]: - 2 x[20,0] + w[20,0] <= 0
 VehicleRobotSynchronization1[20,1]: - 2 x[20,1] + w[20,1] <= 0
 VehicleRobotSynchronization1[20,2]: - 2 x[20,2] + w[20,2] <= 0
 VehicleRobotSynchronization1[20,3]: - 2 x[20,3] + w[20,3] <= 0
 VehicleRobotSynchronization1[20,4]: - 2 x[20,4] + w[20,4] <= 0
 VehicleRobotSynchronization1[20,6]: - 2 x[20,6] + w[20,6] <= 0
 VehicleRobotSynchronization1[20,7]: - 2 x[20,7] + w[20,7] <= 0
 VehicleRobotSynchronization1[20,8]: - 2 x[20,8] + w[20,8] <= 0
 VehicleRobotSynchronization1[20,9]: - 2 x[20,9] + w[20,9] <= 0
 VehicleRobotSynchronization1[20,11]: - 2 x[20,11] + w[20,11] <= 0
 VehicleRobotSynchronization1[20,12]: - 2 x[20,12] + w[20,12] <= 0
 VehicleRobotSynchronization1[20,13]: - 2 x[20,13] + w[20,13] <= 0
 VehicleRobotSynchronization1[20,14]: - 2 x[20,14] + w[20,14] <= 0
 VehicleRobotSynchronization1[20,16]: - 2 x[20,16] + w[20,16] <= 0
 VehicleRobotSynchronization1[20,17]: - 2 x[20,17] + w[20,17] <= 0
 VehicleRobotSynchronization1[20,18]: - 2 x[20,18] + w[20,18] <= 0
 VehicleRobotSynchronization1[20,19]: - 2 x[20,19] + w[20,19] <= 0
 VehicleRobotSynchronization1[20,21]: - 2 x[20,21] + w[20,21] <= 0
 VehicleRobotSynchronization1[20,22]: - 2 x[20,22] + w[20,22] <= 0
 VehicleRobotSynchronization1[20,23]: - 2 x[20,23] + w[20,23] <= 0
 VehicleRobotSynchronization1[20,24]: - 2 x[20,24] + w[20,24] <= 0
 VehicleRobotSynchronization1[20,25]: - 2 x[20,25] + w[20,25] <= 0
 VehicleRobotSynchronization1[21,0]: - 2 x[21,0] + w[21,0] <= 0
 VehicleRobotSynchronization1[21,1]: - 2 x[21,1] + w[21,1] <= 0
 VehicleRobotSynchronization1[21,2]: - 2 x[21,2] + w[21,2] <= 0
 VehicleRobotSynchronization1[21,3]: - 2 x[21,3] + w[21,3] <= 0
 VehicleRobotSynchronization1[21,4]: - 2 x[21,4] + w[21,4] <= 0
 VehicleRobotSynchronization1[21,5]: - 2 x[21,5] + w[21,5] <= 0
 VehicleRobotSynchronization1[21,7]: - 2 x[21,7] + w[21,7] <= 0
 VehicleRobotSynchronization1[21,8]: - 2 x[21,8] + w[21,8] <= 0
 VehicleRobotSynchronization1[21,9]: - 2 x[21,9] + w[21,9] <= 0
 VehicleRobotSynchronization1[21,10]: - 2 x[21,10] + w[21,10] <= 0
 VehicleRobotSynchronization1[21,12]: - 2 x[21,12] + w[21,12] <= 0
 VehicleRobotSynchronization1[21,13]: - 2 x[21,13] + w[21,13] <= 0
 VehicleRobotSynchronization1[21,14]: - 2 x[21,14] + w[21,14] <= 0
 VehicleRobotSynchronization1[21,15]: - 2 x[21,15] + w[21,15] <= 0
 VehicleRobotSynchronization1[21,17]: - 2 x[21,17] + w[21,17] <= 0
 VehicleRobotSynchronization1[21,18]: - 2 x[21,18] + w[21,18] <= 0
 VehicleRobotSynchronization1[21,19]: - 2 x[21,19] + w[21,19] <= 0
 VehicleRobotSynchronization1[21,20]: - 2 x[21,20] + w[21,20] <= 0
 VehicleRobotSynchronization1[21,22]: - 2 x[21,22] + w[21,22] <= 0
 VehicleRobotSynchronization1[21,23]: - 2 x[21,23] + w[21,23] <= 0
 VehicleRobotSynchronization1[21,24]: - 2 x[21,24] + w[21,24] <= 0
 VehicleRobotSynchronization1[21,25]: - 2 x[21,25] + w[21,25] <= 0
 VehicleRobotSynchronization1[22,0]: - 2 x[22,0] + w[22,0] <= 0
 VehicleRobotSynchronization1[22,1]: - 2 x[22,1] + w[22,1] <= 0
 VehicleRobotSynchronization1[22,2]: - 2 x[22,2] + w[22,2] <= 0
 VehicleRobotSynchronization1[22,3]: - 2 x[22,3] + w[22,3] <= 0
 VehicleRobotSynchronization1[22,4]: - 2 x[22,4] + w[22,4] <= 0
 VehicleRobotSynchronization1[22,5]: - 2 x[22,5] + w[22,5] <= 0
 VehicleRobotSynchronization1[22,6]: - 2 x[22,6] + w[22,6] <= 0
 VehicleRobotSynchronization1[22,8]: - 2 x[22,8] + w[22,8] <= 0
 VehicleRobotSynchronization1[22,9]: - 2 x[22,9] + w[22,9] <= 0
 VehicleRobotSynchronization1[22,10]: - 2 x[22,10] + w[22,10] <= 0
 VehicleRobotSynchronization1[22,11]: - 2 x[22,11] + w[22,11] <= 0
 VehicleRobotSynchronization1[22,13]: - 2 x[22,13] + w[22,13] <= 0
 VehicleRobotSynchronization1[22,14]: - 2 x[22,14] + w[22,14] <= 0
 VehicleRobotSynchronization1[22,15]: - 2 x[22,15] + w[22,15] <= 0
 VehicleRobotSynchronization1[22,16]: - 2 x[22,16] + w[22,16] <= 0
 VehicleRobotSynchronization1[22,18]: - 2 x[22,18] + w[22,18] <= 0
 VehicleRobotSynchronization1[22,19]: - 2 x[22,19] + w[22,19] <= 0
 VehicleRobotSynchronization1[22,20]: - 2 x[22,20] + w[22,20] <= 0
 VehicleRobotSynchronization1[22,21]: - 2 x[22,21] + w[22,21] <= 0
 VehicleRobotSynchronization1[22,23]: - 2 x[22,23] + w[22,23] <= 0
 VehicleRobotSynchronization1[22,24]: - 2 x[22,24] + w[22,24] <= 0
 VehicleRobotSynchronization1[22,25]: - 2 x[22,25] + w[22,25] <= 0
 VehicleRobotSynchronization1[23,0]: - 2 x[23,0] + w[23,0] <= 0
 VehicleRobotSynchronization1[23,1]: - 2 x[23,1] + w[23,1] <= 0
 VehicleRobotSynchronization1[23,2]: - 2 x[23,2] + w[23,2] <= 0
 VehicleRobotSynchronization1[23,3]: - 2 x[23,3] + w[23,3] <= 0
 VehicleRobotSynchronization1[23,4]: - 2 x[23,4] + w[23,4] <= 0
 VehicleRobotSynchronization1[23,5]: - 2 x[23,5] + w[23,5] <= 0
 VehicleRobotSynchronization1[23,6]: - 2 x[23,6] + w[23,6] <= 0
 VehicleRobotSynchronization1[23,7]: - 2 x[23,7] + w[23,7] <= 0
 VehicleRobotSynchronization1[23,8]: - 2 x[23,8] + w[23,8] <= 0
 VehicleRobotSynchronization1[23,9]: - 2 x[23,9] + w[23,9] <= 0
 VehicleRobotSynchronization1[23,10]: - 2 x[23,10] + w[23,10] <= 0
 VehicleRobotSynchronization1[23,11]: - 2 x[23,11] + w[23,11] <= 0
 VehicleRobotSynchronization1[23,12]: - 2 x[23,12] + w[23,12] <= 0
 VehicleRobotSynchronization1[23,13]: - 2 x[23,13] + w[23,13] <= 0
 VehicleRobotSynchronization1[23,14]: - 2 x[23,14] + w[23,14] <= 0
 VehicleRobotSynchronization1[23,15]: - 2 x[23,15] + w[23,15] <= 0
 VehicleRobotSynchronization1[23,16]: - 2 x[23,16] + w[23,16] <= 0
 VehicleRobotSynchronization1[23,17]: - 2 x[23,17] + w[23,17] <= 0
 VehicleRobotSynchronization1[23,18]: - 2 x[23,18] + w[23,18] <= 0
 VehicleRobotSynchronization1[23,19]: - 2 x[23,19] + w[23,19] <= 0
 VehicleRobotSynchronization1[23,20]: - 2 x[23,20] + w[23,20] <= 0
 VehicleRobotSynchronization1[23,21]: - 2 x[23,21] + w[23,21] <= 0
 VehicleRobotSynchronization1[23,22]: - 2 x[23,22] + w[23,22] <= 0
 VehicleRobotSynchronization1[23,24]: - 2 x[23,24] + w[23,24] <= 0
 VehicleRobotSynchronization1[23,25]: - 2 x[23,25] + w[23,25] <= 0
 VehicleRobotSynchronization1[24,0]: - 2 x[24,0] + w[24,0] <= 0
 VehicleRobotSynchronization1[24,1]: - 2 x[24,1] + w[24,1] <= 0
 VehicleRobotSynchronization1[24,2]: - 2 x[24,2] + w[24,2] <= 0
 VehicleRobotSynchronization1[24,3]: - 2 x[24,3] + w[24,3] <= 0
 VehicleRobotSynchronization1[24,4]: - 2 x[24,4] + w[24,4] <= 0
 VehicleRobotSynchronization1[24,5]: - 2 x[24,5] + w[24,5] <= 0
 VehicleRobotSynchronization1[24,6]: - 2 x[24,6] + w[24,6] <= 0
 VehicleRobotSynchronization1[24,7]: - 2 x[24,7] + w[24,7] <= 0
 VehicleRobotSynchronization1[24,8]: - 2 x[24,8] + w[24,8] <= 0
 VehicleRobotSynchronization1[24,9]: - 2 x[24,9] + w[24,9] <= 0
 VehicleRobotSynchronization1[24,10]: - 2 x[24,10] + w[24,10] <= 0
 VehicleRobotSynchronization1[24,11]: - 2 x[24,11] + w[24,11] <= 0
 VehicleRobotSynchronization1[24,12]: - 2 x[24,12] + w[24,12] <= 0
 VehicleRobotSynchronization1[24,13]: - 2 x[24,13] + w[24,13] <= 0
 VehicleRobotSynchronization1[24,14]: - 2 x[24,14] + w[24,14] <= 0
 VehicleRobotSynchronization1[24,15]: - 2 x[24,15] + w[24,15] <= 0
 VehicleRobotSynchronization1[24,16]: - 2 x[24,16] + w[24,16] <= 0
 VehicleRobotSynchronization1[24,17]: - 2 x[24,17] + w[24,17] <= 0
 VehicleRobotSynchronization1[24,18]: - 2 x[24,18] + w[24,18] <= 0
 VehicleRobotSynchronization1[24,19]: - 2 x[24,19] + w[24,19] <= 0
 VehicleRobotSynchronization1[24,20]: - 2 x[24,20] + w[24,20] <= 0
 VehicleRobotSynchronization1[24,21]: - 2 x[24,21] + w[24,21] <= 0
 VehicleRobotSynchronization1[24,22]: - 2 x[24,22] + w[24,22] <= 0
 VehicleRobotSynchronization1[24,23]: - 2 x[24,23] + w[24,23] <= 0
 VehicleRobotSynchronization1[24,25]: - 2 x[24,25] + w[24,25] <= 0
 SingleModePerNode[3,13]: y[3,13,1] + y[3,13,2] <= 1
 SingleModePerNode[3,18]: y[3,18,1] + y[3,18,2] <= 1
 SingleModePerNode[4,14]: y[4,14,1] + y[4,14,2] <= 1
 SingleModePerNode[4,19]: y[4,19,1] + y[4,19,2] <= 1
 SingleModePerNode[5,15]: y[5,15,1] + y[5,15,2] <= 1
 SingleModePerNode[5,20]: y[5,20,1] + y[5,20,2] <= 1
 SingleModePerNode[6,16]: y[6,16,1] + y[6,16,2] <= 1
 SingleModePerNode[6,21]: y[6,21,1] + y[6,21,2] <= 1
 SingleModePerNode[7,17]: y[7,17,1] + y[7,17,2] <= 1
 SingleModePerNode[7,22]: y[7,22,1] + y[7,22,2] <= 1
 SingleModePerNode[8,13]: y[8,13,1] + y[8,13,2] <= 1
 SingleModePerNode[8,18]: y[8,18,1] + y[8,18,2] <= 1
 SingleModePerNode[9,14]: y[9,14,1] + y[9,14,2] <= 1
 SingleModePerNode[9,19]: y[9,19,1] + y[9,19,2] <= 1
 SingleModePerNode[10,15]: y[10,15,1] + y[10,15,2] <= 1
 SingleModePerNode[10,20]: y[10,20,1] + y[10,20,2] <= 1
 SingleModePerNode[11,16]: y[11,16,1] + y[11,16,2] <= 1
 SingleModePerNode[11,21]: y[11,21,1] + y[11,21,2] <= 1
 SingleModePerNode[12,17]: y[12,17,1] + y[12,17,2] <= 1
 SingleModePerNode[12,22]: y[12,22,1] + y[12,22,2] <= 1
 SettingMode[3,13]: - w[3,13] + y[3,13,1] + 2 y[3,13,2] = 0
 SettingMode[3,18]: - w[3,18] + y[3,18,1] + 2 y[3,18,2] = 0
 SettingMode[4,14]: - w[4,14] + y[4,14,1] + 2 y[4,14,2] = 0
 SettingMode[4,19]: - w[4,19] + y[4,19,1] + 2 y[4,19,2] = 0
 SettingMode[5,15]: - w[5,15] + y[5,15,1] + 2 y[5,15,2] = 0
 SettingMode[5,20]: - w[5,20] + y[5,20,1] + 2 y[5,20,2] = 0
 SettingMode[6,16]: - w[6,16] + y[6,16,1] + 2 y[6,16,2] = 0
 SettingMode[6,21]: - w[6,21] + y[6,21,1] + 2 y[6,21,2] = 0
 SettingMode[7,17]: - w[7,17] + y[7,17,1] + 2 y[7,17,2] = 0
 SettingMode[7,22]: - w[7,22] + y[7,22,1] + 2 y[7,22,2] = 0
 SettingMode[8,13]: - w[8,13] + y[8,13,1] + 2 y[8,13,2] = 0
 SettingMode[8,18]: - w[8,18] + y[8,18,1] + 2 y[8,18,2] = 0
 SettingMode[9,14]: - w[9,14] + y[9,14,1] + 2 y[9,14,2] = 0
 SettingMode[9,19]: - w[9,19] + y[9,19,1] + 2 y[9,19,2] = 0
 SettingMode[10,15]: - w[10,15] + y[10,15,1] + 2 y[10,15,2] = 0
 SettingMode[10,20]: - w[10,20] + y[10,20,1] + 2 y[10,20,2] = 0
 SettingMode[11,16]: - w[11,16] + y[11,16,1] + 2 y[11,16,2] = 0
 SettingMode[11,21]: - w[11,21] + y[11,21,1] + 2 y[11,21,2] = 0
 SettingMode[12,17]: - w[12,17] + y[12,17,1] + 2 y[12,17,2] = 0
 SettingMode[12,22]: - w[12,22] + y[12,22,1] + 2 y[12,22,2] = 0
 SettingServiceTime[1,3,13]: - 390 y[3,13,1] + service_time[3,13,1] <= 0
 SettingServiceTime[1,3,18]: - 390 y[3,18,1] + service_time[3,18,1] <= 0
 SettingServiceTime[1,4,14]: - 350 y[4,14,1] + service_time[4,14,1] <= 0
 SettingServiceTime[1,4,19]: - 350 y[4,19,1] + service_time[4,19,1] <= 0
 SettingServiceTime[1,5,15]: - 240 y[5,15,1] + service_time[5,15,1] <= 0
 SettingServiceTime[1,5,20]: - 240 y[5,20,1] + service_time[5,20,1] <= 0
 SettingServiceTime[1,6,16]: - 490 y[6,16,1] + service_time[6,16,1] <= 0
 SettingServiceTime[1,6,21]: - 490 y[6,21,1] + service_time[6,21,1] <= 0
 SettingServiceTime[1,7,17]: - 380 y[7,17,1] + service_time[7,17,1] <= 0
 SettingServiceTime[1,7,22]: - 380 y[7,22,1] + service_time[7,22,1] <= 0
 SettingServiceTime[1,8,13]: - 390 y[8,13,1] + service_time[8,13,1] <= 0
 SettingServiceTime[1,8,18]: - 390 y[8,18,1] + service_time[8,18,1] <= 0
 SettingServiceTime[1,9,14]: - 350 y[9,14,1] + service_time[9,14,1] <= 0
 SettingServiceTime[1,9,19]: - 350 y[9,19,1] + service_time[9,19,1] <= 0
 SettingServiceTime[1,10,15]: - 240 y[10,15,1] + service_time[10,15,1]
   <= 0
 SettingServiceTime[1,10,20]: - 240 y[10,20,1] + service_time[10,20,1]
   <= 0
 SettingServiceTime[1,11,16]: - 490 y[11,16,1] + service_time[11,16,1]
   <= 0
 SettingServiceTime[1,11,21]: - 490 y[11,21,1] + service_time[11,21,1]
   <= 0
 SettingServiceTime[1,12,17]: - 380 y[12,17,1] + service_time[12,17,1]
   <= 0
 SettingServiceTime[1,12,22]: - 380 y[12,22,1] + service_time[12,22,1]
   <= 0
 SettingServiceTime[2,3,13]: - 195 y[3,13,2] + service_time[3,13,2] <= 0
 SettingServiceTime[2,3,18]: - 195 y[3,18,2] + service_time[3,18,2] <= 0
 SettingServiceTime[2,4,14]: - 175 y[4,14,2] + service_time[4,14,2] <= 0
 SettingServiceTime[2,4,19]: - 175 y[4,19,2] + service_time[4,19,2] <= 0
 SettingServiceTime[2,5,15]: - 120 y[5,15,2] + service_time[5,15,2] <= 0
 SettingServiceTime[2,5,20]: - 120 y[5,20,2] + service_time[5,20,2] <= 0
 SettingServiceTime[2,6,16]: - 245 y[6,16,2] + service_time[6,16,2] <= 0
 SettingServiceTime[2,6,21]: - 245 y[6,21,2] + service_time[6,21,2] <= 0
 SettingServiceTime[2,7,17]: - 190 y[7,17,2] + service_time[7,17,2] <= 0
 SettingServiceTime[2,7,22]: - 190 y[7,22,2] + service_time[7,22,2] <= 0
 SettingServiceTime[2,8,13]: - 195 y[8,13,2] + service_time[8,13,2] <= 0
 SettingServiceTime[2,8,18]: - 195 y[8,18,2] + service_time[8,18,2] <= 0
 SettingServiceTime[2,9,14]: - 175 y[9,14,2] + service_time[9,14,2] <= 0
 SettingServiceTime[2,9,19]: - 175 y[9,19,2] + service_time[9,19,2] <= 0
 SettingServiceTime[2,10,15]: - 120 y[10,15,2] + service_time[10,15,2]
   <= 0
 SettingServiceTime[2,10,20]: - 120 y[10,20,2] + service_time[10,20,2]
   <= 0
 SettingServiceTime[2,11,16]: - 245 y[11,16,2] + service_time[11,16,2]
   <= 0
 SettingServiceTime[2,11,21]: - 245 y[11,21,2] + service_time[11,21,2]
   <= 0
 SettingServiceTime[2,12,17]: - 190 y[12,17,2] + service_time[12,17,2]
   <= 0
 SettingServiceTime[2,12,22]: - 190 y[12,22,2] + service_time[12,22,2]
   <= 0
 CompleteDemandSatisfaction[3]: service_time[3,13,1]
   + 2 service_time[3,13,2] + service_time[3,18,1] + 2 service_time[3,18,2]
   + service_time[8,13,1] + 2 service_time[8,13,2] + service_time[8,18,1]
   + 2 service_time[8,18,2] = 390
 CompleteDemandSatisfaction[4]: service_time[4,14,1]
   + 2 service_time[4,14,2] + service_time[4,19,1] + 2 service_time[4,19,2]
   + service_time[9,14,1] + 2 service_time[9,14,2] + service_time[9,19,1]
   + 2 service_time[9,19,2] = 350
 CompleteDemandSatisfaction[5]: service_time[5,15,1]
   + 2 service_time[5,15,2] + service_time[5,20,1] + 2 service_time[5,20,2]
   + service_time[10,15,1] + 2 service_time[10,15,2]
   + service_time[10,20,1] + 2 service_time[10,20,2] = 240
 CompleteDemandSatisfaction[6]: service_time[6,16,1]
   + 2 service_time[6,16,2] + service_time[6,21,1] + 2 service_time[6,21,2]
   + service_time[11,16,1] + 2 service_time[11,16,2]
   + service_time[11,21,1] + 2 service_time[11,21,2] = 490
 CompleteDemandSatisfaction[7]: service_time[7,17,1]
   + 2 service_time[7,17,2] + service_time[7,22,1] + 2 service_time[7,22,2]
   + service_time[12,17,1] + 2 service_time[12,17,2]
   + service_time[12,22,1] + 2 service_time[12,22,2] = 380
 AverageTimeRobots: - 0.00142857142857143 w[0,1]
   - 0.00142857142857143 w[0,2] - 7.841440394274109 w[0,3]
   - 4.650865590725213 w[0,4] - 1.380088031176086 w[0,5]
   - 12.72458926567911 w[0,6] - 1.862272848109308 w[0,7]
   - 7.841440394274109 w[0,8] - 4.650865590725213 w[0,9]
   - 1.380088031176086 w[0,10] - 12.72458926567911 w[0,11]
   - 1.862272848109308 w[0,12] - 7.841440394274109 w[0,13]
   - 4.650865590725213 w[0,14] - 1.380088031176086 w[0,15]
   - 12.72458926567911 w[0,16] - 1.862272848109308 w[0,17]
   - 7.841440394274109 w[0,18] - 4.650865590725213 w[0,19]
   - 1.380088031176086 w[0,20] - 12.72458926567911 w[0,21]
   - 1.862272848109308 w[0,22] - 0.00142857142857143 w[0,23]
   - 0.00142857142857143 w[0,24] - 0.00142857142857143 w[0,25]
   - 0.00142857142857143 w[1,1] - 0.00142857142857143 w[1,2]
   - 7.841440394274109 w[1,3] - 4.650865590725213 w[1,4]
   - 1.380088031176086 w[1,5] - 12.72458926567911 w[1,6]
   - 1.862272848109308 w[1,7] - 7.841440394274109 w[1,8]
   - 4.650865590725213 w[1,9] - 1.380088031176086 w[1,10]
   - 12.72458926567911 w[1,11] - 1.862272848109308 w[1,12]
   - 7.841440394274109 w[1,13] - 4.650865590725213 w[1,14]
   - 1.380088031176086 w[1,15] - 12.72458926567911 w[1,16]
   - 1.862272848109308 w[1,17] - 7.841440394274109 w[1,18]
   - 4.650865590725213 w[1,19] - 1.380088031176086 w[1,20]
   - 12.72458926567911 w[1,21] - 1.862272848109308 w[1,22]
   - 0.00142857142857143 w[1,23] - 0.00142857142857143 w[1,24]
   - 0.00142857142857143 w[1,25] - 0.00142857142857143 w[2,1]
   - 0.00142857142857143 w[2,2] - 7.841440394274109 w[2,3]
   - 4.650865590725213 w[2,4] - 1.380088031176086 w[2,5]
   - 12.72458926567911 w[2,6] - 1.862272848109308 w[2,7]
   - 7.841440394274109 w[2,8] - 4.650865590725213 w[2,9]
   - 1.380088031176086 w[2,10] - 12.72458926567911 w[2,11]
   - 1.862272848109308 w[2,12] - 7.841440394274109 w[2,13]
   - 4.650865590725213 w[2,14] - 1.380088031176086 w[2,15]
   - 12.72458926567911 w[2,16] - 1.862272848109308 w[2,17]
   - 7.841440394274109 w[2,18] - 4.650865590725213 w[2,19]
   - 1.380088031176086 w[2,20] - 12.72458926567911 w[2,21]
   - 1.862272848109308 w[2,22] - 0.00142857142857143 w[2,23]
   - 0.00142857142857143 w[2,24] - 0.00142857142857143 w[2,25]
   - 7.841440394274109 w[3,1] - 7.841440394274109 w[3,2]
   - 0.00142857142857143 w[3,3] - 5.847459304474707 w[3,4]
   - 8.17438252315196 w[3,5] - 10.24279337147347 w[3,6]
   - 5.980794808793723 w[3,7] - 0.00142857142857143 w[3,8]
   - 5.847459304474707 w[3,9] - 8.17438252315196 w[3,10]
   - 10.24279337147347 w[3,11] - 5.980794808793723 w[3,12]
   - 0.00142857142857143 w[3,13] - 5.847459304474707 w[3,14]
   - 8.17438252315196 w[3,15] - 10.24279337147347 w[3,16]
   - 5.980794808793723 w[3,17] - 0.00142857142857143 w[3,18]
   - 5.847459304474707 w[3,19] - 8.17438252315196 w[3,20]
   - 10.24279337147347 w[3,21] - 5.980794808793723 w[3,22]
   - 7.841440394274109 w[3,23] - 7.841440394274109 w[3,24]
   - 7.841440394274109 w[3,25] - 4.650865590725213 w[4,1]
   - 4.650865590725213 w[4,2] - 5.847459304474707 w[4,3]
   - 0.00142857142857143 w[4,4] - 5.856581789606746 w[4,5]
   - 8.109927967333126 w[4,6] - 3.608037782122388 w[4,7]
   - 5.847459304474707 w[4,8] - 0.00142857142857143 w[4,9]
   - 5.856581789606746 w[4,10] - 8.109927967333126 w[4,11]
   - 3.608037782122388 w[4,12] - 5.847459304474707 w[4,13]
   - 0.00142857142857143 w[4,14] - 5.856581789606746 w[4,15]
   - 8.109927967333126 w[4,16] - 3.608037782122388 w[4,17]
   - 5.847459304474707 w[4,18] - 0.00142857142857143 w[4,19]
   - 5.856581789606746 w[4,20] - 8.109927967333126 w[4,21]
   - 3.608037782122388 w[4,22] - 4.650865590725213 w[4,23]
   - 4.650865590725213 w[4,24] - 4.650865590725213 w[4,25]
   - 1.380088031176086 w[5,1] - 1.380088031176086 w[5,2]
   - 8.17438252315196 w[5,3] - 5.856581789606746 w[5,4]
   - 0.00142857142857143 w[5,5] - 13.96519426812116 w[5,6]
   - 2.524175583665984 w[5,7] - 8.17438252315196 w[5,8]
   - 5.856581789606746 w[5,9] - 0.00142857142857143 w[5,10]
   - 13.96519426812116 w[5,11] - 2.524175583665984 w[5,12]
   - 8.17438252315196 w[5,13] - 5.856581789606746 w[5,14]
   - 0.00142857142857143 w[5,15] - 13.96519426812116 w[5,16]
   - 2.524175583665984 w[5,17] - 8.17438252315196 w[5,18]
   - 5.856581789606746 w[5,19] - 0.00142857142857143 w[5,20]
   - 13.96519426812116 w[5,21] - 2.524175583665984 w[5,22]
   - 1.380088031176086 w[5,23] - 1.380088031176086 w[5,24]
   - 1.380088031176086 w[5,25] - 12.72458926567911 w[6,1]
   - 12.72458926567911 w[6,2] - 10.24279337147347 w[6,3]
   - 8.109927967333126 w[6,4] - 13.96519426812116 w[6,5]
   - 0.00142857142857143 w[6,6] - 11.65633090491722 w[6,7]
   - 10.24279337147347 w[6,8] - 8.109927967333126 w[6,9]
   - 13.96519426812116 w[6,10] - 0.00142857142857143 w[6,11]
   - 11.65633090491722 w[6,12] - 10.24279337147347 w[6,13]
   - 8.109927967333126 w[6,14] - 13.96519426812116 w[6,15]
   - 0.00142857142857143 w[6,16] - 11.65633090491722 w[6,17]
   - 10.24279337147347 w[6,18] - 8.109927967333126 w[6,19]
   - 13.96519426812116 w[6,20] - 0.00142857142857143 w[6,21]
   - 11.65633090491722 w[6,22] - 12.72458926567911 w[6,23]
   - 12.72458926567911 w[6,24] - 12.72458926567911 w[6,25]
   - 1.862272848109308 w[7,1] - 1.862272848109308 w[7,2]
   - 5.980794808793723 w[7,3] - 3.608037782122388 w[7,4]
   - 2.524175583665984 w[7,5] - 11.65633090491722 w[7,6]
   - 0.00142857142857143 w[7,7] - 5.980794808793723 w[7,8]
   - 3.608037782122388 w[7,9] - 2.524175583665984 w[7,10]
   - 11.65633090491722 w[7,11] - 0.00142857142857143 w[7,12]
   - 5.980794808793723 w[7,13] - 3.608037782122388 w[7,14]
   - 2.524175583665984 w[7,15] - 11.65633090491722 w[7,16]
   - 0.00142857142857143 w[7,17] - 5.980794808793723 w[7,18]
   - 3.608037782122388 w[7,19] - 2.524175583665984 w[7,20]
   - 11.65633090491722 w[7,21] - 0.00142857142857143 w[7,22]
   - 1.862272848109308 w[7,23] - 1.862272848109308 w[7,24]
   - 1.862272848109308 w[7,25] - 7.841440394274109 w[8,1]
   - 7.841440394274109 w[8,2] - 0.00142857142857143 w[8,3]
   - 5.847459304474707 w[8,4] - 8.17438252315196 w[8,5]
   - 10.24279337147347 w[8,6] - 5.980794808793723 w[8,7]
   - 0.00142857142857143 w[8,8] - 5.847459304474707 w[8,9]
   - 8.17438252315196 w[8,10] - 10.24279337147347 w[8,11]
   - 5.980794808793723 w[8,12] - 0.00142857142857143 w[8,13]
   - 5.847459304474707 w[8,14] - 8.17438252315196 w[8,15]
   - 10.24279337147347 w[8,16] - 5.980794808793723 w[8,17]
   - 0.00142857142857143 w[8,18] - 5.847459304474707 w[8,19]
   - 8.17438252315196 w[8,20] - 10.24279337147347 w[8,21]
   - 5.980794808793723 w[8,22] - 7.841440394274109 w[8,23]
   - 7.841440394274109 w[8,24] - 7.841440394274109 w[8,25]
   - 4.650865590725213 w[9,1] - 4.650865590725213 w[9,2]
   - 5.847459304474707 w[9,3] - 0.00142857142857143 w[9,4]
   - 5.856581789606746 w[9,5] - 8.109927967333126 w[9,6]
   - 3.608037782122388 w[9,7] - 5.847459304474707 w[9,8]
   - 0.00142857142857143 w[9,9] - 5.856581789606746 w[9,10]
   - 8.109927967333126 w[9,11] - 3.608037782122388 w[9,12]
   - 5.847459304474707 w[9,13] - 0.00142857142857143 w[9,14]
   - 5.856581789606746 w[9,15] - 8.109927967333126 w[9,16]
   - 3.608037782122388 w[9,17] - 5.847459304474707 w[9,18]
   - 0.00142857142857143 w[9,19] - 5.856581789606746 w[9,20]
   - 8.109927967333126 w[9,21] - 3.608037782122388 w[9,22]
   - 4.650865590725213 w[9,23] - 4.650865590725213 w[9,24]
   - 4.650865590725213 w[9,25] - 1.380088031176086 w[10,1]
   - 1.380088031176086 w[10,2] - 8.17438252315196 w[10,3]
   - 5.856581789606746 w[10,4] - 0.00142857142857143 w[10,5]
   - 13.96519426812116 w[10,6] - 2.524175583665984 w[10,7]
   - 8.17438252315196 w[10,8] - 5.856581789606746 w[10,9]
   - 0.00142857142857143 w[10,10] - 13.96519426812116 w[10,11]
   - 2.524175583665984 w[10,12] - 8.17438252315196 w[10,13]
   - 5.856581789606746 w[10,14] - 0.00142857142857143 w[10,15]
   - 13.96519426812116 w[10,16] - 2.524175583665984 w[10,17]
   - 8.17438252315196 w[10,18] - 5.856581789606746 w[10,19]
   - 0.00142857142857143 w[10,20] - 13.96519426812116 w[10,21]
   - 2.524175583665984 w[10,22] - 1.380088031176086 w[10,23]
   - 1.380088031176086 w[10,24] - 1.380088031176086 w[10,25]
   - 12.72458926567911 w[11,1] - 12.72458926567911 w[11,2]
   - 10.24279337147347 w[11,3] - 8.109927967333126 w[11,4]
   - 13.96519426812116 w[11,5] - 0.00142857142857143 w[11,6]
   - 11.65633090491722 w[11,7] - 10.24279337147347 w[11,8]
   - 8.109927967333126 w[11,9] - 13.96519426812116 w[11,10]
   - 0.00142857142857143 w[11,11] - 11.65633090491722 w[11,12]
   - 10.24279337147347 w[11,13] - 8.109927967333126 w[11,14]
   - 13.96519426812116 w[11,15] - 0.00142857142857143 w[11,16]
   - 11.65633090491722 w[11,17] - 10.24279337147347 w[11,18]
   - 8.109927967333126 w[11,19] - 13.96519426812116 w[11,20]
   - 0.00142857142857143 w[11,21] - 11.65633090491722 w[11,22]
   - 12.72458926567911 w[11,23] - 12.72458926567911 w[11,24]
   - 12.72458926567911 w[11,25] - 1.862272848109308 w[12,1]
   - 1.862272848109308 w[12,2] - 5.980794808793723 w[12,3]
   - 3.608037782122388 w[12,4] - 2.524175583665984 w[12,5]
   - 11.65633090491722 w[12,6] - 0.00142857142857143 w[12,7]
   - 5.980794808793723 w[12,8] - 3.608037782122388 w[12,9]
   - 2.524175583665984 w[12,10] - 11.65633090491722 w[12,11]
   - 0.00142857142857143 w[12,12] - 5.980794808793723 w[12,13]
   - 3.608037782122388 w[12,14] - 2.524175583665984 w[12,15]
   - 11.65633090491722 w[12,16] - 0.00142857142857143 w[12,17]
   - 5.980794808793723 w[12,18] - 3.608037782122388 w[12,19]
   - 2.524175583665984 w[12,20] - 11.65633090491722 w[12,21]
   - 0.00142857142857143 w[12,22] - 1.862272848109308 w[12,23]
   - 1.862272848109308 w[12,24] - 1.862272848109308 w[12,25]
   - 7.841440394274109 w[13,1] - 7.841440394274109 w[13,2]
   - 0.00142857142857143 w[13,3] - 5.847459304474707 w[13,4]
   - 8.17438252315196 w[13,5] - 10.24279337147347 w[13,6]
   - 5.980794808793723 w[13,7] - 0.00142857142857143 w[13,8]
   - 5.847459304474707 w[13,9] - 8.17438252315196 w[13,10]
   - 10.24279337147347 w[13,11] - 5.980794808793723 w[13,12]
   - 0.00142857142857143 w[13,13] - 5.847459304474707 w[13,14]
   - 8.17438252315196 w[13,15] - 10.24279337147347 w[13,16]
   - 5.980794808793723 w[13,17] - 0.00142857142857143 w[13,18]
   - 5.847459304474707 w[13,19] - 8.17438252315196 w[13,20]
   - 10.24279337147347 w[13,21] - 5.980794808793723 w[13,22]
   - 7.841440394274109 w[13,23] - 7.841440394274109 w[13,24]
   - 7.841440394274109 w[13,25] - 4.650865590725213 w[14,1]
   - 4.650865590725213 w[14,2] - 5.847459304474707 w[14,3]
   - 0.00142857142857143 w[14,4] - 5.856581789606746 w[14,5]
   - 8.109927967333126 w[14,6] - 3.608037782122388 w[14,7]
   - 5.847459304474707 w[14,8] - 0.00142857142857143 w[14,9]
   - 5.856581789606746 w[14,10] - 8.109927967333126 w[14,11]
   - 3.608037782122388 w[14,12] - 5.847459304474707 w[14,13]
   - 0.00142857142857143 w[14,14] - 5.856581789606746 w[14,15]
   - 8.109927967333126 w[14,16] - 3.608037782122388 w[14,17]
   - 5.847459304474707 w[14,18] - 0.00142857142857143 w[14,19]
   - 5.856581789606746 w[14,20] - 8.109927967333126 w[14,21]
   - 3.608037782122388 w[14,22] - 4.650865590725213 w[14,23]
   - 4.650865590725213 w[14,24] - 4.650865590725213 w[14,25]
   - 1.380088031176086 w[15,1] - 1.380088031176086 w[15,2]
   - 8.17438252315196 w[15,3] - 5.856581789606746 w[15,4]
   - 0.00142857142857143 w[15,5] - 13.96519426812116 w[15,6]
   - 2.524175583665984 w[15,7] - 8.17438252315196 w[15,8]
   - 5.856581789606746 w[15,9] - 0.00142857142857143 w[15,10]
   - 13.96519426812116 w[15,11] - 2.524175583665984 w[15,12]
   - 8.17438252315196 w[15,13] - 5.856581789606746 w[15,14]
   - 0.00142857142857143 w[15,15] - 13.96519426812116 w[15,16]
   - 2.524175583665984 w[15,17] - 8.17438252315196 w[15,18]
   - 5.856581789606746 w[15,19] - 0.00142857142857143 w[15,20]
   - 13.96519426812116 w[15,21] - 2.524175583665984 w[15,22]
   - 1.380088031176086 w[15,23] - 1.380088031176086 w[15,24]
   - 1.380088031176086 w[15,25] - 12.72458926567911 w[16,1]
   - 12.72458926567911 w[16,2] - 10.24279337147347 w[16,3]
   - 8.109927967333126 w[16,4] - 13.96519426812116 w[16,5]
   - 0.00142857142857143 w[16,6] - 11.65633090491722 w[16,7]
   - 10.24279337147347 w[16,8] - 8.109927967333126 w[16,9]
   - 13.96519426812116 w[16,10] - 0.00142857142857143 w[16,11]
   - 11.65633090491722 w[16,12] - 10.24279337147347 w[16,13]
   - 8.109927967333126 w[16,14] - 13.96519426812116 w[16,15]
   - 0.00142857142857143 w[16,16] - 11.65633090491722 w[16,17]
   - 10.24279337147347 w[16,18] - 8.109927967333126 w[16,19]
   - 13.96519426812116 w[16,20] - 0.00142857142857143 w[16,21]
   - 11.65633090491722 w[16,22] - 12.72458926567911 w[16,23]
   - 12.72458926567911 w[16,24] - 12.72458926567911 w[16,25]
   - 1.862272848109308 w[17,1] - 1.862272848109308 w[17,2]
   - 5.980794808793723 w[17,3] - 3.608037782122388 w[17,4]
   - 2.524175583665984 w[17,5] - 11.65633090491722 w[17,6]
   - 0.00142857142857143 w[17,7] - 5.980794808793723 w[17,8]
   - 3.608037782122388 w[17,9] - 2.524175583665984 w[17,10]
   - 11.65633090491722 w[17,11] - 0.00142857142857143 w[17,12]
   - 5.980794808793723 w[17,13] - 3.608037782122388 w[17,14]
   - 2.524175583665984 w[17,15] - 11.65633090491722 w[17,16]
   - 0.00142857142857143 w[17,17] - 5.980794808793723 w[17,18]
   - 3.608037782122388 w[17,19] - 2.524175583665984 w[17,20]
   - 11.65633090491722 w[17,21] - 0.00142857142857143 w[17,22]
   - 1.862272848109308 w[17,23] - 1.862272848109308 w[17,24]
   - 1.862272848109308 w[17,25] - 7.841440394274109 w[18,1]
   - 7.841440394274109 w[18,2] - 0.00142857142857143 w[18,3]
   - 5.847459304474707 w[18,4] - 8.17438252315196 w[18,5]
   - 10.24279337147347 w[18,6] - 5.980794808793723 w[18,7]
   - 0.00142857142857143 w[18,8] - 5.847459304474707 w[18,9]
   - 8.17438252315196 w[18,10] - 10.24279337147347 w[18,11]
   - 5.980794808793723 w[18,12] - 0.00142857142857143 w[18,13]
   - 5.847459304474707 w[18,14] - 8.17438252315196 w[18,15]
   - 10.24279337147347 w[18,16] - 5.980794808793723 w[18,17]
   - 0.00142857142857143 w[18,18] - 5.847459304474707 w[18,19]
   - 8.17438252315196 w[18,20] - 10.24279337147347 w[18,21]
   - 5.980794808793723 w[18,22] - 7.841440394274109 w[18,23]
   - 7.841440394274109 w[18,24] - 7.841440394274109 w[18,25]
   - 4.650865590725213 w[19,1] - 4.650865590725213 w[19,2]
   - 5.847459304474707 w[19,3] - 0.00142857142857143 w[19,4]
   - 5.856581789606746 w[19,5] - 8.109927967333126 w[19,6]
   - 3.608037782122388 w[19,7] - 5.847459304474707 w[19,8]
   - 0.00142857142857143 w[19,9] - 5.856581789606746 w[19,10]
   - 8.109927967333126 w[19,11] - 3.608037782122388 w[19,12]
   - 5.847459304474707 w[19,13] - 0.00142857142857143 w[19,14]
   - 5.856581789606746 w[19,15] - 8.109927967333126 w[19,16]
   - 3.608037782122388 w[19,17] - 5.847459304474707 w[19,18]
   - 0.00142857142857143 w[19,19] - 5.856581789606746 w[19,20]
   - 8.109927967333126 w[19,21] - 3.608037782122388 w[19,22]
   - 4.650865590725213 w[19,23] - 4.650865590725213 w[19,24]
   - 4.650865590725213 w[19,25] - 1.380088031176086 w[20,1]
   - 1.380088031176086 w[20,2] - 8.17438252315196 w[20,3]
   - 5.856581789606746 w[20,4] - 0.00142857142857143 w[20,5]
   - 13.96519426812116 w[20,6] - 2.524175583665984 w[20,7]
   - 8.17438252315196 w[20,8] - 5.856581789606746 w[20,9]
   - 0.00142857142857143 w[20,10] - 13.96519426812116 w[20,11]
   - 2.524175583665984 w[20,12] - 8.17438252315196 w[20,13]
   - 5.856581789606746 w[20,14] - 0.00142857142857143 w[20,15]
   - 13.96519426812116 w[20,16] - 2.524175583665984 w[20,17]
   - 8.17438252315196 w[20,18] - 5.856581789606746 w[20,19]
   - 0.00142857142857143 w[20,20] - 13.96519426812116 w[20,21]
   - 2.524175583665984 w[20,22] - 1.380088031176086 w[20,23]
   - 1.380088031176086 w[20,24] - 1.380088031176086 w[20,25]
   - 12.72458926567911 w[21,1] - 12.72458926567911 w[21,2]
   - 10.24279337147347 w[21,3] - 8.109927967333126 w[21,4]
   - 13.96519426812116 w[21,5] - 0.00142857142857143 w[21,6]
   - 11.65633090491722 w[21,7] - 10.24279337147347 w[21,8]
   - 8.109927967333126 w[21,9] - 13.96519426812116 w[21,10]
   - 0.00142857142857143 w[21,11] - 11.65633090491722 w[21,12]
   - 10.24279337147347 w[21,13] - 8.109927967333126 w[21,14]
   - 13.96519426812116 w[21,15] - 0.00142857142857143 w[21,16]
   - 11.65633090491722 w[21,17] - 10.24279337147347 w[21,18]
   - 8.109927967333126 w[21,19] - 13.96519426812116 w[21,20]
   - 0.00142857142857143 w[21,21] - 11.65633090491722 w[21,22]
   - 12.72458926567911 w[21,23] - 12.72458926567911 w[21,24]
   - 12.72458926567911 w[21,25] - 1.862272848109308 w[22,1]
   - 1.862272848109308 w[22,2] - 5.980794808793723 w[22,3]
   - 3.608037782122388 w[22,4] - 2.524175583665984 w[22,5]
   - 11.65633090491722 w[22,6] - 0.00142857142857143 w[22,7]
   - 5.980794808793723 w[22,8] - 3.608037782122388 w[22,9]
   - 2.524175583665984 w[22,10] - 11.65633090491722 w[22,11]
   - 0.00142857142857143 w[22,12] - 5.980794808793723 w[22,13]
   - 3.608037782122388 w[22,14] - 2.524175583665984 w[22,15]
   - 11.65633090491722 w[22,16] - 0.00142857142857143 w[22,17]
   - 5.980794808793723 w[22,18] - 3.608037782122388 w[22,19]
   - 2.524175583665984 w[22,20] - 11.65633090491722 w[22,21]
   - 0.00142857142857143 w[22,22] - 1.862272848109308 w[22,23]
   - 1.862272848109308 w[22,24] - 1.862272848109308 w[22,25]
   - 0.00142857142857143 w[23,1] - 0.00142857142857143 w[23,2]
   - 7.841440394274109 w[23,3] - 4.650865590725213 w[23,4]
   - 1.380088031176086 w[23,5] - 12.72458926567911 w[23,6]
   - 1.862272848109308 w[23,7] - 7.841440394274109 w[23,8]
   - 4.650865590725213 w[23,9] - 1.380088031176086 w[23,10]
   - 12.72458926567911 w[23,11] - 1.862272848109308 w[23,12]
   - 7.841440394274109 w[23,13] - 4.650865590725213 w[23,14]
   - 1.380088031176086 w[23,15] - 12.72458926567911 w[23,16]
   - 1.862272848109308 w[23,17] - 7.841440394274109 w[23,18]
   - 4.650865590725213 w[23,19] - 1.380088031176086 w[23,20]
   - 12.72458926567911 w[23,21] - 1.862272848109308 w[23,22]
   - 0.00142857142857143 w[23,23] - 0.00142857142857143 w[23,24]
   - 0.00142857142857143 w[23,25] - 0.00142857142857143 w[24,1]
   - 0.00142857142857143 w[24,2] - 7.841440394274109 w[24,3]
   - 4.650865590725213 w[24,4] - 1.380088031176086 w[24,5]
   - 12.72458926567911 w[24,6] - 1.862272848109308 w[24,7]
   - 7.841440394274109 w[24,8] - 4.650865590725213 w[24,9]
   - 1.380088031176086 w[24,10] - 12.72458926567911 w[24,11]
   - 1.862272848109308 w[24,12] - 7.841440394274109 w[24,13]
   - 4.650865590725213 w[24,14] - 1.380088031176086 w[24,15]
   - 12.72458926567911 w[24,16] - 1.862272848109308 w[24,17]
   - 7.841440394274109 w[24,18] - 4.650865590725213 w[24,19]
   - 1.380088031176086 w[24,20] - 12.72458926567911 w[24,21]
   - 1.862272848109308 w[24,22] - 0.00142857142857143 w[24,23]
   - 0.00142857142857143 w[24,24] - 0.00142857142857143 w[24,25]
   - 0.1428571428571428 service_time[3,13,1]
   - 0.2857142857142857 service_time[3,13,2]
   - 0.1428571428571428 service_time[3,18,1]
   - 0.2857142857142857 service_time[3,18,2]
   - 0.1428571428571428 service_time[4,14,1]
   - 0.2857142857142857 service_time[4,14,2]
   - 0.1428571428571428 service_time[4,19,1]
   - 0.2857142857142857 service_time[4,19,2]
   - 0.1428571428571428 service_time[5,15,1]
   - 0.2857142857142857 service_time[5,15,2]
   - 0.1428571428571428 service_time[5,20,1]
   - 0.2857142857142857 service_time[5,20,2]
   - 0.1428571428571428 service_time[6,16,1]
   - 0.2857142857142857 service_time[6,16,2]
   - 0.1428571428571428 service_time[6,21,1]
   - 0.2857142857142857 service_time[6,21,2]
   - 0.1428571428571428 service_time[7,17,1]
   - 0.2857142857142857 service_time[7,17,2]
   - 0.1428571428571428 service_time[7,22,1]
   - 0.2857142857142857 service_time[7,22,2]
   - 0.1428571428571428 service_time[8,13,1]
   - 0.2857142857142857 service_time[8,13,2]
   - 0.1428571428571428 service_time[8,18,1]
   - 0.2857142857142857 service_time[8,18,2]
   - 0.1428571428571428 service_time[9,14,1]
   - 0.2857142857142857 service_time[9,14,2]
   - 0.1428571428571428 service_time[9,19,1]
   - 0.2857142857142857 service_time[9,19,2]
   - 0.1428571428571428 service_time[10,15,1]
   - 0.2857142857142857 service_time[10,15,2]
   - 0.1428571428571428 service_time[10,20,1]
   - 0.2857142857142857 service_time[10,20,2]
   - 0.1428571428571428 service_time[11,16,1]
   - 0.2857142857142857 service_time[11,16,2]
   - 0.1428571428571428 service_time[11,21,1]
   - 0.2857142857142857 service_time[11,21,2]
   - 0.1428571428571428 service_time[12,17,1]
   - 0.2857142857142857 service_time[12,17,2]
   - 0.1428571428571428 service_time[12,22,1]
   - 0.2857142857142857 service_time[12,22,2] + start_service[25] >= 0
 VehicleTimes[1]: - 0.01 x[0,1] - 0.01 x[0,2] - 54.89008275991876 x[0,3]
   - 32.5560591350765 x[0,4] - 9.660616218232601 x[0,5]
   - 89.07212485975379 x[0,6] - 13.03590993676516 x[0,7]
   - 54.89008275991876 x[0,13] - 32.5560591350765 x[0,14]
   - 9.660616218232601 x[0,15] - 89.07212485975379 x[0,16]
   - 13.03590993676516 x[0,17] - 0.01 x[0,23] - 0.01 x[0,24] - 0.01 x[0,25]
   - 0.01 x[1,1] - 0.01 x[1,2] - 54.89008275991876 x[1,3]
   - 32.5560591350765 x[1,4] - 9.660616218232601 x[1,5]
   - 89.07212485975379 x[1,6] - 13.03590993676516 x[1,7]
   - 54.89008275991876 x[1,13] - 32.5560591350765 x[1,14]
   - 9.660616218232601 x[1,15] - 89.07212485975379 x[1,16]
   - 13.03590993676516 x[1,17] - 0.01 x[1,23] - 0.01 x[1,24] - 0.01 x[1,25]
   - 0.01 x[2,1] - 0.01 x[2,2] - 54.89008275991876 x[2,3]
   - 32.5560591350765 x[2,4] - 9.660616218232601 x[2,5]
   - 89.07212485975379 x[2,6] - 13.03590993676516 x[2,7]
   - 54.89008275991876 x[2,13] - 32.5560591350765 x[2,14]
   - 9.660616218232601 x[2,15] - 89.07212485975379 x[2,16]
   - 13.03590993676516 x[2,17] - 0.01 x[2,23] - 0.01 x[2,24] - 0.01 x[2,25]
   - 54.89008275991876 x[3,1] - 54.89008275991876 x[3,2] - 0.01 x[3,3]
   - 40.93221513132295 x[3,4] - 57.22067766206373 x[3,5]
   - 71.69955360031429 x[3,6] - 41.86556366155607 x[3,7] - 0.01 x[3,13]
   - 40.93221513132295 x[3,14] - 57.22067766206373 x[3,15]
   - 71.69955360031429 x[3,16] - 41.86556366155607 x[3,17]
   - 54.89008275991876 x[3,23] - 54.89008275991876 x[3,24]
   - 54.89008275991876 x[3,25] - 32.5560591350765 x[4,1]
   - 32.5560591350765 x[4,2] - 40.93221513132295 x[4,3] - 0.01 x[4,4]
   - 40.99607252724722 x[4,5] - 56.76949577133189 x[4,6]
   - 25.25626447485672 x[4,7] - 40.93221513132295 x[4,13] - 0.01 x[4,14]
   - 40.99607252724722 x[4,15] - 56.76949577133189 x[4,16]
   - 25.25626447485672 x[4,17] - 32.5560591350765 x[4,23]
   - 32.5560591350765 x[4,24] - 32.5560591350765 x[4,25]
   - 9.660616218232601 x[5,1] - 9.660616218232601 x[5,2]
   - 57.22067766206373 x[5,3] - 40.99607252724722 x[5,4] - 0.01 x[5,5]
   - 97.75635987684809 x[5,6] - 17.66922908566189 x[5,7]
   - 57.22067766206373 x[5,13] - 40.99607252724722 x[5,14] - 0.01 x[5,15]
   - 97.75635987684809 x[5,16] - 17.66922908566189 x[5,17]
   - 9.660616218232601 x[5,23] - 9.660616218232601 x[5,24]
   - 9.660616218232601 x[5,25] - 89.07212485975379 x[6,1]
   - 89.07212485975379 x[6,2] - 71.69955360031429 x[6,3]
   - 56.76949577133189 x[6,4] - 97.75635987684809 x[6,5] - 0.01 x[6,6]
   - 81.59431633442051 x[6,7] - 71.69955360031429 x[6,13]
   - 56.76949577133189 x[6,14] - 97.75635987684809 x[6,15] - 0.01 x[6,16]
   - 81.59431633442051 x[6,17] - 89.07212485975379 x[6,23]
   - 89.07212485975379 x[6,24] - 89.07212485975379 x[6,25]
   - 13.03590993676516 x[7,1] - 13.03590993676516 x[7,2]
   - 41.86556366155607 x[7,3] - 25.25626447485672 x[7,4]
   - 17.66922908566189 x[7,5] - 81.59431633442051 x[7,6] - 0.01 x[7,7]
   - 41.86556366155607 x[7,13] - 25.25626447485672 x[7,14]
   - 17.66922908566189 x[7,15] - 81.59431633442051 x[7,16] - 0.01 x[7,17]
   - 13.03590993676516 x[7,23] - 13.03590993676516 x[7,24]
   - 13.03590993676516 x[7,25] - 54.89008275991876 x[13,1]
   - 54.89008275991876 x[13,2] - 0.01 x[13,3] - 40.93221513132295 x[13,4]
   - 57.22067766206373 x[13,5] - 71.69955360031429 x[13,6]
   - 41.86556366155607 x[13,7] - 0.01 x[13,13] - 40.93221513132295 x[13,14]
   - 57.22067766206373 x[13,15] - 71.69955360031429 x[13,16]
   - 41.86556366155607 x[13,17] - 54.89008275991876 x[13,23]
   - 54.89008275991876 x[13,24] - 54.89008275991876 x[13,25]
   - 32.5560591350765 x[14,1] - 32.5560591350765 x[14,2]
   - 40.93221513132295 x[14,3] - 0.01 x[14,4] - 40.99607252724722 x[14,5]
   - 56.76949577133189 x[14,6] - 25.25626447485672 x[14,7]
   - 40.93221513132295 x[14,13] - 0.01 x[14,14]
   - 40.99607252724722 x[14,15] - 56.76949577133189 x[14,16]
   - 25.25626447485672 x[14,17] - 32.5560591350765 x[14,23]
   - 32.5560591350765 x[14,24] - 32.5560591350765 x[14,25]
   - 9.660616218232601 x[15,1] - 9.660616218232601 x[15,2]
   - 57.22067766206373 x[15,3] - 40.99607252724722 x[15,4] - 0.01 x[15,5]
   - 97.75635987684809 x[15,6] - 17.66922908566189 x[15,7]
   - 57.22067766206373 x[15,13] - 40.99607252724722 x[15,14]
   - 0.01 x[15,15] - 97.75635987684809 x[15,16]
   - 17.66922908566189 x[15,17] - 9.660616218232601 x[15,23]
   - 9.660616218232601 x[15,24] - 9.660616218232601 x[15,25]
   - 89.07212485975379 x[16,1] - 89.07212485975379 x[16,2]
   - 71.69955360031429 x[16,3] - 56.76949577133189 x[16,4]
   - 97.75635987684809 x[16,5] - 0.01 x[16,6] - 81.59431633442051 x[16,7]
   - 71.69955360031429 x[16,13] - 56.76949577133189 x[16,14]
   - 97.75635987684809 x[16,15] - 0.01 x[16,16]
   - 81.59431633442051 x[16,17] - 89.07212485975379 x[16,23]
   - 89.07212485975379 x[16,24] - 89.07212485975379 x[16,25]
   - 13.03590993676516 x[17,1] - 13.03590993676516 x[17,2]
   - 41.86556366155607 x[17,3] - 25.25626447485672 x[17,4]
   - 17.66922908566189 x[17,5] - 81.59431633442051 x[17,6] - 0.01 x[17,7]
   - 41.86556366155607 x[17,13] - 25.25626447485672 x[17,14]
   - 17.66922908566189 x[17,15] - 81.59431633442051 x[17,16]
   - 0.01 x[17,17] - 13.03590993676516 x[17,23]
   - 13.03590993676516 x[17,24] - 13.03590993676516 x[17,25] - 0.01 x[23,1]
   - 0.01 x[23,2] - 54.89008275991876 x[23,3] - 32.5560591350765 x[23,4]
   - 9.660616218232601 x[23,5] - 89.07212485975379 x[23,6]
   - 13.03590993676516 x[23,7] - 54.89008275991876 x[23,13]
   - 32.5560591350765 x[23,14] - 9.660616218232601 x[23,15]
   - 89.07212485975379 x[23,16] - 13.03590993676516 x[23,17]
   - 0.01 x[23,23] - 0.01 x[23,24] - 0.01 x[23,25] - 0.01 x[24,1]
   - 0.01 x[24,2] - 54.89008275991876 x[24,3] - 32.5560591350765 x[24,4]
   - 9.660616218232601 x[24,5] - 89.07212485975379 x[24,6]
   - 13.03590993676516 x[24,7] - 54.89008275991876 x[24,13]
   - 32.5560591350765 x[24,14] - 9.660616218232601 x[24,15]
   - 89.07212485975379 x[24,16] - 13.03590993676516 x[24,17]
   - 0.01 x[24,23] - 0.01 x[24,24] - 0.01 x[24,25] + start_service[25]
   >= 0
 VehicleTimes[2]: - 0.01 x[0,1] - 0.01 x[0,2] - 54.89008275991876 x[0,8]
   - 32.5560591350765 x[0,9] - 9.660616218232601 x[0,10]
   - 89.07212485975379 x[0,11] - 13.03590993676516 x[0,12]
   - 54.89008275991876 x[0,18] - 32.5560591350765 x[0,19]
   - 9.660616218232601 x[0,20] - 89.07212485975379 x[0,21]
   - 13.03590993676516 x[0,22] - 0.01 x[0,23] - 0.01 x[0,24] - 0.01 x[0,25]
   - 0.01 x[1,1] - 0.01 x[1,2] - 54.89008275991876 x[1,8]
   - 32.5560591350765 x[1,9] - 9.660616218232601 x[1,10]
   - 89.07212485975379 x[1,11] - 13.03590993676516 x[1,12]
   - 54.89008275991876 x[1,18] - 32.5560591350765 x[1,19]
   - 9.660616218232601 x[1,20] - 89.07212485975379 x[1,21]
   - 13.03590993676516 x[1,22] - 0.01 x[1,23] - 0.01 x[1,24] - 0.01 x[1,25]
   - 0.01 x[2,1] - 0.01 x[2,2] - 54.89008275991876 x[2,8]
   - 32.5560591350765 x[2,9] - 9.660616218232601 x[2,10]
   - 89.07212485975379 x[2,11] - 13.03590993676516 x[2,12]
   - 54.89008275991876 x[2,18] - 32.5560591350765 x[2,19]
   - 9.660616218232601 x[2,20] - 89.07212485975379 x[2,21]
   - 13.03590993676516 x[2,22] - 0.01 x[2,23] - 0.01 x[2,24] - 0.01 x[2,25]
   - 54.89008275991876 x[8,1] - 54.89008275991876 x[8,2] - 0.01 x[8,8]
   - 40.93221513132295 x[8,9] - 57.22067766206373 x[8,10]
   - 71.69955360031429 x[8,11] - 41.86556366155607 x[8,12] - 0.01 x[8,18]
   - 40.93221513132295 x[8,19] - 57.22067766206373 x[8,20]
   - 71.69955360031429 x[8,21] - 41.86556366155607 x[8,22]
   - 54.89008275991876 x[8,23] - 54.89008275991876 x[8,24]
   - 54.89008275991876 x[8,25] - 32.5560591350765 x[9,1]
   - 32.5560591350765 x[9,2] - 40.93221513132295 x[9,8] - 0.01 x[9,9]
   - 40.99607252724722 x[9,10] - 56.76949577133189 x[9,11]
   - 25.25626447485672 x[9,12] - 40.93221513132295 x[9,18] - 0.01 x[9,19]
   - 40.99607252724722 x[9,20] - 56.76949577133189 x[9,21]
   - 25.25626447485672 x[9,22] - 32.5560591350765 x[9,23]
   - 32.5560591350765 x[9,24] - 32.5560591350765 x[9,25]
   - 9.660616218232601 x[10,1] - 9.660616218232601 x[10,2]
   - 57.22067766206373 x[10,8] - 40.99607252724722 x[10,9] - 0.01 x[10,10]
   - 97.75635987684809 x[10,11] - 17.66922908566189 x[10,12]
   - 57.22067766206373 x[10,18] - 40.99607252724722 x[10,19]
   - 0.01 x[10,20] - 97.75635987684809 x[10,21]
   - 17.66922908566189 x[10,22] - 9.660616218232601 x[10,23]
   - 9.660616218232601 x[10,24] - 9.660616218232601 x[10,25]
   - 89.07212485975379 x[11,1] - 89.07212485975379 x[11,2]
   - 71.69955360031429 x[11,8] - 56.76949577133189 x[11,9]
   - 97.75635987684809 x[11,10] - 0.01 x[11,11]
   - 81.59431633442051 x[11,12] - 71.69955360031429 x[11,18]
   - 56.76949577133189 x[11,19] - 97.75635987684809 x[11,20]
   - 0.01 x[11,21] - 81.59431633442051 x[11,22]
   - 89.07212485975379 x[11,23] - 89.07212485975379 x[11,24]
   - 89.07212485975379 x[11,25] - 13.03590993676516 x[12,1]
   - 13.03590993676516 x[12,2] - 41.86556366155607 x[12,8]
   - 25.25626447485672 x[12,9] - 17.66922908566189 x[12,10]
   - 81.59431633442051 x[12,11] - 0.01 x[12,12]
   - 41.86556366155607 x[12,18] - 25.25626447485672 x[12,19]
   - 17.66922908566189 x[12,20] - 81.59431633442051 x[12,21]
   - 0.01 x[12,22] - 13.03590993676516 x[12,23]
   - 13.03590993676516 x[12,24] - 13.03590993676516 x[12,25]
   - 54.89008275991876 x[18,1] - 54.89008275991876 x[18,2] - 0.01 x[18,8]
   - 40.93221513132295 x[18,9] - 57.22067766206373 x[18,10]
   - 71.69955360031429 x[18,11] - 41.86556366155607 x[18,12]
   - 0.01 x[18,18] - 40.93221513132295 x[18,19]
   - 57.22067766206373 x[18,20] - 71.69955360031429 x[18,21]
   - 41.86556366155607 x[18,22] - 54.89008275991876 x[18,23]
   - 54.89008275991876 x[18,24] - 54.89008275991876 x[18,25]
   - 32.5560591350765 x[19,1] - 32.5560591350765 x[19,2]
   - 40.93221513132295 x[19,8] - 0.01 x[19,9] - 40.99607252724722 x[19,10]
   - 56.76949577133189 x[19,11] - 25.25626447485672 x[19,12]
   - 40.93221513132295 x[19,18] - 0.01 x[19,19]
   - 40.99607252724722 x[19,20] - 56.76949577133189 x[19,21]
   - 25.25626447485672 x[19,22] - 32.5560591350765 x[19,23]
   - 32.5560591350765 x[19,24] - 32.5560591350765 x[19,25]
   - 9.660616218232601 x[20,1] - 9.660616218232601 x[20,2]
   - 57.22067766206373 x[20,8] - 40.99607252724722 x[20,9] - 0.01 x[20,10]
   - 97.75635987684809 x[20,11] - 17.66922908566189 x[20,12]
   - 57.22067766206373 x[20,18] - 40.99607252724722 x[20,19]
   - 0.01 x[20,20] - 97.75635987684809 x[20,21]
   - 17.66922908566189 x[20,22] - 9.660616218232601 x[20,23]
   - 9.660616218232601 x[20,24] - 9.660616218232601 x[20,25]
   - 89.07212485975379 x[21,1] - 89.07212485975379 x[21,2]
   - 71.69955360031429 x[21,8] - 56.76949577133189 x[21,9]
   - 97.75635987684809 x[21,10] - 0.01 x[21,11]
   - 81.59431633442051 x[21,12] - 71.69955360031429 x[21,18]
   - 56.76949577133189 x[21,19] - 97.75635987684809 x[21,20]
   - 0.01 x[21,21] - 81.59431633442051 x[21,22]
   - 89.07212485975379 x[21,23] - 89.07212485975379 x[21,24]
   - 89.07212485975379 x[21,25] - 13.03590993676516 x[22,1]
   - 13.03590993676516 x[22,2] - 41.86556366155607 x[22,8]
   - 25.25626447485672 x[22,9] - 17.66922908566189 x[22,10]
   - 81.59431633442051 x[22,11] - 0.01 x[22,12]
   - 41.86556366155607 x[22,18] - 25.25626447485672 x[22,19]
   - 17.66922908566189 x[22,20] - 81.59431633442051 x[22,21]
   - 0.01 x[22,22] - 13.03590993676516 x[22,23]
   - 13.03590993676516 x[22,24] - 13.03590993676516 x[22,25] - 0.01 x[23,1]
   - 0.01 x[23,2] - 54.89008275991876 x[23,8] - 32.5560591350765 x[23,9]
   - 9.660616218232601 x[23,10] - 89.07212485975379 x[23,11]
   - 13.03590993676516 x[23,12] - 54.89008275991876 x[23,18]
   - 32.5560591350765 x[23,19] - 9.660616218232601 x[23,20]
   - 89.07212485975379 x[23,21] - 13.03590993676516 x[23,22]
   - 0.01 x[23,23] - 0.01 x[23,24] - 0.01 x[23,25] - 0.01 x[24,1]
   - 0.01 x[24,2] - 54.89008275991876 x[24,8] - 32.5560591350765 x[24,9]
   - 9.660616218232601 x[24,10] - 89.07212485975379 x[24,11]
   - 13.03590993676516 x[24,12] - 54.89008275991876 x[24,18]
   - 32.5560591350765 x[24,19] - 9.660616218232601 x[24,20]
   - 89.07212485975379 x[24,21] - 13.03590993676516 x[24,22]
   - 0.01 x[24,23] - 0.01 x[24,24] - 0.01 x[24,25] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[0]: - 0.02 x[0,1] - 0.02 x[0,2]
   - 109.7801655198375 x[0,3] - 65.11211827015299 x[0,4]
   - 19.3212324364652 x[0,5] - 178.1442497195076 x[0,6]
   - 26.07181987353031 x[0,7] - 109.7801655198375 x[0,8]
   - 65.11211827015299 x[0,9] - 19.3212324364652 x[0,10]
   - 178.1442497195076 x[0,11] - 26.07181987353031 x[0,12]
   - 109.7801655198375 x[0,13] - 65.11211827015299 x[0,14]
   - 19.3212324364652 x[0,15] - 178.1442497195076 x[0,16]
   - 26.07181987353031 x[0,17] - 109.7801655198375 x[0,18]
   - 65.11211827015299 x[0,19] - 19.3212324364652 x[0,20]
   - 178.1442497195076 x[0,21] - 26.07181987353031 x[0,22] - 0.02 x[0,23]
   - 0.02 x[0,24] - 0.02 x[0,25] - start_service[0] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[1]: - 0.02 x[1,1] - 0.02 x[1,2]
   - 109.7801655198375 x[1,3] - 65.11211827015299 x[1,4]
   - 19.3212324364652 x[1,5] - 178.1442497195076 x[1,6]
   - 26.07181987353031 x[1,7] - 109.7801655198375 x[1,8]
   - 65.11211827015299 x[1,9] - 19.3212324364652 x[1,10]
   - 178.1442497195076 x[1,11] - 26.07181987353031 x[1,12]
   - 109.7801655198375 x[1,13] - 65.11211827015299 x[1,14]
   - 19.3212324364652 x[1,15] - 178.1442497195076 x[1,16]
   - 26.07181987353031 x[1,17] - 109.7801655198375 x[1,18]
   - 65.11211827015299 x[1,19] - 19.3212324364652 x[1,20]
   - 178.1442497195076 x[1,21] - 26.07181987353031 x[1,22] - 0.02 x[1,23]
   - 0.02 x[1,24] - 0.02 x[1,25] - start_service[1] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[2]: - 0.02 x[2,1] - 0.02 x[2,2]
   - 109.7801655198375 x[2,3] - 65.11211827015299 x[2,4]
   - 19.3212324364652 x[2,5] - 178.1442497195076 x[2,6]
   - 26.07181987353031 x[2,7] - 109.7801655198375 x[2,8]
   - 65.11211827015299 x[2,9] - 19.3212324364652 x[2,10]
   - 178.1442497195076 x[2,11] - 26.07181987353031 x[2,12]
   - 109.7801655198375 x[2,13] - 65.11211827015299 x[2,14]
   - 19.3212324364652 x[2,15] - 178.1442497195076 x[2,16]
   - 26.07181987353031 x[2,17] - 109.7801655198375 x[2,18]
   - 65.11211827015299 x[2,19] - 19.3212324364652 x[2,20]
   - 178.1442497195076 x[2,21] - 26.07181987353031 x[2,22] - 0.02 x[2,23]
   - 0.02 x[2,24] - 0.02 x[2,25] - start_service[2] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[3]: - 54.90008275991876 x[3,1]
   - 54.90008275991876 x[3,2] - 54.90008275991876 x[3,3]
   - 73.48827426639944 x[3,4] - 66.88129388029633 x[3,5]
   - 160.7716784600681 x[3,6] - 54.90147359832122 x[3,7]
   - 54.90008275991876 x[3,8] - 73.48827426639944 x[3,9]
   - 66.88129388029633 x[3,10] - 160.7716784600681 x[3,11]
   - 54.90147359832122 x[3,12] - 54.90008275991876 x[3,13]
   - 73.48827426639944 x[3,14] - 66.88129388029633 x[3,15]
   - 160.7716784600681 x[3,16] - 54.90147359832122 x[3,17]
   - 54.90008275991876 x[3,18] - 73.48827426639944 x[3,19]
   - 66.88129388029633 x[3,20] - 160.7716784600681 x[3,21]
   - 54.90147359832122 x[3,22] - 54.90008275991876 x[3,23]
   - 54.90008275991876 x[3,24] - 54.90008275991876 x[3,25]
   - start_service[3] + start_service[25] >= 0
 LowerBoundRoutingVehicle[4]: - 32.56605913507649 x[4,1]
   - 32.56605913507649 x[4,2] - 95.8222978912417 x[4,3]
   - 32.56605913507649 x[4,4] - 50.65668874547983 x[4,5]
   - 145.8416206310857 x[4,6] - 38.29217441162187 x[4,7]
   - 95.8222978912417 x[4,8] - 32.56605913507649 x[4,9]
   - 50.65668874547983 x[4,10] - 145.8416206310857 x[4,11]
   - 38.29217441162187 x[4,12] - 95.8222978912417 x[4,13]
   - 32.56605913507649 x[4,14] - 50.65668874547983 x[4,15]
   - 145.8416206310857 x[4,16] - 38.29217441162187 x[4,17]
   - 95.8222978912417 x[4,18] - 32.56605913507649 x[4,19]
   - 50.65668874547983 x[4,20] - 145.8416206310857 x[4,21]
   - 38.29217441162187 x[4,22] - 32.56605913507649 x[4,23]
   - 32.56605913507649 x[4,24] - 32.56605913507649 x[4,25]
   - start_service[4] + start_service[25] >= 0
 LowerBoundRoutingVehicle[5]: - 9.670616218232601 x[5,1]
   - 9.670616218232601 x[5,2] - 112.1107604219825 x[5,3]
   - 73.55213166232372 x[5,4] - 9.670616218232601 x[5,5]
   - 186.8284847366019 x[5,6] - 30.70513902242705 x[5,7]
   - 112.1107604219825 x[5,8] - 73.55213166232372 x[5,9]
   - 9.670616218232601 x[5,10] - 186.8284847366019 x[5,11]
   - 30.70513902242705 x[5,12] - 112.1107604219825 x[5,13]
   - 73.55213166232372 x[5,14] - 9.670616218232601 x[5,15]
   - 186.8284847366019 x[5,16] - 30.70513902242705 x[5,17]
   - 112.1107604219825 x[5,18] - 73.55213166232372 x[5,19]
   - 9.670616218232601 x[5,20] - 186.8284847366019 x[5,21]
   - 30.70513902242705 x[5,22] - 9.670616218232601 x[5,23]
   - 9.670616218232601 x[5,24] - 9.670616218232601 x[5,25]
   - start_service[5] + start_service[25] >= 0
 LowerBoundRoutingVehicle[6]: - 89.08212485975379 x[6,1]
   - 89.08212485975379 x[6,2] - 126.589636360233 x[6,3]
   - 89.32555490640839 x[6,4] - 107.4169760950807 x[6,5]
   - 89.08212485975379 x[6,6] - 94.63022627118566 x[6,7]
   - 126.589636360233 x[6,8] - 89.32555490640839 x[6,9]
   - 107.4169760950807 x[6,10] - 89.08212485975379 x[6,11]
   - 94.63022627118566 x[6,12] - 126.589636360233 x[6,13]
   - 89.32555490640839 x[6,14] - 107.4169760950807 x[6,15]
   - 89.08212485975379 x[6,16] - 94.63022627118566 x[6,17]
   - 126.589636360233 x[6,18] - 89.32555490640839 x[6,19]
   - 107.4169760950807 x[6,20] - 89.08212485975379 x[6,21]
   - 94.63022627118566 x[6,22] - 89.08212485975379 x[6,23]
   - 89.08212485975379 x[6,24] - 89.08212485975379 x[6,25]
   - start_service[6] + start_service[25] >= 0
 LowerBoundRoutingVehicle[7]: - 13.04590993676516 x[7,1]
   - 13.04590993676516 x[7,2] - 96.75564642147484 x[7,3]
   - 57.81232360993322 x[7,4] - 27.32984530389449 x[7,5]
   - 170.6664411941743 x[7,6] - 13.04590993676516 x[7,7]
   - 96.75564642147484 x[7,8] - 57.81232360993322 x[7,9]
   - 27.32984530389449 x[7,10] - 170.6664411941743 x[7,11]
   - 13.04590993676516 x[7,12] - 96.75564642147484 x[7,13]
   - 57.81232360993322 x[7,14] - 27.32984530389449 x[7,15]
   - 170.6664411941743 x[7,16] - 13.04590993676516 x[7,17]
   - 96.75564642147484 x[7,18] - 57.81232360993322 x[7,19]
   - 27.32984530389449 x[7,20] - 170.6664411941743 x[7,21]
   - 13.04590993676516 x[7,22] - 13.04590993676516 x[7,23]
   - 13.04590993676516 x[7,24] - 13.04590993676516 x[7,25]
   - start_service[7] + start_service[25] >= 0
 LowerBoundRoutingVehicle[8]: - 54.90008275991876 x[8,1]
   - 54.90008275991876 x[8,2] - 54.90008275991876 x[8,3]
   - 73.48827426639944 x[8,4] - 66.88129388029633 x[8,5]
   - 160.7716784600681 x[8,6] - 54.90147359832122 x[8,7]
   - 54.90008275991876 x[8,8] - 73.48827426639944 x[8,9]
   - 66.88129388029633 x[8,10] - 160.7716784600681 x[8,11]
   - 54.90147359832122 x[8,12] - 54.90008275991876 x[8,13]
   - 73.48827426639944 x[8,14] - 66.88129388029633 x[8,15]
   - 160.7716784600681 x[8,16] - 54.90147359832122 x[8,17]
   - 54.90008275991876 x[8,18] - 73.48827426639944 x[8,19]
   - 66.88129388029633 x[8,20] - 160.7716784600681 x[8,21]
   - 54.90147359832122 x[8,22] - 54.90008275991876 x[8,23]
   - 54.90008275991876 x[8,24] - 54.90008275991876 x[8,25]
   - start_service[8] + start_service[25] >= 0
 LowerBoundRoutingVehicle[9]: - 32.56605913507649 x[9,1]
   - 32.56605913507649 x[9,2] - 95.8222978912417 x[9,3]
   - 32.56605913507649 x[9,4] - 50.65668874547983 x[9,5]
   - 145.8416206310857 x[9,6] - 38.29217441162187 x[9,7]
   - 95.8222978912417 x[9,8] - 32.56605913507649 x[9,9]
   - 50.65668874547983 x[9,10] - 145.8416206310857 x[9,11]
   - 38.29217441162187 x[9,12] - 95.8222978912417 x[9,13]
   - 32.56605913507649 x[9,14] - 50.65668874547983 x[9,15]
   - 145.8416206310857 x[9,16] - 38.29217441162187 x[9,17]
   - 95.8222978912417 x[9,18] - 32.56605913507649 x[9,19]
   - 50.65668874547983 x[9,20] - 145.8416206310857 x[9,21]
   - 38.29217441162187 x[9,22] - 32.56605913507649 x[9,23]
   - 32.56605913507649 x[9,24] - 32.56605913507649 x[9,25]
   - start_service[9] + start_service[25] >= 0
 LowerBoundRoutingVehicle[10]: - 9.670616218232601 x[10,1]
   - 9.670616218232601 x[10,2] - 112.1107604219825 x[10,3]
   - 73.55213166232372 x[10,4] - 9.670616218232601 x[10,5]
   - 186.8284847366019 x[10,6] - 30.70513902242705 x[10,7]
   - 112.1107604219825 x[10,8] - 73.55213166232372 x[10,9]
   - 9.670616218232601 x[10,10] - 186.8284847366019 x[10,11]
   - 30.70513902242705 x[10,12] - 112.1107604219825 x[10,13]
   - 73.55213166232372 x[10,14] - 9.670616218232601 x[10,15]
   - 186.8284847366019 x[10,16] - 30.70513902242705 x[10,17]
   - 112.1107604219825 x[10,18] - 73.55213166232372 x[10,19]
   - 9.670616218232601 x[10,20] - 186.8284847366019 x[10,21]
   - 30.70513902242705 x[10,22] - 9.670616218232601 x[10,23]
   - 9.670616218232601 x[10,24] - 9.670616218232601 x[10,25]
   - start_service[10] + start_service[25] >= 0
 LowerBoundRoutingVehicle[11]: - 89.08212485975379 x[11,1]
   - 89.08212485975379 x[11,2] - 126.589636360233 x[11,3]
   - 89.32555490640839 x[11,4] - 107.4169760950807 x[11,5]
   - 89.08212485975379 x[11,6] - 94.63022627118566 x[11,7]
   - 126.589636360233 x[11,8] - 89.32555490640839 x[11,9]
   - 107.4169760950807 x[11,10] - 89.08212485975379 x[11,11]
   - 94.63022627118566 x[11,12] - 126.589636360233 x[11,13]
   - 89.32555490640839 x[11,14] - 107.4169760950807 x[11,15]
   - 89.08212485975379 x[11,16] - 94.63022627118566 x[11,17]
   - 126.589636360233 x[11,18] - 89.32555490640839 x[11,19]
   - 107.4169760950807 x[11,20] - 89.08212485975379 x[11,21]
   - 94.63022627118566 x[11,22] - 89.08212485975379 x[11,23]
   - 89.08212485975379 x[11,24] - 89.08212485975379 x[11,25]
   - start_service[11] + start_service[25] >= 0
 LowerBoundRoutingVehicle[12]: - 13.04590993676516 x[12,1]
   - 13.04590993676516 x[12,2] - 96.75564642147484 x[12,3]
   - 57.81232360993322 x[12,4] - 27.32984530389449 x[12,5]
   - 170.6664411941743 x[12,6] - 13.04590993676516 x[12,7]
   - 96.75564642147484 x[12,8] - 57.81232360993322 x[12,9]
   - 27.32984530389449 x[12,10] - 170.6664411941743 x[12,11]
   - 13.04590993676516 x[12,12] - 96.75564642147484 x[12,13]
   - 57.81232360993322 x[12,14] - 27.32984530389449 x[12,15]
   - 170.6664411941743 x[12,16] - 13.04590993676516 x[12,17]
   - 96.75564642147484 x[12,18] - 57.81232360993322 x[12,19]
   - 27.32984530389449 x[12,20] - 170.6664411941743 x[12,21]
   - 13.04590993676516 x[12,22] - 13.04590993676516 x[12,23]
   - 13.04590993676516 x[12,24] - 13.04590993676516 x[12,25]
   - start_service[12] + start_service[25] >= 0
 LowerBoundRoutingVehicle[13]: - 54.90008275991876 x[13,1]
   - 54.90008275991876 x[13,2] - 54.90008275991876 x[13,3]
   - 73.48827426639944 x[13,4] - 66.88129388029633 x[13,5]
   - 160.7716784600681 x[13,6] - 54.90147359832122 x[13,7]
   - 54.90008275991876 x[13,8] - 73.48827426639944 x[13,9]
   - 66.88129388029633 x[13,10] - 160.7716784600681 x[13,11]
   - 54.90147359832122 x[13,12] - 54.90008275991876 x[13,13]
   - 73.48827426639944 x[13,14] - 66.88129388029633 x[13,15]
   - 160.7716784600681 x[13,16] - 54.90147359832122 x[13,17]
   - 54.90008275991876 x[13,18] - 73.48827426639944 x[13,19]
   - 66.88129388029633 x[13,20] - 160.7716784600681 x[13,21]
   - 54.90147359832122 x[13,22] - 54.90008275991876 x[13,23]
   - 54.90008275991876 x[13,24] - 54.90008275991876 x[13,25]
   - start_service[13] + start_service[25] >= 0
 LowerBoundRoutingVehicle[14]: - 32.56605913507649 x[14,1]
   - 32.56605913507649 x[14,2] - 95.8222978912417 x[14,3]
   - 32.56605913507649 x[14,4] - 50.65668874547983 x[14,5]
   - 145.8416206310857 x[14,6] - 38.29217441162187 x[14,7]
   - 95.8222978912417 x[14,8] - 32.56605913507649 x[14,9]
   - 50.65668874547983 x[14,10] - 145.8416206310857 x[14,11]
   - 38.29217441162187 x[14,12] - 95.8222978912417 x[14,13]
   - 32.56605913507649 x[14,14] - 50.65668874547983 x[14,15]
   - 145.8416206310857 x[14,16] - 38.29217441162187 x[14,17]
   - 95.8222978912417 x[14,18] - 32.56605913507649 x[14,19]
   - 50.65668874547983 x[14,20] - 145.8416206310857 x[14,21]
   - 38.29217441162187 x[14,22] - 32.56605913507649 x[14,23]
   - 32.56605913507649 x[14,24] - 32.56605913507649 x[14,25]
   - start_service[14] + start_service[25] >= 0
 LowerBoundRoutingVehicle[15]: - 9.670616218232601 x[15,1]
   - 9.670616218232601 x[15,2] - 112.1107604219825 x[15,3]
   - 73.55213166232372 x[15,4] - 9.670616218232601 x[15,5]
   - 186.8284847366019 x[15,6] - 30.70513902242705 x[15,7]
   - 112.1107604219825 x[15,8] - 73.55213166232372 x[15,9]
   - 9.670616218232601 x[15,10] - 186.8284847366019 x[15,11]
   - 30.70513902242705 x[15,12] - 112.1107604219825 x[15,13]
   - 73.55213166232372 x[15,14] - 9.670616218232601 x[15,15]
   - 186.8284847366019 x[15,16] - 30.70513902242705 x[15,17]
   - 112.1107604219825 x[15,18] - 73.55213166232372 x[15,19]
   - 9.670616218232601 x[15,20] - 186.8284847366019 x[15,21]
   - 30.70513902242705 x[15,22] - 9.670616218232601 x[15,23]
   - 9.670616218232601 x[15,24] - 9.670616218232601 x[15,25]
   - start_service[15] + start_service[25] >= 0
 LowerBoundRoutingVehicle[16]: - 89.08212485975379 x[16,1]
   - 89.08212485975379 x[16,2] - 126.589636360233 x[16,3]
   - 89.32555490640839 x[16,4] - 107.4169760950807 x[16,5]
   - 89.08212485975379 x[16,6] - 94.63022627118566 x[16,7]
   - 126.589636360233 x[16,8] - 89.32555490640839 x[16,9]
   - 107.4169760950807 x[16,10] - 89.08212485975379 x[16,11]
   - 94.63022627118566 x[16,12] - 126.589636360233 x[16,13]
   - 89.32555490640839 x[16,14] - 107.4169760950807 x[16,15]
   - 89.08212485975379 x[16,16] - 94.63022627118566 x[16,17]
   - 126.589636360233 x[16,18] - 89.32555490640839 x[16,19]
   - 107.4169760950807 x[16,20] - 89.08212485975379 x[16,21]
   - 94.63022627118566 x[16,22] - 89.08212485975379 x[16,23]
   - 89.08212485975379 x[16,24] - 89.08212485975379 x[16,25]
   - start_service[16] + start_service[25] >= 0
 LowerBoundRoutingVehicle[17]: - 13.04590993676516 x[17,1]
   - 13.04590993676516 x[17,2] - 96.75564642147484 x[17,3]
   - 57.81232360993322 x[17,4] - 27.32984530389449 x[17,5]
   - 170.6664411941743 x[17,6] - 13.04590993676516 x[17,7]
   - 96.75564642147484 x[17,8] - 57.81232360993322 x[17,9]
   - 27.32984530389449 x[17,10] - 170.6664411941743 x[17,11]
   - 13.04590993676516 x[17,12] - 96.75564642147484 x[17,13]
   - 57.81232360993322 x[17,14] - 27.32984530389449 x[17,15]
   - 170.6664411941743 x[17,16] - 13.04590993676516 x[17,17]
   - 96.75564642147484 x[17,18] - 57.81232360993322 x[17,19]
   - 27.32984530389449 x[17,20] - 170.6664411941743 x[17,21]
   - 13.04590993676516 x[17,22] - 13.04590993676516 x[17,23]
   - 13.04590993676516 x[17,24] - 13.04590993676516 x[17,25]
   - start_service[17] + start_service[25] >= 0
 LowerBoundRoutingVehicle[18]: - 54.90008275991876 x[18,1]
   - 54.90008275991876 x[18,2] - 54.90008275991876 x[18,3]
   - 73.48827426639944 x[18,4] - 66.88129388029633 x[18,5]
   - 160.7716784600681 x[18,6] - 54.90147359832122 x[18,7]
   - 54.90008275991876 x[18,8] - 73.48827426639944 x[18,9]
   - 66.88129388029633 x[18,10] - 160.7716784600681 x[18,11]
   - 54.90147359832122 x[18,12] - 54.90008275991876 x[18,13]
   - 73.48827426639944 x[18,14] - 66.88129388029633 x[18,15]
   - 160.7716784600681 x[18,16] - 54.90147359832122 x[18,17]
   - 54.90008275991876 x[18,18] - 73.48827426639944 x[18,19]
   - 66.88129388029633 x[18,20] - 160.7716784600681 x[18,21]
   - 54.90147359832122 x[18,22] - 54.90008275991876 x[18,23]
   - 54.90008275991876 x[18,24] - 54.90008275991876 x[18,25]
   - start_service[18] + start_service[25] >= 0
 LowerBoundRoutingVehicle[19]: - 32.56605913507649 x[19,1]
   - 32.56605913507649 x[19,2] - 95.8222978912417 x[19,3]
   - 32.56605913507649 x[19,4] - 50.65668874547983 x[19,5]
   - 145.8416206310857 x[19,6] - 38.29217441162187 x[19,7]
   - 95.8222978912417 x[19,8] - 32.56605913507649 x[19,9]
   - 50.65668874547983 x[19,10] - 145.8416206310857 x[19,11]
   - 38.29217441162187 x[19,12] - 95.8222978912417 x[19,13]
   - 32.56605913507649 x[19,14] - 50.65668874547983 x[19,15]
   - 145.8416206310857 x[19,16] - 38.29217441162187 x[19,17]
   - 95.8222978912417 x[19,18] - 32.56605913507649 x[19,19]
   - 50.65668874547983 x[19,20] - 145.8416206310857 x[19,21]
   - 38.29217441162187 x[19,22] - 32.56605913507649 x[19,23]
   - 32.56605913507649 x[19,24] - 32.56605913507649 x[19,25]
   - start_service[19] + start_service[25] >= 0
 LowerBoundRoutingVehicle[20]: - 9.670616218232601 x[20,1]
   - 9.670616218232601 x[20,2] - 112.1107604219825 x[20,3]
   - 73.55213166232372 x[20,4] - 9.670616218232601 x[20,5]
   - 186.8284847366019 x[20,6] - 30.70513902242705 x[20,7]
   - 112.1107604219825 x[20,8] - 73.55213166232372 x[20,9]
   - 9.670616218232601 x[20,10] - 186.8284847366019 x[20,11]
   - 30.70513902242705 x[20,12] - 112.1107604219825 x[20,13]
   - 73.55213166232372 x[20,14] - 9.670616218232601 x[20,15]
   - 186.8284847366019 x[20,16] - 30.70513902242705 x[20,17]
   - 112.1107604219825 x[20,18] - 73.55213166232372 x[20,19]
   - 9.670616218232601 x[20,20] - 186.8284847366019 x[20,21]
   - 30.70513902242705 x[20,22] - 9.670616218232601 x[20,23]
   - 9.670616218232601 x[20,24] - 9.670616218232601 x[20,25]
   - start_service[20] + start_service[25] >= 0
 LowerBoundRoutingVehicle[21]: - 89.08212485975379 x[21,1]
   - 89.08212485975379 x[21,2] - 126.589636360233 x[21,3]
   - 89.32555490640839 x[21,4] - 107.4169760950807 x[21,5]
   - 89.08212485975379 x[21,6] - 94.63022627118566 x[21,7]
   - 126.589636360233 x[21,8] - 89.32555490640839 x[21,9]
   - 107.4169760950807 x[21,10] - 89.08212485975379 x[21,11]
   - 94.63022627118566 x[21,12] - 126.589636360233 x[21,13]
   - 89.32555490640839 x[21,14] - 107.4169760950807 x[21,15]
   - 89.08212485975379 x[21,16] - 94.63022627118566 x[21,17]
   - 126.589636360233 x[21,18] - 89.32555490640839 x[21,19]
   - 107.4169760950807 x[21,20] - 89.08212485975379 x[21,21]
   - 94.63022627118566 x[21,22] - 89.08212485975379 x[21,23]
   - 89.08212485975379 x[21,24] - 89.08212485975379 x[21,25]
   - start_service[21] + start_service[25] >= 0
 LowerBoundRoutingVehicle[22]: - 13.04590993676516 x[22,1]
   - 13.04590993676516 x[22,2] - 96.75564642147484 x[22,3]
   - 57.81232360993322 x[22,4] - 27.32984530389449 x[22,5]
   - 170.6664411941743 x[22,6] - 13.04590993676516 x[22,7]
   - 96.75564642147484 x[22,8] - 57.81232360993322 x[22,9]
   - 27.32984530389449 x[22,10] - 170.6664411941743 x[22,11]
   - 13.04590993676516 x[22,12] - 96.75564642147484 x[22,13]
   - 57.81232360993322 x[22,14] - 27.32984530389449 x[22,15]
   - 170.6664411941743 x[22,16] - 13.04590993676516 x[22,17]
   - 96.75564642147484 x[22,18] - 57.81232360993322 x[22,19]
   - 27.32984530389449 x[22,20] - 170.6664411941743 x[22,21]
   - 13.04590993676516 x[22,22] - 13.04590993676516 x[22,23]
   - 13.04590993676516 x[22,24] - 13.04590993676516 x[22,25]
   - start_service[22] + start_service[25] >= 0
 LowerBoundRoutingVehicle[23]: - 0.02 x[23,1] - 0.02 x[23,2]
   - 109.7801655198375 x[23,3] - 65.11211827015299 x[23,4]
   - 19.3212324364652 x[23,5] - 178.1442497195076 x[23,6]
   - 26.07181987353031 x[23,7] - 109.7801655198375 x[23,8]
   - 65.11211827015299 x[23,9] - 19.3212324364652 x[23,10]
   - 178.1442497195076 x[23,11] - 26.07181987353031 x[23,12]
   - 109.7801655198375 x[23,13] - 65.11211827015299 x[23,14]
   - 19.3212324364652 x[23,15] - 178.1442497195076 x[23,16]
   - 26.07181987353031 x[23,17] - 109.7801655198375 x[23,18]
   - 65.11211827015299 x[23,19] - 19.3212324364652 x[23,20]
   - 178.1442497195076 x[23,21] - 26.07181987353031 x[23,22]
   - 0.02 x[23,23] - 0.02 x[23,24] - 0.02 x[23,25] - start_service[23]
   + start_service[25] >= 0
 LowerBoundRoutingVehicle[24]: - 0.02 x[24,1] - 0.02 x[24,2]
   - 109.7801655198375 x[24,3] - 65.11211827015299 x[24,4]
   - 19.3212324364652 x[24,5] - 178.1442497195076 x[24,6]
   - 26.07181987353031 x[24,7] - 109.7801655198375 x[24,8]
   - 65.11211827015299 x[24,9] - 19.3212324364652 x[24,10]
   - 178.1442497195076 x[24,11] - 26.07181987353031 x[24,12]
   - 109.7801655198375 x[24,13] - 65.11211827015299 x[24,14]
   - 19.3212324364652 x[24,15] - 178.1442497195076 x[24,16]
   - 26.07181987353031 x[24,17] - 109.7801655198375 x[24,18]
   - 65.11211827015299 x[24,19] - 19.3212324364652 x[24,20]
   - 178.1442497195076 x[24,21] - 26.07181987353031 x[24,22]
   - 0.02 x[24,23] - 0.02 x[24,24] - 0.02 x[24,25] - start_service[24]
   + start_service[25] >= 0
 LowerBoundArrivalVehicle[1]: - 0.02 x[0,1] - 0.02 x[1,1] - 0.02 x[2,1]
   - 109.7801655198375 x[3,1] - 65.11211827015299 x[4,1]
   - 19.3212324364652 x[5,1] - 178.1442497195076 x[6,1]
   - 26.07181987353031 x[7,1] - 109.7801655198375 x[8,1]
   - 65.11211827015299 x[9,1] - 19.3212324364652 x[10,1]
   - 178.1442497195076 x[11,1] - 26.07181987353031 x[12,1]
   - 109.7801655198375 x[13,1] - 65.11211827015299 x[14,1]
   - 19.3212324364652 x[15,1] - 178.1442497195076 x[16,1]
   - 26.07181987353031 x[17,1] - 109.7801655198375 x[18,1]
   - 65.11211827015299 x[19,1] - 19.3212324364652 x[20,1]
   - 178.1442497195076 x[21,1] - 26.07181987353031 x[22,1] - 0.02 x[23,1]
   - 0.02 x[24,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 0.02 x[0,2] - 0.02 x[1,2] - 0.02 x[2,2]
   - 109.7801655198375 x[3,2] - 65.11211827015299 x[4,2]
   - 19.3212324364652 x[5,2] - 178.1442497195076 x[6,2]
   - 26.07181987353031 x[7,2] - 109.7801655198375 x[8,2]
   - 65.11211827015299 x[9,2] - 19.3212324364652 x[10,2]
   - 178.1442497195076 x[11,2] - 26.07181987353031 x[12,2]
   - 109.7801655198375 x[13,2] - 65.11211827015299 x[14,2]
   - 19.3212324364652 x[15,2] - 178.1442497195076 x[16,2]
   - 26.07181987353031 x[17,2] - 109.7801655198375 x[18,2]
   - 65.11211827015299 x[19,2] - 19.3212324364652 x[20,2]
   - 178.1442497195076 x[21,2] - 26.07181987353031 x[22,2] - 0.02 x[23,2]
   - 0.02 x[24,2] - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 54.90008275991876 x[0,3]
   - 54.90008275991876 x[1,3] - 54.90008275991876 x[2,3]
   - 54.90008275991876 x[3,3] - 73.48827426639944 x[4,3]
   - 66.88129388029633 x[5,3] - 160.7716784600681 x[6,3]
   - 54.90147359832122 x[7,3] - 54.90008275991876 x[8,3]
   - 73.48827426639944 x[9,3] - 66.88129388029633 x[10,3]
   - 160.7716784600681 x[11,3] - 54.90147359832122 x[12,3]
   - 54.90008275991876 x[13,3] - 73.48827426639944 x[14,3]
   - 66.88129388029633 x[15,3] - 160.7716784600681 x[16,3]
   - 54.90147359832122 x[17,3] - 54.90008275991876 x[18,3]
   - 73.48827426639944 x[19,3] - 66.88129388029633 x[20,3]
   - 160.7716784600681 x[21,3] - 54.90147359832122 x[22,3]
   - 54.90008275991876 x[23,3] - 54.90008275991876 x[24,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 32.56605913507649 x[0,4]
   - 32.56605913507649 x[1,4] - 32.56605913507649 x[2,4]
   - 95.8222978912417 x[3,4] - 32.56605913507649 x[4,4]
   - 50.65668874547983 x[5,4] - 145.8416206310857 x[6,4]
   - 38.29217441162187 x[7,4] - 95.8222978912417 x[8,4]
   - 32.56605913507649 x[9,4] - 50.65668874547983 x[10,4]
   - 145.8416206310857 x[11,4] - 38.29217441162187 x[12,4]
   - 95.8222978912417 x[13,4] - 32.56605913507649 x[14,4]
   - 50.65668874547983 x[15,4] - 145.8416206310857 x[16,4]
   - 38.29217441162187 x[17,4] - 95.8222978912417 x[18,4]
   - 32.56605913507649 x[19,4] - 50.65668874547983 x[20,4]
   - 145.8416206310857 x[21,4] - 38.29217441162187 x[22,4]
   - 32.56605913507649 x[23,4] - 32.56605913507649 x[24,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 9.670616218232601 x[0,5]
   - 9.670616218232601 x[1,5] - 9.670616218232601 x[2,5]
   - 112.1107604219825 x[3,5] - 73.55213166232372 x[4,5]
   - 9.670616218232601 x[5,5] - 186.8284847366019 x[6,5]
   - 30.70513902242705 x[7,5] - 112.1107604219825 x[8,5]
   - 73.55213166232372 x[9,5] - 9.670616218232601 x[10,5]
   - 186.8284847366019 x[11,5] - 30.70513902242705 x[12,5]
   - 112.1107604219825 x[13,5] - 73.55213166232372 x[14,5]
   - 9.670616218232601 x[15,5] - 186.8284847366019 x[16,5]
   - 30.70513902242705 x[17,5] - 112.1107604219825 x[18,5]
   - 73.55213166232372 x[19,5] - 9.670616218232601 x[20,5]
   - 186.8284847366019 x[21,5] - 30.70513902242705 x[22,5]
   - 9.670616218232601 x[23,5] - 9.670616218232601 x[24,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 89.08212485975379 x[0,6]
   - 89.08212485975379 x[1,6] - 89.08212485975379 x[2,6]
   - 126.589636360233 x[3,6] - 89.32555490640839 x[4,6]
   - 107.4169760950807 x[5,6] - 89.08212485975379 x[6,6]
   - 94.63022627118566 x[7,6] - 126.589636360233 x[8,6]
   - 89.32555490640839 x[9,6] - 107.4169760950807 x[10,6]
   - 89.08212485975379 x[11,6] - 94.63022627118566 x[12,6]
   - 126.589636360233 x[13,6] - 89.32555490640839 x[14,6]
   - 107.4169760950807 x[15,6] - 89.08212485975379 x[16,6]
   - 94.63022627118566 x[17,6] - 126.589636360233 x[18,6]
   - 89.32555490640839 x[19,6] - 107.4169760950807 x[20,6]
   - 89.08212485975379 x[21,6] - 94.63022627118566 x[22,6]
   - 89.08212485975379 x[23,6] - 89.08212485975379 x[24,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 13.04590993676516 x[0,7]
   - 13.04590993676516 x[1,7] - 13.04590993676516 x[2,7]
   - 96.75564642147484 x[3,7] - 57.81232360993322 x[4,7]
   - 27.32984530389449 x[5,7] - 170.6664411941743 x[6,7]
   - 13.04590993676516 x[7,7] - 96.75564642147484 x[8,7]
   - 57.81232360993322 x[9,7] - 27.32984530389449 x[10,7]
   - 170.6664411941743 x[11,7] - 13.04590993676516 x[12,7]
   - 96.75564642147484 x[13,7] - 57.81232360993322 x[14,7]
   - 27.32984530389449 x[15,7] - 170.6664411941743 x[16,7]
   - 13.04590993676516 x[17,7] - 96.75564642147484 x[18,7]
   - 57.81232360993322 x[19,7] - 27.32984530389449 x[20,7]
   - 170.6664411941743 x[21,7] - 13.04590993676516 x[22,7]
   - 13.04590993676516 x[23,7] - 13.04590993676516 x[24,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 54.90008275991876 x[0,8]
   - 54.90008275991876 x[1,8] - 54.90008275991876 x[2,8]
   - 54.90008275991876 x[3,8] - 73.48827426639944 x[4,8]
   - 66.88129388029633 x[5,8] - 160.7716784600681 x[6,8]
   - 54.90147359832122 x[7,8] - 54.90008275991876 x[8,8]
   - 73.48827426639944 x[9,8] - 66.88129388029633 x[10,8]
   - 160.7716784600681 x[11,8] - 54.90147359832122 x[12,8]
   - 54.90008275991876 x[13,8] - 73.48827426639944 x[14,8]
   - 66.88129388029633 x[15,8] - 160.7716784600681 x[16,8]
   - 54.90147359832122 x[17,8] - 54.90008275991876 x[18,8]
   - 73.48827426639944 x[19,8] - 66.88129388029633 x[20,8]
   - 160.7716784600681 x[21,8] - 54.90147359832122 x[22,8]
   - 54.90008275991876 x[23,8] - 54.90008275991876 x[24,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 32.56605913507649 x[0,9]
   - 32.56605913507649 x[1,9] - 32.56605913507649 x[2,9]
   - 95.8222978912417 x[3,9] - 32.56605913507649 x[4,9]
   - 50.65668874547983 x[5,9] - 145.8416206310857 x[6,9]
   - 38.29217441162187 x[7,9] - 95.8222978912417 x[8,9]
   - 32.56605913507649 x[9,9] - 50.65668874547983 x[10,9]
   - 145.8416206310857 x[11,9] - 38.29217441162187 x[12,9]
   - 95.8222978912417 x[13,9] - 32.56605913507649 x[14,9]
   - 50.65668874547983 x[15,9] - 145.8416206310857 x[16,9]
   - 38.29217441162187 x[17,9] - 95.8222978912417 x[18,9]
   - 32.56605913507649 x[19,9] - 50.65668874547983 x[20,9]
   - 145.8416206310857 x[21,9] - 38.29217441162187 x[22,9]
   - 32.56605913507649 x[23,9] - 32.56605913507649 x[24,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 9.670616218232601 x[0,10]
   - 9.670616218232601 x[1,10] - 9.670616218232601 x[2,10]
   - 112.1107604219825 x[3,10] - 73.55213166232372 x[4,10]
   - 9.670616218232601 x[5,10] - 186.8284847366019 x[6,10]
   - 30.70513902242705 x[7,10] - 112.1107604219825 x[8,10]
   - 73.55213166232372 x[9,10] - 9.670616218232601 x[10,10]
   - 186.8284847366019 x[11,10] - 30.70513902242705 x[12,10]
   - 112.1107604219825 x[13,10] - 73.55213166232372 x[14,10]
   - 9.670616218232601 x[15,10] - 186.8284847366019 x[16,10]
   - 30.70513902242705 x[17,10] - 112.1107604219825 x[18,10]
   - 73.55213166232372 x[19,10] - 9.670616218232601 x[20,10]
   - 186.8284847366019 x[21,10] - 30.70513902242705 x[22,10]
   - 9.670616218232601 x[23,10] - 9.670616218232601 x[24,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 89.08212485975379 x[0,11]
   - 89.08212485975379 x[1,11] - 89.08212485975379 x[2,11]
   - 126.589636360233 x[3,11] - 89.32555490640839 x[4,11]
   - 107.4169760950807 x[5,11] - 89.08212485975379 x[6,11]
   - 94.63022627118566 x[7,11] - 126.589636360233 x[8,11]
   - 89.32555490640839 x[9,11] - 107.4169760950807 x[10,11]
   - 89.08212485975379 x[11,11] - 94.63022627118566 x[12,11]
   - 126.589636360233 x[13,11] - 89.32555490640839 x[14,11]
   - 107.4169760950807 x[15,11] - 89.08212485975379 x[16,11]
   - 94.63022627118566 x[17,11] - 126.589636360233 x[18,11]
   - 89.32555490640839 x[19,11] - 107.4169760950807 x[20,11]
   - 89.08212485975379 x[21,11] - 94.63022627118566 x[22,11]
   - 89.08212485975379 x[23,11] - 89.08212485975379 x[24,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 13.04590993676516 x[0,12]
   - 13.04590993676516 x[1,12] - 13.04590993676516 x[2,12]
   - 96.75564642147484 x[3,12] - 57.81232360993322 x[4,12]
   - 27.32984530389449 x[5,12] - 170.6664411941743 x[6,12]
   - 13.04590993676516 x[7,12] - 96.75564642147484 x[8,12]
   - 57.81232360993322 x[9,12] - 27.32984530389449 x[10,12]
   - 170.6664411941743 x[11,12] - 13.04590993676516 x[12,12]
   - 96.75564642147484 x[13,12] - 57.81232360993322 x[14,12]
   - 27.32984530389449 x[15,12] - 170.6664411941743 x[16,12]
   - 13.04590993676516 x[17,12] - 96.75564642147484 x[18,12]
   - 57.81232360993322 x[19,12] - 27.32984530389449 x[20,12]
   - 170.6664411941743 x[21,12] - 13.04590993676516 x[22,12]
   - 13.04590993676516 x[23,12] - 13.04590993676516 x[24,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 54.90008275991876 x[0,13]
   - 54.90008275991876 x[1,13] - 54.90008275991876 x[2,13]
   - 54.90008275991876 x[3,13] - 73.48827426639944 x[4,13]
   - 66.88129388029633 x[5,13] - 160.7716784600681 x[6,13]
   - 54.90147359832122 x[7,13] - 54.90008275991876 x[8,13]
   - 73.48827426639944 x[9,13] - 66.88129388029633 x[10,13]
   - 160.7716784600681 x[11,13] - 54.90147359832122 x[12,13]
   - 54.90008275991876 x[13,13] - 73.48827426639944 x[14,13]
   - 66.88129388029633 x[15,13] - 160.7716784600681 x[16,13]
   - 54.90147359832122 x[17,13] - 54.90008275991876 x[18,13]
   - 73.48827426639944 x[19,13] - 66.88129388029633 x[20,13]
   - 160.7716784600681 x[21,13] - 54.90147359832122 x[22,13]
   - 54.90008275991876 x[23,13] - 54.90008275991876 x[24,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 32.56605913507649 x[0,14]
   - 32.56605913507649 x[1,14] - 32.56605913507649 x[2,14]
   - 95.8222978912417 x[3,14] - 32.56605913507649 x[4,14]
   - 50.65668874547983 x[5,14] - 145.8416206310857 x[6,14]
   - 38.29217441162187 x[7,14] - 95.8222978912417 x[8,14]
   - 32.56605913507649 x[9,14] - 50.65668874547983 x[10,14]
   - 145.8416206310857 x[11,14] - 38.29217441162187 x[12,14]
   - 95.8222978912417 x[13,14] - 32.56605913507649 x[14,14]
   - 50.65668874547983 x[15,14] - 145.8416206310857 x[16,14]
   - 38.29217441162187 x[17,14] - 95.8222978912417 x[18,14]
   - 32.56605913507649 x[19,14] - 50.65668874547983 x[20,14]
   - 145.8416206310857 x[21,14] - 38.29217441162187 x[22,14]
   - 32.56605913507649 x[23,14] - 32.56605913507649 x[24,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 9.670616218232601 x[0,15]
   - 9.670616218232601 x[1,15] - 9.670616218232601 x[2,15]
   - 112.1107604219825 x[3,15] - 73.55213166232372 x[4,15]
   - 9.670616218232601 x[5,15] - 186.8284847366019 x[6,15]
   - 30.70513902242705 x[7,15] - 112.1107604219825 x[8,15]
   - 73.55213166232372 x[9,15] - 9.670616218232601 x[10,15]
   - 186.8284847366019 x[11,15] - 30.70513902242705 x[12,15]
   - 112.1107604219825 x[13,15] - 73.55213166232372 x[14,15]
   - 9.670616218232601 x[15,15] - 186.8284847366019 x[16,15]
   - 30.70513902242705 x[17,15] - 112.1107604219825 x[18,15]
   - 73.55213166232372 x[19,15] - 9.670616218232601 x[20,15]
   - 186.8284847366019 x[21,15] - 30.70513902242705 x[22,15]
   - 9.670616218232601 x[23,15] - 9.670616218232601 x[24,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 89.08212485975379 x[0,16]
   - 89.08212485975379 x[1,16] - 89.08212485975379 x[2,16]
   - 126.589636360233 x[3,16] - 89.32555490640839 x[4,16]
   - 107.4169760950807 x[5,16] - 89.08212485975379 x[6,16]
   - 94.63022627118566 x[7,16] - 126.589636360233 x[8,16]
   - 89.32555490640839 x[9,16] - 107.4169760950807 x[10,16]
   - 89.08212485975379 x[11,16] - 94.63022627118566 x[12,16]
   - 126.589636360233 x[13,16] - 89.32555490640839 x[14,16]
   - 107.4169760950807 x[15,16] - 89.08212485975379 x[16,16]
   - 94.63022627118566 x[17,16] - 126.589636360233 x[18,16]
   - 89.32555490640839 x[19,16] - 107.4169760950807 x[20,16]
   - 89.08212485975379 x[21,16] - 94.63022627118566 x[22,16]
   - 89.08212485975379 x[23,16] - 89.08212485975379 x[24,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 13.04590993676516 x[0,17]
   - 13.04590993676516 x[1,17] - 13.04590993676516 x[2,17]
   - 96.75564642147484 x[3,17] - 57.81232360993322 x[4,17]
   - 27.32984530389449 x[5,17] - 170.6664411941743 x[6,17]
   - 13.04590993676516 x[7,17] - 96.75564642147484 x[8,17]
   - 57.81232360993322 x[9,17] - 27.32984530389449 x[10,17]
   - 170.6664411941743 x[11,17] - 13.04590993676516 x[12,17]
   - 96.75564642147484 x[13,17] - 57.81232360993322 x[14,17]
   - 27.32984530389449 x[15,17] - 170.6664411941743 x[16,17]
   - 13.04590993676516 x[17,17] - 96.75564642147484 x[18,17]
   - 57.81232360993322 x[19,17] - 27.32984530389449 x[20,17]
   - 170.6664411941743 x[21,17] - 13.04590993676516 x[22,17]
   - 13.04590993676516 x[23,17] - 13.04590993676516 x[24,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 54.90008275991876 x[0,18]
   - 54.90008275991876 x[1,18] - 54.90008275991876 x[2,18]
   - 54.90008275991876 x[3,18] - 73.48827426639944 x[4,18]
   - 66.88129388029633 x[5,18] - 160.7716784600681 x[6,18]
   - 54.90147359832122 x[7,18] - 54.90008275991876 x[8,18]
   - 73.48827426639944 x[9,18] - 66.88129388029633 x[10,18]
   - 160.7716784600681 x[11,18] - 54.90147359832122 x[12,18]
   - 54.90008275991876 x[13,18] - 73.48827426639944 x[14,18]
   - 66.88129388029633 x[15,18] - 160.7716784600681 x[16,18]
   - 54.90147359832122 x[17,18] - 54.90008275991876 x[18,18]
   - 73.48827426639944 x[19,18] - 66.88129388029633 x[20,18]
   - 160.7716784600681 x[21,18] - 54.90147359832122 x[22,18]
   - 54.90008275991876 x[23,18] - 54.90008275991876 x[24,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 32.56605913507649 x[0,19]
   - 32.56605913507649 x[1,19] - 32.56605913507649 x[2,19]
   - 95.8222978912417 x[3,19] - 32.56605913507649 x[4,19]
   - 50.65668874547983 x[5,19] - 145.8416206310857 x[6,19]
   - 38.29217441162187 x[7,19] - 95.8222978912417 x[8,19]
   - 32.56605913507649 x[9,19] - 50.65668874547983 x[10,19]
   - 145.8416206310857 x[11,19] - 38.29217441162187 x[12,19]
   - 95.8222978912417 x[13,19] - 32.56605913507649 x[14,19]
   - 50.65668874547983 x[15,19] - 145.8416206310857 x[16,19]
   - 38.29217441162187 x[17,19] - 95.8222978912417 x[18,19]
   - 32.56605913507649 x[19,19] - 50.65668874547983 x[20,19]
   - 145.8416206310857 x[21,19] - 38.29217441162187 x[22,19]
   - 32.56605913507649 x[23,19] - 32.56605913507649 x[24,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 9.670616218232601 x[0,20]
   - 9.670616218232601 x[1,20] - 9.670616218232601 x[2,20]
   - 112.1107604219825 x[3,20] - 73.55213166232372 x[4,20]
   - 9.670616218232601 x[5,20] - 186.8284847366019 x[6,20]
   - 30.70513902242705 x[7,20] - 112.1107604219825 x[8,20]
   - 73.55213166232372 x[9,20] - 9.670616218232601 x[10,20]
   - 186.8284847366019 x[11,20] - 30.70513902242705 x[12,20]
   - 112.1107604219825 x[13,20] - 73.55213166232372 x[14,20]
   - 9.670616218232601 x[15,20] - 186.8284847366019 x[16,20]
   - 30.70513902242705 x[17,20] - 112.1107604219825 x[18,20]
   - 73.55213166232372 x[19,20] - 9.670616218232601 x[20,20]
   - 186.8284847366019 x[21,20] - 30.70513902242705 x[22,20]
   - 9.670616218232601 x[23,20] - 9.670616218232601 x[24,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 89.08212485975379 x[0,21]
   - 89.08212485975379 x[1,21] - 89.08212485975379 x[2,21]
   - 126.589636360233 x[3,21] - 89.32555490640839 x[4,21]
   - 107.4169760950807 x[5,21] - 89.08212485975379 x[6,21]
   - 94.63022627118566 x[7,21] - 126.589636360233 x[8,21]
   - 89.32555490640839 x[9,21] - 107.4169760950807 x[10,21]
   - 89.08212485975379 x[11,21] - 94.63022627118566 x[12,21]
   - 126.589636360233 x[13,21] - 89.32555490640839 x[14,21]
   - 107.4169760950807 x[15,21] - 89.08212485975379 x[16,21]
   - 94.63022627118566 x[17,21] - 126.589636360233 x[18,21]
   - 89.32555490640839 x[19,21] - 107.4169760950807 x[20,21]
   - 89.08212485975379 x[21,21] - 94.63022627118566 x[22,21]
   - 89.08212485975379 x[23,21] - 89.08212485975379 x[24,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 13.04590993676516 x[0,22]
   - 13.04590993676516 x[1,22] - 13.04590993676516 x[2,22]
   - 96.75564642147484 x[3,22] - 57.81232360993322 x[4,22]
   - 27.32984530389449 x[5,22] - 170.6664411941743 x[6,22]
   - 13.04590993676516 x[7,22] - 96.75564642147484 x[8,22]
   - 57.81232360993322 x[9,22] - 27.32984530389449 x[10,22]
   - 170.6664411941743 x[11,22] - 13.04590993676516 x[12,22]
   - 96.75564642147484 x[13,22] - 57.81232360993322 x[14,22]
   - 27.32984530389449 x[15,22] - 170.6664411941743 x[16,22]
   - 13.04590993676516 x[17,22] - 96.75564642147484 x[18,22]
   - 57.81232360993322 x[19,22] - 27.32984530389449 x[20,22]
   - 170.6664411941743 x[21,22] - 13.04590993676516 x[22,22]
   - 13.04590993676516 x[23,22] - 13.04590993676516 x[24,22]
   - start_service[0] + start_service[22] >= 0
 LowerBoundArrivalVehicle[23]: - 0.02 x[0,23] - 0.02 x[1,23] - 0.02 x[2,23]
   - 109.7801655198375 x[3,23] - 65.11211827015299 x[4,23]
   - 19.3212324364652 x[5,23] - 178.1442497195076 x[6,23]
   - 26.07181987353031 x[7,23] - 109.7801655198375 x[8,23]
   - 65.11211827015299 x[9,23] - 19.3212324364652 x[10,23]
   - 178.1442497195076 x[11,23] - 26.07181987353031 x[12,23]
   - 109.7801655198375 x[13,23] - 65.11211827015299 x[14,23]
   - 19.3212324364652 x[15,23] - 178.1442497195076 x[16,23]
   - 26.07181987353031 x[17,23] - 109.7801655198375 x[18,23]
   - 65.11211827015299 x[19,23] - 19.3212324364652 x[20,23]
   - 178.1442497195076 x[21,23] - 26.07181987353031 x[22,23]
   - 0.02 x[23,23] - 0.02 x[24,23] - start_service[0] + start_service[23]
   >= 0
 LowerBoundArrivalVehicle[24]: - 0.02 x[0,24] - 0.02 x[1,24] - 0.02 x[2,24]
   - 109.7801655198375 x[3,24] - 65.11211827015299 x[4,24]
   - 19.3212324364652 x[5,24] - 178.1442497195076 x[6,24]
   - 26.07181987353031 x[7,24] - 109.7801655198375 x[8,24]
   - 65.11211827015299 x[9,24] - 19.3212324364652 x[10,24]
   - 178.1442497195076 x[11,24] - 26.07181987353031 x[12,24]
   - 109.7801655198375 x[13,24] - 65.11211827015299 x[14,24]
   - 19.3212324364652 x[15,24] - 178.1442497195076 x[16,24]
   - 26.07181987353031 x[17,24] - 109.7801655198375 x[18,24]
   - 65.11211827015299 x[19,24] - 19.3212324364652 x[20,24]
   - 178.1442497195076 x[21,24] - 26.07181987353031 x[22,24]
   - 0.02 x[23,24] - 0.02 x[24,24] - start_service[0] + start_service[24]
   >= 0
 LowerBoundArrivalVehicle[25]: - 0.02 x[0,25] - 0.02 x[1,25] - 0.02 x[2,25]
   - 109.7801655198375 x[3,25] - 65.11211827015299 x[4,25]
   - 19.3212324364652 x[5,25] - 178.1442497195076 x[6,25]
   - 26.07181987353031 x[7,25] - 109.7801655198375 x[8,25]
   - 65.11211827015299 x[9,25] - 19.3212324364652 x[10,25]
   - 178.1442497195076 x[11,25] - 26.07181987353031 x[12,25]
   - 109.7801655198375 x[13,25] - 65.11211827015299 x[14,25]
   - 19.3212324364652 x[15,25] - 178.1442497195076 x[16,25]
   - 26.07181987353031 x[17,25] - 109.7801655198375 x[18,25]
   - 65.11211827015299 x[19,25] - 19.3212324364652 x[20,25]
   - 178.1442497195076 x[21,25] - 26.07181987353031 x[22,25]
   - 0.02 x[23,25] - 0.02 x[24,25] - start_service[0] + start_service[25]
   >= 0
 LeaveArrivalNodesNotFullyLoaded[3]: w[3,0] + w[3,1] + w[3,2] + w[3,3]
   + w[3,4] + w[3,5] + w[3,6] + w[3,7] + w[3,8] + w[3,9] + w[3,10]
   + w[3,11] + w[3,12] + w[3,14] + w[3,15] + w[3,16] + w[3,17] + w[3,19]
   + w[3,20] + w[3,21] + w[3,22] + w[3,23] + w[3,24] + w[3,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[4]: w[4,0] + w[4,1] + w[4,2] + w[4,3]
   + w[4,4] + w[4,5] + w[4,6] + w[4,7] + w[4,8] + w[4,9] + w[4,10]
   + w[4,11] + w[4,12] + w[4,13] + w[4,15] + w[4,16] + w[4,17] + w[4,18]
   + w[4,20] + w[4,21] + w[4,22] + w[4,23] + w[4,24] + w[4,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[5]: w[5,0] + w[5,1] + w[5,2] + w[5,3]
   + w[5,4] + w[5,5] + w[5,6] + w[5,7] + w[5,8] + w[5,9] + w[5,10]
   + w[5,11] + w[5,12] + w[5,13] + w[5,14] + w[5,16] + w[5,17] + w[5,18]
   + w[5,19] + w[5,21] + w[5,22] + w[5,23] + w[5,24] + w[5,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[6]: w[6,0] + w[6,1] + w[6,2] + w[6,3]
   + w[6,4] + w[6,5] + w[6,6] + w[6,7] + w[6,8] + w[6,9] + w[6,10]
   + w[6,11] + w[6,12] + w[6,13] + w[6,14] + w[6,15] + w[6,17] + w[6,18]
   + w[6,19] + w[6,20] + w[6,22] + w[6,23] + w[6,24] + w[6,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[7]: w[7,0] + w[7,1] + w[7,2] + w[7,3]
   + w[7,4] + w[7,5] + w[7,6] + w[7,7] + w[7,8] + w[7,9] + w[7,10]
   + w[7,11] + w[7,12] + w[7,13] + w[7,14] + w[7,15] + w[7,16] + w[7,18]
   + w[7,19] + w[7,20] + w[7,21] + w[7,23] + w[7,24] + w[7,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[8]: w[8,0] + w[8,1] + w[8,2] + w[8,3]
   + w[8,4] + w[8,5] + w[8,6] + w[8,7] + w[8,8] + w[8,9] + w[8,10]
   + w[8,11] + w[8,12] + w[8,14] + w[8,15] + w[8,16] + w[8,17] + w[8,19]
   + w[8,20] + w[8,21] + w[8,22] + w[8,23] + w[8,24] + w[8,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[9]: w[9,0] + w[9,1] + w[9,2] + w[9,3]
   + w[9,4] + w[9,5] + w[9,6] + w[9,7] + w[9,8] + w[9,9] + w[9,10]
   + w[9,11] + w[9,12] + w[9,13] + w[9,15] + w[9,16] + w[9,17] + w[9,18]
   + w[9,20] + w[9,21] + w[9,22] + w[9,23] + w[9,24] + w[9,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[10]: w[10,0] + w[10,1] + w[10,2] + w[10,3]
   + w[10,4] + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,10]
   + w[10,11] + w[10,12] + w[10,13] + w[10,14] + w[10,16] + w[10,17]
   + w[10,18] + w[10,19] + w[10,21] + w[10,22] + w[10,23] + w[10,24]
   + w[10,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[11]: w[11,0] + w[11,1] + w[11,2] + w[11,3]
   + w[11,4] + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10]
   + w[11,11] + w[11,12] + w[11,13] + w[11,14] + w[11,15] + w[11,17]
   + w[11,18] + w[11,19] + w[11,20] + w[11,22] + w[11,23] + w[11,24]
   + w[11,25] <= 1
 LeaveArrivalNodesNotFullyLoaded[12]: w[12,0] + w[12,1] + w[12,2] + w[12,3]
   + w[12,4] + w[12,5] + w[12,6] + w[12,7] + w[12,8] + w[12,9] + w[12,10]
   + w[12,11] + w[12,12] + w[12,13] + w[12,14] + w[12,15] + w[12,16]
   + w[12,18] + w[12,19] + w[12,20] + w[12,21] + w[12,23] + w[12,24]
   + w[12,25] <= 1
 VisitDepartureNodesNotFullyLoaded[13]: w[0,13] + w[1,13] + w[2,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[9,13] + w[10,13] + w[11,13]
   + w[12,13] + w[13,13] + w[14,13] + w[15,13] + w[16,13] + w[17,13]
   + w[18,13] + w[19,13] + w[20,13] + w[21,13] + w[22,13] + w[23,13]
   + w[24,13] + w[25,13] <= 1
 VisitDepartureNodesNotFullyLoaded[14]: w[0,14] + w[1,14] + w[2,14]
   + w[3,14] + w[5,14] + w[6,14] + w[7,14] + w[8,14] + w[10,14] + w[11,14]
   + w[12,14] + w[13,14] + w[14,14] + w[15,14] + w[16,14] + w[17,14]
   + w[18,14] + w[19,14] + w[20,14] + w[21,14] + w[22,14] + w[23,14]
   + w[24,14] + w[25,14] <= 1
 VisitDepartureNodesNotFullyLoaded[15]: w[0,15] + w[1,15] + w[2,15]
   + w[3,15] + w[4,15] + w[6,15] + w[7,15] + w[8,15] + w[9,15] + w[11,15]
   + w[12,15] + w[13,15] + w[14,15] + w[15,15] + w[16,15] + w[17,15]
   + w[18,15] + w[19,15] + w[20,15] + w[21,15] + w[22,15] + w[23,15]
   + w[24,15] + w[25,15] <= 1
 VisitDepartureNodesNotFullyLoaded[16]: w[0,16] + w[1,16] + w[2,16]
   + w[3,16] + w[4,16] + w[5,16] + w[7,16] + w[8,16] + w[9,16] + w[10,16]
   + w[12,16] + w[13,16] + w[14,16] + w[15,16] + w[16,16] + w[17,16]
   + w[18,16] + w[19,16] + w[20,16] + w[21,16] + w[22,16] + w[23,16]
   + w[24,16] + w[25,16] <= 1
 VisitDepartureNodesNotFullyLoaded[17]: w[0,17] + w[1,17] + w[2,17]
   + w[3,17] + w[4,17] + w[5,17] + w[6,17] + w[8,17] + w[9,17] + w[10,17]
   + w[11,17] + w[13,17] + w[14,17] + w[15,17] + w[16,17] + w[17,17]
   + w[18,17] + w[19,17] + w[20,17] + w[21,17] + w[22,17] + w[23,17]
   + w[24,17] + w[25,17] <= 1
 VisitDepartureNodesNotFullyLoaded[18]: w[0,18] + w[1,18] + w[2,18]
   + w[4,18] + w[5,18] + w[6,18] + w[7,18] + w[9,18] + w[10,18] + w[11,18]
   + w[12,18] + w[13,18] + w[14,18] + w[15,18] + w[16,18] + w[17,18]
   + w[18,18] + w[19,18] + w[20,18] + w[21,18] + w[22,18] + w[23,18]
   + w[24,18] + w[25,18] <= 1
 VisitDepartureNodesNotFullyLoaded[19]: w[0,19] + w[1,19] + w[2,19]
   + w[3,19] + w[5,19] + w[6,19] + w[7,19] + w[8,19] + w[10,19] + w[11,19]
   + w[12,19] + w[13,19] + w[14,19] + w[15,19] + w[16,19] + w[17,19]
   + w[18,19] + w[19,19] + w[20,19] + w[21,19] + w[22,19] + w[23,19]
   + w[24,19] + w[25,19] <= 1
 VisitDepartureNodesNotFullyLoaded[20]: w[0,20] + w[1,20] + w[2,20]
   + w[3,20] + w[4,20] + w[6,20] + w[7,20] + w[8,20] + w[9,20] + w[11,20]
   + w[12,20] + w[13,20] + w[14,20] + w[15,20] + w[16,20] + w[17,20]
   + w[18,20] + w[19,20] + w[20,20] + w[21,20] + w[22,20] + w[23,20]
   + w[24,20] + w[25,20] <= 1
 VisitDepartureNodesNotFullyLoaded[21]: w[0,21] + w[1,21] + w[2,21]
   + w[3,21] + w[4,21] + w[5,21] + w[7,21] + w[8,21] + w[9,21] + w[10,21]
   + w[12,21] + w[13,21] + w[14,21] + w[15,21] + w[16,21] + w[17,21]
   + w[18,21] + w[19,21] + w[20,21] + w[21,21] + w[22,21] + w[23,21]
   + w[24,21] + w[25,21] <= 1
 VisitDepartureNodesNotFullyLoaded[22]: w[0,22] + w[1,22] + w[2,22]
   + w[3,22] + w[4,22] + w[5,22] + w[6,22] + w[8,22] + w[9,22] + w[10,22]
   + w[11,22] + w[13,22] + w[14,22] + w[15,22] + w[16,22] + w[17,22]
   + w[18,22] + w[19,22] + w[20,22] + w[21,22] + w[22,22] + w[23,22]
   + w[24,22] + w[25,22] <= 1
 LeaveDepartureNodesNotEmpty[13]: w[13,1] + w[13,2] + w[13,3] + w[13,4]
   + w[13,5] + w[13,6] + w[13,7] + w[13,8] + w[13,9] + w[13,10] + w[13,11]
   + w[13,12] + w[13,13] + w[13,14] + w[13,15] + w[13,16] + w[13,17]
   + w[13,18] + w[13,19] + w[13,20] + w[13,21] + w[13,22] + w[13,23]
   + w[13,24] + w[13,25] >= 1
 LeaveDepartureNodesNotEmpty[14]: w[14,1] + w[14,2] + w[14,3] + w[14,4]
   + w[14,5] + w[14,6] + w[14,7] + w[14,8] + w[14,9] + w[14,10] + w[14,11]
   + w[14,12] + w[14,13] + w[14,14] + w[14,15] + w[14,16] + w[14,17]
   + w[14,18] + w[14,19] + w[14,20] + w[14,21] + w[14,22] + w[14,23]
   + w[14,24] + w[14,25] >= 1
 LeaveDepartureNodesNotEmpty[15]: w[15,1] + w[15,2] + w[15,3] + w[15,4]
   + w[15,5] + w[15,6] + w[15,7] + w[15,8] + w[15,9] + w[15,10] + w[15,11]
   + w[15,12] + w[15,13] + w[15,14] + w[15,15] + w[15,16] + w[15,17]
   + w[15,18] + w[15,19] + w[15,20] + w[15,21] + w[15,22] + w[15,23]
   + w[15,24] + w[15,25] >= 1
 LeaveDepartureNodesNotEmpty[16]: w[16,1] + w[16,2] + w[16,3] + w[16,4]
   + w[16,5] + w[16,6] + w[16,7] + w[16,8] + w[16,9] + w[16,10] + w[16,11]
   + w[16,12] + w[16,13] + w[16,14] + w[16,15] + w[16,16] + w[16,17]
   + w[16,18] + w[16,19] + w[16,20] + w[16,21] + w[16,22] + w[16,23]
   + w[16,24] + w[16,25] >= 1
 LeaveDepartureNodesNotEmpty[17]: w[17,1] + w[17,2] + w[17,3] + w[17,4]
   + w[17,5] + w[17,6] + w[17,7] + w[17,8] + w[17,9] + w[17,10] + w[17,11]
   + w[17,12] + w[17,13] + w[17,14] + w[17,15] + w[17,16] + w[17,17]
   + w[17,18] + w[17,19] + w[17,20] + w[17,21] + w[17,22] + w[17,23]
   + w[17,24] + w[17,25] >= 1
 LeaveDepartureNodesNotEmpty[18]: w[18,1] + w[18,2] + w[18,3] + w[18,4]
   + w[18,5] + w[18,6] + w[18,7] + w[18,8] + w[18,9] + w[18,10] + w[18,11]
   + w[18,12] + w[18,13] + w[18,14] + w[18,15] + w[18,16] + w[18,17]
   + w[18,18] + w[18,19] + w[18,20] + w[18,21] + w[18,22] + w[18,23]
   + w[18,24] + w[18,25] >= 1
 LeaveDepartureNodesNotEmpty[19]: w[19,1] + w[19,2] + w[19,3] + w[19,4]
   + w[19,5] + w[19,6] + w[19,7] + w[19,8] + w[19,9] + w[19,10] + w[19,11]
   + w[19,12] + w[19,13] + w[19,14] + w[19,15] + w[19,16] + w[19,17]
   + w[19,18] + w[19,19] + w[19,20] + w[19,21] + w[19,22] + w[19,23]
   + w[19,24] + w[19,25] >= 1
 LeaveDepartureNodesNotEmpty[20]: w[20,1] + w[20,2] + w[20,3] + w[20,4]
   + w[20,5] + w[20,6] + w[20,7] + w[20,8] + w[20,9] + w[20,10] + w[20,11]
   + w[20,12] + w[20,13] + w[20,14] + w[20,15] + w[20,16] + w[20,17]
   + w[20,18] + w[20,19] + w[20,20] + w[20,21] + w[20,22] + w[20,23]
   + w[20,24] + w[20,25] >= 1
 LeaveDepartureNodesNotEmpty[21]: w[21,1] + w[21,2] + w[21,3] + w[21,4]
   + w[21,5] + w[21,6] + w[21,7] + w[21,8] + w[21,9] + w[21,10] + w[21,11]
   + w[21,12] + w[21,13] + w[21,14] + w[21,15] + w[21,16] + w[21,17]
   + w[21,18] + w[21,19] + w[21,20] + w[21,21] + w[21,22] + w[21,23]
   + w[21,24] + w[21,25] >= 1
 LeaveDepartureNodesNotEmpty[22]: w[22,1] + w[22,2] + w[22,3] + w[22,4]
   + w[22,5] + w[22,6] + w[22,7] + w[22,8] + w[22,9] + w[22,10] + w[22,11]
   + w[22,12] + w[22,13] + w[22,14] + w[22,15] + w[22,16] + w[22,17]
   + w[22,18] + w[22,19] + w[22,20] + w[22,21] + w[22,22] + w[22,23]
   + w[22,24] + w[22,25] >= 1
 VehiclesStayInTheirNodeSetAfterDepotVisits[1,1]: x[0,1] - x[1,0] - x[1,2]
   - x[1,3] - x[1,4] - x[1,5] - x[1,6] - x[1,7] - x[1,13] - x[1,14]
   - x[1,15] - x[1,16] - x[1,17] - x[1,23] - x[1,24] - x[1,25] + x[2,1]
   + x[3,1] + x[4,1] + x[5,1] + x[6,1] + x[7,1] + x[13,1] + x[14,1]
   + x[15,1] + x[16,1] + x[17,1] + x[23,1] + x[24,1] + x[25,1] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[1,2]: x[0,1] - x[1,0] - x[1,2]
   - x[1,8] - x[1,9] - x[1,10] - x[1,11] - x[1,12] - x[1,18] - x[1,19]
   - x[1,20] - x[1,21] - x[1,22] - x[1,23] - x[1,24] - x[1,25] + x[2,1]
   + x[8,1] + x[9,1] + x[10,1] + x[11,1] + x[12,1] + x[18,1] + x[19,1]
   + x[20,1] + x[21,1] + x[22,1] + x[23,1] + x[24,1] + x[25,1] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[2,1]: x[0,2] + x[1,2] - x[2,0]
   - x[2,1] - x[2,3] - x[2,4] - x[2,5] - x[2,6] - x[2,7] - x[2,13]
   - x[2,14] - x[2,15] - x[2,16] - x[2,17] - x[2,23] - x[2,24] - x[2,25]
   + x[3,2] + x[4,2] + x[5,2] + x[6,2] + x[7,2] + x[13,2] + x[14,2]
   + x[15,2] + x[16,2] + x[17,2] + x[23,2] + x[24,2] + x[25,2] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[2,2]: x[0,2] + x[1,2] - x[2,0]
   - x[2,1] - x[2,8] - x[2,9] - x[2,10] - x[2,11] - x[2,12] - x[2,18]
   - x[2,19] - x[2,20] - x[2,21] - x[2,22] - x[2,23] - x[2,24] - x[2,25]
   + x[8,2] + x[9,2] + x[10,2] + x[11,2] + x[12,2] + x[18,2] + x[19,2]
   + x[20,2] + x[21,2] + x[22,2] + x[23,2] + x[24,2] + x[25,2] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[23,1]: x[0,23] + x[1,23]
   + x[2,23] + x[3,23] + x[4,23] + x[5,23] + x[6,23] + x[7,23] + x[13,23]
   + x[14,23] + x[15,23] + x[16,23] + x[17,23] - x[23,0] - x[23,1]
   - x[23,2] - x[23,3] - x[23,4] - x[23,5] - x[23,6] - x[23,7] - x[23,13]
   - x[23,14] - x[23,15] - x[23,16] - x[23,17] - x[23,24] - x[23,25]
   + x[24,23] + x[25,23] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[23,2]: x[0,23] + x[1,23]
   + x[2,23] + x[8,23] + x[9,23] + x[10,23] + x[11,23] + x[12,23]
   + x[18,23] + x[19,23] + x[20,23] + x[21,23] + x[22,23] - x[23,0]
   - x[23,1] - x[23,2] - x[23,8] - x[23,9] - x[23,10] - x[23,11] - x[23,12]
   - x[23,18] - x[23,19] - x[23,20] - x[23,21] - x[23,22] - x[23,24]
   - x[23,25] + x[24,23] + x[25,23] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[24,1]: x[0,24] + x[1,24]
   + x[2,24] + x[3,24] + x[4,24] + x[5,24] + x[6,24] + x[7,24] + x[13,24]
   + x[14,24] + x[15,24] + x[16,24] + x[17,24] + x[23,24] - x[24,0]
   - x[24,1] - x[24,2] - x[24,3] - x[24,4] - x[24,5] - x[24,6] - x[24,7]
   - x[24,13] - x[24,14] - x[24,15] - x[24,16] - x[24,17] - x[24,23]
   - x[24,25] + x[25,24] = 0
 VehiclesStayInTheirNodeSetAfterDepotVisits[24,2]: x[0,24] + x[1,24]
   + x[2,24] + x[8,24] + x[9,24] + x[10,24] + x[11,24] + x[12,24]
   + x[18,24] + x[19,24] + x[20,24] + x[21,24] + x[22,24] + x[23,24]
   - x[24,0] - x[24,1] - x[24,2] - x[24,8] - x[24,9] - x[24,10] - x[24,11]
   - x[24,12] - x[24,18] - x[24,19] - x[24,20] - x[24,21] - x[24,22]
   - x[24,23] - x[24,25] + x[25,24] = 0
 R3337: <= -1
Bounds
 x[0,0] = 0
 x[0,1] = 0
 x[0,2] = 0
 x[1,0] = 0
 x[1,1] = 0
 x[1,2] = 0
 x[2,0] = 0
 x[2,1] = 0
 x[2,2] = 0
 x[3,0] = 0
 x[3,3] = 0
 x[3,8] = 0
 x[3,9] = 0
 x[3,10] = 0
 x[3,11] = 0
 x[3,12] = 0
 x[3,18] = 0
 x[3,19] = 0
 x[3,20] = 0
 x[3,21] = 0
 x[3,22] = 0
 x[4,0] = 0
 x[4,4] = 0
 x[4,8] = 0
 x[4,9] = 0
 x[4,10] = 0
 x[4,11] = 0
 x[4,12] = 0
 x[4,18] = 0
 x[4,19] = 0
 x[4,20] = 0
 x[4,21] = 0
 x[4,22] = 0
 x[5,0] = 0
 x[5,5] = 0
 x[5,8] = 0
 x[5,9] = 0
 x[5,10] = 0
 x[5,11] = 0
 x[5,12] = 0
 x[5,18] = 0
 x[5,19] = 0
 x[5,20] = 0
 x[5,21] = 0
 x[5,22] = 0
 x[6,0] = 0
 x[6,6] = 0
 x[6,8] = 0
 x[6,9] = 0
 x[6,10] = 0
 x[6,11] = 0
 x[6,12] = 0
 x[6,18] = 0
 x[6,19] = 0
 x[6,20] = 0
 x[6,21] = 0
 x[6,22] = 0
 x[7,0] = 0
 x[7,7] = 0
 x[7,8] = 0
 x[7,9] = 0
 x[7,10] = 0
 x[7,11] = 0
 x[7,12] = 0
 x[7,18] = 0
 x[7,19] = 0
 x[7,20] = 0
 x[7,21] = 0
 x[7,22] = 0
 x[8,0] = 0
 x[8,3] = 0
 x[8,4] = 0
 x[8,5] = 0
 x[8,6] = 0
 x[8,7] = 0
 x[8,8] = 0
 x[8,13] = 0
 x[8,14] = 0
 x[8,15] = 0
 x[8,16] = 0
 x[8,17] = 0
 x[9,0] = 0
 x[9,3] = 0
 x[9,4] = 0
 x[9,5] = 0
 x[9,6] = 0
 x[9,7] = 0
 x[9,9] = 0
 x[9,13] = 0
 x[9,14] = 0
 x[9,15] = 0
 x[9,16] = 0
 x[9,17] = 0
 x[10,0] = 0
 x[10,3] = 0
 x[10,4] = 0
 x[10,5] = 0
 x[10,6] = 0
 x[10,7] = 0
 x[10,10] = 0
 x[10,13] = 0
 x[10,14] = 0
 x[10,15] = 0
 x[10,16] = 0
 x[10,17] = 0
 x[11,0] = 0
 x[11,3] = 0
 x[11,4] = 0
 x[11,5] = 0
 x[11,6] = 0
 x[11,7] = 0
 x[11,11] = 0
 x[11,13] = 0
 x[11,14] = 0
 x[11,15] = 0
 x[11,16] = 0
 x[11,17] = 0
 x[12,0] = 0
 x[12,3] = 0
 x[12,4] = 0
 x[12,5] = 0
 x[12,6] = 0
 x[12,7] = 0
 x[12,12] = 0
 x[12,13] = 0
 x[12,14] = 0
 x[12,15] = 0
 x[12,16] = 0
 x[12,17] = 0
 x[13,0] = 0
 x[13,3] = 0
 x[13,8] = 0
 x[13,13] = 0
 x[13,18] = 0
 x[13,19] = 0
 x[13,20] = 0
 x[13,21] = 0
 x[13,22] = 0
 x[14,0] = 0
 x[14,4] = 0
 x[14,9] = 0
 x[14,14] = 0
 x[14,18] = 0
 x[14,19] = 0
 x[14,20] = 0
 x[14,21] = 0
 x[14,22] = 0
 x[15,0] = 0
 x[15,5] = 0
 x[15,10] = 0
 x[15,15] = 0
 x[15,18] = 0
 x[15,19] = 0
 x[15,20] = 0
 x[15,21] = 0
 x[15,22] = 0
 x[16,0] = 0
 x[16,6] = 0
 x[16,11] = 0
 x[16,16] = 0
 x[16,18] = 0
 x[16,19] = 0
 x[16,20] = 0
 x[16,21] = 0
 x[16,22] = 0
 x[17,0] = 0
 x[17,7] = 0
 x[17,12] = 0
 x[17,17] = 0
 x[17,18] = 0
 x[17,19] = 0
 x[17,20] = 0
 x[17,21] = 0
 x[17,22] = 0
 x[18,0] = 0
 x[18,3] = 0
 x[18,8] = 0
 x[18,13] = 0
 x[18,14] = 0
 x[18,15] = 0
 x[18,16] = 0
 x[18,17] = 0
 x[18,18] = 0
 x[19,0] = 0
 x[19,4] = 0
 x[19,9] = 0
 x[19,13] = 0
 x[19,14] = 0
 x[19,15] = 0
 x[19,16] = 0
 x[19,17] = 0
 x[19,19] = 0
 x[20,0] = 0
 x[20,5] = 0
 x[20,10] = 0
 x[20,13] = 0
 x[20,14] = 0
 x[20,15] = 0
 x[20,16] = 0
 x[20,17] = 0
 x[20,20] = 0
 x[21,0] = 0
 x[21,6] = 0
 x[21,11] = 0
 x[21,13] = 0
 x[21,14] = 0
 x[21,15] = 0
 x[21,16] = 0
 x[21,17] = 0
 x[21,21] = 0
 x[22,0] = 0
 x[22,7] = 0
 x[22,12] = 0
 x[22,13] = 0
 x[22,14] = 0
 x[22,15] = 0
 x[22,16] = 0
 x[22,17] = 0
 x[22,22] = 0
 x[23,0] = 0
 x[23,23] = 0
 x[23,24] = 0
 x[23,25] = 0
 x[24,0] = 0
 x[24,23] = 0
 x[24,24] = 0
 x[24,25] = 0
 x[25,0] = 0
 x[25,1] = 0
 x[25,2] = 0
 x[25,3] = 0
 x[25,4] = 0
 x[25,5] = 0
 x[25,6] = 0
 x[25,7] = 0
 x[25,8] = 0
 x[25,9] = 0
 x[25,10] = 0
 x[25,11] = 0
 x[25,12] = 0
 x[25,13] = 0
 x[25,14] = 0
 x[25,15] = 0
 x[25,16] = 0
 x[25,17] = 0
 x[25,18] = 0
 x[25,19] = 0
 x[25,20] = 0
 x[25,21] = 0
 x[25,22] = 0
 x[25,23] = 0
 x[25,24] = 0
 x[25,25] = 0
 w[0,0] = -0
 w[0,1] <= 2
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
 w[0,24] <= 2
 w[0,25] <= 2
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
 w[1,24] <= 2
 w[1,25] <= 2
 w[2,0] = -0
 w[2,1] <= 2
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
 w[2,24] <= 2
 w[2,25] <= 2
 w[3,0] = -0
 w[3,1] = -0
 w[3,2] = -0
 w[3,3] = -0
 w[3,4] <= 2
 w[3,5] <= 2
 w[3,6] <= 2
 w[3,7] <= 2
 w[3,8] = -0
 w[3,9] = -0
 w[3,10] = -0
 w[3,11] = -0
 w[3,12] = -0
 w[3,13] <= 2
 w[3,14] <= 2
 w[3,15] <= 2
 w[3,16] <= 2
 w[3,17] <= 2
 w[3,18] <= 2
 w[3,19] = -0
 w[3,20] = -0
 w[3,21] = -0
 w[3,22] = -0
 w[3,23] <= 2
 w[3,24] <= 2
 w[3,25] <= 2
 w[4,0] = -0
 w[4,1] = -0
 w[4,2] = -0
 w[4,3] <= 2
 w[4,4] = -0
 w[4,5] <= 2
 w[4,6] <= 2
 w[4,7] <= 2
 w[4,8] = -0
 w[4,9] = -0
 w[4,10] = -0
 w[4,11] = -0
 w[4,12] = -0
 w[4,13] <= 2
 w[4,14] <= 2
 w[4,15] <= 2
 w[4,16] <= 2
 w[4,17] <= 2
 w[4,18] = -0
 w[4,19] <= 2
 w[4,20] = -0
 w[4,21] = -0
 w[4,22] = -0
 w[4,23] <= 2
 w[4,24] <= 2
 w[4,25] <= 2
 w[5,0] = -0
 w[5,1] = -0
 w[5,2] = -0
 w[5,3] <= 2
 w[5,4] <= 2
 w[5,5] = -0
 w[5,6] <= 2
 w[5,7] <= 2
 w[5,8] = -0
 w[5,9] = -0
 w[5,10] = -0
 w[5,11] = -0
 w[5,12] = -0
 w[5,13] <= 2
 w[5,14] <= 2
 w[5,15] <= 2
 w[5,16] <= 2
 w[5,17] <= 2
 w[5,18] = -0
 w[5,19] = -0
 w[5,20] <= 2
 w[5,21] = -0
 w[5,22] = -0
 w[5,23] <= 2
 w[5,24] <= 2
 w[5,25] <= 2
 w[6,0] = -0
 w[6,1] = -0
 w[6,2] = -0
 w[6,3] <= 2
 w[6,4] <= 2
 w[6,5] <= 2
 w[6,6] = -0
 w[6,7] <= 2
 w[6,8] = -0
 w[6,9] = -0
 w[6,10] = -0
 w[6,11] = -0
 w[6,12] = -0
 w[6,13] <= 2
 w[6,14] <= 2
 w[6,15] <= 2
 w[6,16] <= 2
 w[6,17] <= 2
 w[6,18] = -0
 w[6,19] = -0
 w[6,20] = -0
 w[6,21] <= 2
 w[6,22] = -0
 w[6,23] <= 2
 w[6,24] <= 2
 w[6,25] <= 2
 w[7,0] = -0
 w[7,1] = -0
 w[7,2] = -0
 w[7,3] <= 2
 w[7,4] <= 2
 w[7,5] <= 2
 w[7,6] <= 2
 w[7,7] = -0
 w[7,8] = -0
 w[7,9] = -0
 w[7,10] = -0
 w[7,11] = -0
 w[7,12] = -0
 w[7,13] <= 2
 w[7,14] <= 2
 w[7,15] <= 2
 w[7,16] <= 2
 w[7,17] <= 2
 w[7,18] = -0
 w[7,19] = -0
 w[7,20] = -0
 w[7,21] = -0
 w[7,22] <= 2
 w[7,23] <= 2
 w[7,24] <= 2
 w[7,25] <= 2
 w[8,0] = -0
 w[8,1] = -0
 w[8,2] = -0
 w[8,3] = -0
 w[8,4] = -0
 w[8,5] = -0
 w[8,6] = -0
 w[8,7] = -0
 w[8,8] = -0
 w[8,9] <= 2
 w[8,10] <= 2
 w[8,11] <= 2
 w[8,12] <= 2
 w[8,13] <= 2
 w[8,14] = -0
 w[8,15] = -0
 w[8,16] = -0
 w[8,17] = -0
 w[8,18] <= 2
 w[8,19] <= 2
 w[8,20] <= 2
 w[8,21] <= 2
 w[8,22] <= 2
 w[8,23] <= 2
 w[8,24] <= 2
 w[8,25] <= 2
 w[9,0] = -0
 w[9,1] = -0
 w[9,2] = -0
 w[9,3] = -0
 w[9,4] = -0
 w[9,5] = -0
 w[9,6] = -0
 w[9,7] = -0
 w[9,8] <= 2
 w[9,9] = -0
 w[9,10] <= 2
 w[9,11] <= 2
 w[9,12] <= 2
 w[9,13] = -0
 w[9,14] <= 2
 w[9,15] = -0
 w[9,16] = -0
 w[9,17] = -0
 w[9,18] <= 2
 w[9,19] <= 2
 w[9,20] <= 2
 w[9,21] <= 2
 w[9,22] <= 2
 w[9,23] <= 2
 w[9,24] <= 2
 w[9,25] <= 2
 w[10,0] = -0
 w[10,1] = -0
 w[10,2] = -0
 w[10,3] = -0
 w[10,4] = -0
 w[10,5] = -0
 w[10,6] = -0
 w[10,7] = -0
 w[10,8] <= 2
 w[10,9] <= 2
 w[10,10] = -0
 w[10,11] <= 2
 w[10,12] <= 2
 w[10,13] = -0
 w[10,14] = -0
 w[10,15] <= 2
 w[10,16] = -0
 w[10,17] = -0
 w[10,18] <= 2
 w[10,19] <= 2
 w[10,20] <= 2
 w[10,21] <= 2
 w[10,22] <= 2
 w[10,23] <= 2
 w[10,24] <= 2
 w[10,25] <= 2
 w[11,0] = -0
 w[11,1] = -0
 w[11,2] = -0
 w[11,3] = -0
 w[11,4] = -0
 w[11,5] = -0
 w[11,6] = -0
 w[11,7] = -0
 w[11,8] <= 2
 w[11,9] <= 2
 w[11,10] <= 2
 w[11,11] = -0
 w[11,12] <= 2
 w[11,13] = -0
 w[11,14] = -0
 w[11,15] = -0
 w[11,16] <= 2
 w[11,17] = -0
 w[11,18] <= 2
 w[11,19] <= 2
 w[11,20] <= 2
 w[11,21] <= 2
 w[11,22] <= 2
 w[11,23] <= 2
 w[11,24] <= 2
 w[11,25] <= 2
 w[12,0] = -0
 w[12,1] = -0
 w[12,2] = -0
 w[12,3] = -0
 w[12,4] = -0
 w[12,5] = -0
 w[12,6] = -0
 w[12,7] = -0
 w[12,8] <= 2
 w[12,9] <= 2
 w[12,10] <= 2
 w[12,11] <= 2
 w[12,12] = -0
 w[12,13] = -0
 w[12,14] = -0
 w[12,15] = -0
 w[12,16] = -0
 w[12,17] <= 2
 w[12,18] <= 2
 w[12,19] <= 2
 w[12,20] <= 2
 w[12,21] <= 2
 w[12,22] <= 2
 w[12,23] <= 2
 w[12,24] <= 2
 w[12,25] <= 2
 w[13,0] = -0
 w[13,1] = -0
 w[13,2] = -0
 w[13,3] = -0
 w[13,4] <= 2
 w[13,5] <= 2
 w[13,6] <= 2
 w[13,7] <= 2
 w[13,8] = -0
 w[13,9] <= 2
 w[13,10] <= 2
 w[13,11] <= 2
 w[13,12] <= 2
 w[13,13] = -0
 w[13,14] <= 2
 w[13,15] <= 2
 w[13,16] <= 2
 w[13,17] <= 2
 w[13,18] = -0
 w[13,19] = -0
 w[13,20] = -0
 w[13,21] = -0
 w[13,22] = -0
 w[13,23] <= 2
 w[13,24] <= 2
 w[13,25] <= 2
 w[14,0] = -0
 w[14,1] = -0
 w[14,2] = -0
 w[14,3] <= 2
 w[14,4] = -0
 w[14,5] <= 2
 w[14,6] <= 2
 w[14,7] <= 2
 w[14,8] <= 2
 w[14,9] = -0
 w[14,10] <= 2
 w[14,11] <= 2
 w[14,12] <= 2
 w[14,13] <= 2
 w[14,14] = -0
 w[14,15] <= 2
 w[14,16] <= 2
 w[14,17] <= 2
 w[14,18] = -0
 w[14,19] = -0
 w[14,20] = -0
 w[14,21] = -0
 w[14,22] = -0
 w[14,23] <= 2
 w[14,24] <= 2
 w[14,25] <= 2
 w[15,0] = -0
 w[15,1] = -0
 w[15,2] = -0
 w[15,3] <= 2
 w[15,4] <= 2
 w[15,5] = -0
 w[15,6] <= 2
 w[15,7] <= 2
 w[15,8] <= 2
 w[15,9] <= 2
 w[15,10] = -0
 w[15,11] <= 2
 w[15,12] <= 2
 w[15,13] <= 2
 w[15,14] <= 2
 w[15,15] = -0
 w[15,16] <= 2
 w[15,17] <= 2
 w[15,18] = -0
 w[15,19] = -0
 w[15,20] = -0
 w[15,21] = -0
 w[15,22] = -0
 w[15,23] <= 2
 w[15,24] <= 2
 w[15,25] <= 2
 w[16,0] = -0
 w[16,1] = -0
 w[16,2] = -0
 w[16,3] <= 2
 w[16,4] <= 2
 w[16,5] <= 2
 w[16,6] = -0
 w[16,7] <= 2
 w[16,8] <= 2
 w[16,9] <= 2
 w[16,10] <= 2
 w[16,11] = -0
 w[16,12] <= 2
 w[16,13] <= 2
 w[16,14] <= 2
 w[16,15] <= 2
 w[16,16] = -0
 w[16,17] <= 2
 w[16,18] = -0
 w[16,19] = -0
 w[16,20] = -0
 w[16,21] = -0
 w[16,22] = -0
 w[16,23] <= 2
 w[16,24] <= 2
 w[16,25] <= 2
 w[17,0] = -0
 w[17,1] = -0
 w[17,2] = -0
 w[17,3] <= 2
 w[17,4] <= 2
 w[17,5] <= 2
 w[17,6] <= 2
 w[17,7] = -0
 w[17,8] <= 2
 w[17,9] <= 2
 w[17,10] <= 2
 w[17,11] <= 2
 w[17,12] = -0
 w[17,13] <= 2
 w[17,14] <= 2
 w[17,15] <= 2
 w[17,16] <= 2
 w[17,17] = -0
 w[17,18] = -0
 w[17,19] = -0
 w[17,20] = -0
 w[17,21] = -0
 w[17,22] = -0
 w[17,23] <= 2
 w[17,24] <= 2
 w[17,25] <= 2
 w[18,0] = -0
 w[18,1] = -0
 w[18,2] = -0
 w[18,3] = -0
 w[18,4] <= 2
 w[18,5] <= 2
 w[18,6] <= 2
 w[18,7] <= 2
 w[18,8] = -0
 w[18,9] <= 2
 w[18,10] <= 2
 w[18,11] <= 2
 w[18,12] <= 2
 w[18,13] = -0
 w[18,14] = -0
 w[18,15] = -0
 w[18,16] = -0
 w[18,17] = -0
 w[18,18] = -0
 w[18,19] <= 2
 w[18,20] <= 2
 w[18,21] <= 2
 w[18,22] <= 2
 w[18,23] <= 2
 w[18,24] <= 2
 w[18,25] <= 2
 w[19,0] = -0
 w[19,1] = -0
 w[19,2] = -0
 w[19,3] <= 2
 w[19,4] = -0
 w[19,5] <= 2
 w[19,6] <= 2
 w[19,7] <= 2
 w[19,8] <= 2
 w[19,9] = -0
 w[19,10] <= 2
 w[19,11] <= 2
 w[19,12] <= 2
 w[19,13] = -0
 w[19,14] = -0
 w[19,15] = -0
 w[19,16] = -0
 w[19,17] = -0
 w[19,18] <= 2
 w[19,19] = -0
 w[19,20] <= 2
 w[19,21] <= 2
 w[19,22] <= 2
 w[19,23] <= 2
 w[19,24] <= 2
 w[19,25] <= 2
 w[20,0] = -0
 w[20,1] = -0
 w[20,2] = -0
 w[20,3] <= 2
 w[20,4] <= 2
 w[20,5] = -0
 w[20,6] <= 2
 w[20,7] <= 2
 w[20,8] <= 2
 w[20,9] <= 2
 w[20,10] = -0
 w[20,11] <= 2
 w[20,12] <= 2
 w[20,13] = -0
 w[20,14] = -0
 w[20,15] = -0
 w[20,16] = -0
 w[20,17] = -0
 w[20,18] <= 2
 w[20,19] <= 2
 w[20,20] = -0
 w[20,21] <= 2
 w[20,22] <= 2
 w[20,23] <= 2
 w[20,24] <= 2
 w[20,25] <= 2
 w[21,0] = -0
 w[21,1] = -0
 w[21,2] = -0
 w[21,3] <= 2
 w[21,4] <= 2
 w[21,5] <= 2
 w[21,6] = -0
 w[21,7] <= 2
 w[21,8] <= 2
 w[21,9] <= 2
 w[21,10] <= 2
 w[21,11] = -0
 w[21,12] <= 2
 w[21,13] = -0
 w[21,14] = -0
 w[21,15] = -0
 w[21,16] = -0
 w[21,17] = -0
 w[21,18] <= 2
 w[21,19] <= 2
 w[21,20] <= 2
 w[21,21] = -0
 w[21,22] <= 2
 w[21,23] <= 2
 w[21,24] <= 2
 w[21,25] <= 2
 w[22,0] = -0
 w[22,1] = -0
 w[22,2] = -0
 w[22,3] <= 2
 w[22,4] <= 2
 w[22,5] <= 2
 w[22,6] <= 2
 w[22,7] = -0
 w[22,8] <= 2
 w[22,9] <= 2
 w[22,10] <= 2
 w[22,11] <= 2
 w[22,12] = -0
 w[22,13] = -0
 w[22,14] = -0
 w[22,15] = -0
 w[22,16] = -0
 w[22,17] = -0
 w[22,18] <= 2
 w[22,19] <= 2
 w[22,20] <= 2
 w[22,21] <= 2
 w[22,22] = -0
 w[22,23] <= 2
 w[22,24] <= 2
 w[22,25] <= 2
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
 w[23,24] <= 2
 w[23,25] <= 2
 w[24,0] = -0
 w[24,1] = -0
 w[24,2] = -0
 w[24,3] = -0
 w[24,4] = -0
 w[24,5] = -0
 w[24,6] = -0
 w[24,7] = -0
 w[24,8] = -0
 w[24,9] = -0
 w[24,10] = -0
 w[24,11] = -0
 w[24,12] = -0
 w[24,13] = -0
 w[24,14] = -0
 w[24,15] = -0
 w[24,16] = -0
 w[24,17] = -0
 w[24,18] = -0
 w[24,19] = -0
 w[24,20] = -0
 w[24,21] = -0
 w[24,22] = -0
 w[24,23] <= 2
 w[24,24] = -0
 w[24,25] <= 2
 w[25,0] = -0
 w[25,1] = -0
 w[25,2] = -0
 w[25,3] = -0
 w[25,4] = -0
 w[25,5] = -0
 w[25,6] = -0
 w[25,7] = -0
 w[25,8] = -0
 w[25,9] = -0
 w[25,10] = -0
 w[25,11] = -0
 w[25,12] = -0
 w[25,13] = -0
 w[25,14] = -0
 w[25,15] = -0
 w[25,16] = -0
 w[25,17] = -0
 w[25,18] = -0
 w[25,19] = -0
 w[25,20] = -0
 w[25,21] = -0
 w[25,22] = -0
 w[25,23] = -0
 w[25,24] = -0
 w[25,25] = -0
 v[0,0] = 0
 v[1,0] = 0
 v[1,1] = 0
 v[2,0] = 0
 v[2,2] = 0
 v[3,0] = 0
 v[3,1] = 0
 v[3,2] = 0
 v[3,3] = 0
 v[3,8] = 0
 v[3,9] = 0
 v[3,10] = 0
 v[3,11] = 0
 v[3,12] = 0
 v[3,19] = 0
 v[3,20] = 0
 v[3,21] = 0
 v[3,22] = 0
 v[4,0] = 0
 v[4,1] = 0
 v[4,2] = 0
 v[4,4] = 0
 v[4,8] = 0
 v[4,9] = 0
 v[4,10] = 0
 v[4,11] = 0
 v[4,12] = 0
 v[4,18] = 0
 v[4,20] = 0
 v[4,21] = 0
 v[4,22] = 0
 v[5,0] = 0
 v[5,1] = 0
 v[5,2] = 0
 v[5,5] = 0
 v[5,8] = 0
 v[5,9] = 0
 v[5,10] = 0
 v[5,11] = 0
 v[5,12] = 0
 v[5,18] = 0
 v[5,19] = 0
 v[5,21] = 0
 v[5,22] = 0
 v[6,0] = 0
 v[6,1] = 0
 v[6,2] = 0
 v[6,6] = 0
 v[6,8] = 0
 v[6,9] = 0
 v[6,10] = 0
 v[6,11] = 0
 v[6,12] = 0
 v[6,18] = 0
 v[6,19] = 0
 v[6,20] = 0
 v[6,22] = 0
 v[7,0] = 0
 v[7,1] = 0
 v[7,2] = 0
 v[7,7] = 0
 v[7,8] = 0
 v[7,9] = 0
 v[7,10] = 0
 v[7,11] = 0
 v[7,12] = 0
 v[7,18] = 0
 v[7,19] = 0
 v[7,20] = 0
 v[7,21] = 0
 v[8,0] = 0
 v[8,1] = 0
 v[8,2] = 0
 v[8,3] = 0
 v[8,4] = 0
 v[8,5] = 0
 v[8,6] = 0
 v[8,7] = 0
 v[8,8] = 0
 v[8,14] = 0
 v[8,15] = 0
 v[8,16] = 0
 v[8,17] = 0
 v[9,0] = 0
 v[9,1] = 0
 v[9,2] = 0
 v[9,3] = 0
 v[9,4] = 0
 v[9,5] = 0
 v[9,6] = 0
 v[9,7] = 0
 v[9,9] = 0
 v[9,13] = 0
 v[9,15] = 0
 v[9,16] = 0
 v[9,17] = 0
 v[10,0] = 0
 v[10,1] = 0
 v[10,2] = 0
 v[10,3] = 0
 v[10,4] = 0
 v[10,5] = 0
 v[10,6] = 0
 v[10,7] = 0
 v[10,10] = 0
 v[10,13] = 0
 v[10,14] = 0
 v[10,16] = 0
 v[10,17] = 0
 v[11,0] = 0
 v[11,1] = 0
 v[11,2] = 0
 v[11,3] = 0
 v[11,4] = 0
 v[11,5] = 0
 v[11,6] = 0
 v[11,7] = 0
 v[11,11] = 0
 v[11,13] = 0
 v[11,14] = 0
 v[11,15] = 0
 v[11,17] = 0
 v[12,0] = 0
 v[12,1] = 0
 v[12,2] = 0
 v[12,3] = 0
 v[12,4] = 0
 v[12,5] = 0
 v[12,6] = 0
 v[12,7] = 0
 v[12,12] = 0
 v[12,13] = 0
 v[12,14] = 0
 v[12,15] = 0
 v[12,16] = 0
 v[13,0] = 0
 v[13,1] = 0
 v[13,2] = 0
 v[13,3] = 0
 v[13,8] = 0
 v[13,13] = 0
 v[13,18] = 0
 v[13,19] = 0
 v[13,20] = 0
 v[13,21] = 0
 v[13,22] = 0
 v[14,0] = 0
 v[14,1] = 0
 v[14,2] = 0
 v[14,4] = 0
 v[14,9] = 0
 v[14,14] = 0
 v[14,18] = 0
 v[14,19] = 0
 v[14,20] = 0
 v[14,21] = 0
 v[14,22] = 0
 v[15,0] = 0
 v[15,1] = 0
 v[15,2] = 0
 v[15,5] = 0
 v[15,10] = 0
 v[15,15] = 0
 v[15,18] = 0
 v[15,19] = 0
 v[15,20] = 0
 v[15,21] = 0
 v[15,22] = 0
 v[16,0] = 0
 v[16,1] = 0
 v[16,2] = 0
 v[16,6] = 0
 v[16,11] = 0
 v[16,16] = 0
 v[16,18] = 0
 v[16,19] = 0
 v[16,20] = 0
 v[16,21] = 0
 v[16,22] = 0
 v[17,0] = 0
 v[17,1] = 0
 v[17,2] = 0
 v[17,7] = 0
 v[17,12] = 0
 v[17,17] = 0
 v[17,18] = 0
 v[17,19] = 0
 v[17,20] = 0
 v[17,21] = 0
 v[17,22] = 0
 v[18,0] = 0
 v[18,1] = 0
 v[18,2] = 0
 v[18,3] = 0
 v[18,8] = 0
 v[18,13] = 0
 v[18,14] = 0
 v[18,15] = 0
 v[18,16] = 0
 v[18,17] = 0
 v[18,18] = 0
 v[19,0] = 0
 v[19,1] = 0
 v[19,2] = 0
 v[19,4] = 0
 v[19,9] = 0
 v[19,13] = 0
 v[19,14] = 0
 v[19,15] = 0
 v[19,16] = 0
 v[19,17] = 0
 v[19,19] = 0
 v[20,0] = 0
 v[20,1] = 0
 v[20,2] = 0
 v[20,5] = 0
 v[20,10] = 0
 v[20,13] = 0
 v[20,14] = 0
 v[20,15] = 0
 v[20,16] = 0
 v[20,17] = 0
 v[20,20] = 0
 v[21,0] = 0
 v[21,1] = 0
 v[21,2] = 0
 v[21,6] = 0
 v[21,11] = 0
 v[21,13] = 0
 v[21,14] = 0
 v[21,15] = 0
 v[21,16] = 0
 v[21,17] = 0
 v[21,21] = 0
 v[22,0] = 0
 v[22,1] = 0
 v[22,2] = 0
 v[22,7] = 0
 v[22,12] = 0
 v[22,13] = 0
 v[22,14] = 0
 v[22,15] = 0
 v[22,16] = 0
 v[22,17] = 0
 v[22,22] = 0
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
 v[24,0] = 0
 v[24,1] = 0
 v[24,2] = 0
 v[24,3] = 0
 v[24,4] = 0
 v[24,5] = 0
 v[24,6] = 0
 v[24,7] = 0
 v[24,8] = 0
 v[24,9] = 0
 v[24,10] = 0
 v[24,11] = 0
 v[24,12] = 0
 v[24,13] = 0
 v[24,14] = 0
 v[24,15] = 0
 v[24,16] = 0
 v[24,17] = 0
 v[24,18] = 0
 v[24,19] = 0
 v[24,20] = 0
 v[24,21] = 0
 v[24,22] = 0
 v[24,24] = 0
 v[25,0] = 0
 v[25,1] = 0
 v[25,2] = 0
 v[25,3] = 0
 v[25,4] = 0
 v[25,5] = 0
 v[25,6] = 0
 v[25,7] = 0
 v[25,8] = 0
 v[25,9] = 0
 v[25,10] = 0
 v[25,11] = 0
 v[25,12] = 0
 v[25,13] = 0
 v[25,14] = 0
 v[25,15] = 0
 v[25,16] = 0
 v[25,17] = 0
 v[25,18] = 0
 v[25,19] = 0
 v[25,20] = 0
 v[25,21] = 0
 v[25,22] = 0
 v[25,23] = 0
 v[25,24] = 0
 v[25,25] = 0
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
 start_service[24] <= 10000
 start_service[25] <= 10000
Binaries
 x[0,0] x[0,1] x[0,2] x[0,3] x[0,4] x[0,5] x[0,6] x[0,7] x[0,8] x[0,9]
 x[0,10] x[0,11] x[0,12] x[0,13] x[0,14] x[0,15] x[0,16] x[0,17] x[0,18]
 x[0,19] x[0,20] x[0,21] x[0,22] x[0,23] x[0,24] x[0,25] x[1,0] x[1,1]
 x[1,2] x[1,3] x[1,4] x[1,5] x[1,6] x[1,7] x[1,8] x[1,9] x[1,10] x[1,11]
 x[1,12] x[1,13] x[1,14] x[1,15] x[1,16] x[1,17] x[1,18] x[1,19] x[1,20]
 x[1,21] x[1,22] x[1,23] x[1,24] x[1,25] x[2,0] x[2,1] x[2,2] x[2,3] x[2,4]
 x[2,5] x[2,6] x[2,7] x[2,8] x[2,9] x[2,10] x[2,11] x[2,12] x[2,13] x[2,14]
 x[2,15] x[2,16] x[2,17] x[2,18] x[2,19] x[2,20] x[2,21] x[2,22] x[2,23]
 x[2,24] x[2,25] x[3,0] x[3,1] x[3,2] x[3,3] x[3,4] x[3,5] x[3,6] x[3,7]
 x[3,8] x[3,9] x[3,10] x[3,11] x[3,12] x[3,13] x[3,14] x[3,15] x[3,16]
 x[3,17] x[3,18] x[3,19] x[3,20] x[3,21] x[3,22] x[3,23] x[3,24] x[3,25]
 x[4,0] x[4,1] x[4,2] x[4,3] x[4,4] x[4,5] x[4,6] x[4,7] x[4,8] x[4,9]
 x[4,10] x[4,11] x[4,12] x[4,13] x[4,14] x[4,15] x[4,16] x[4,17] x[4,18]
 x[4,19] x[4,20] x[4,21] x[4,22] x[4,23] x[4,24] x[4,25] x[5,0] x[5,1]
 x[5,2] x[5,3] x[5,4] x[5,5] x[5,6] x[5,7] x[5,8] x[5,9] x[5,10] x[5,11]
 x[5,12] x[5,13] x[5,14] x[5,15] x[5,16] x[5,17] x[5,18] x[5,19] x[5,20]
 x[5,21] x[5,22] x[5,23] x[5,24] x[5,25] x[6,0] x[6,1] x[6,2] x[6,3] x[6,4]
 x[6,5] x[6,6] x[6,7] x[6,8] x[6,9] x[6,10] x[6,11] x[6,12] x[6,13] x[6,14]
 x[6,15] x[6,16] x[6,17] x[6,18] x[6,19] x[6,20] x[6,21] x[6,22] x[6,23]
 x[6,24] x[6,25] x[7,0] x[7,1] x[7,2] x[7,3] x[7,4] x[7,5] x[7,6] x[7,7]
 x[7,8] x[7,9] x[7,10] x[7,11] x[7,12] x[7,13] x[7,14] x[7,15] x[7,16]
 x[7,17] x[7,18] x[7,19] x[7,20] x[7,21] x[7,22] x[7,23] x[7,24] x[7,25]
 x[8,0] x[8,1] x[8,2] x[8,3] x[8,4] x[8,5] x[8,6] x[8,7] x[8,8] x[8,9]
 x[8,10] x[8,11] x[8,12] x[8,13] x[8,14] x[8,15] x[8,16] x[8,17] x[8,18]
 x[8,19] x[8,20] x[8,21] x[8,22] x[8,23] x[8,24] x[8,25] x[9,0] x[9,1]
 x[9,2] x[9,3] x[9,4] x[9,5] x[9,6] x[9,7] x[9,8] x[9,9] x[9,10] x[9,11]
 x[9,12] x[9,13] x[9,14] x[9,15] x[9,16] x[9,17] x[9,18] x[9,19] x[9,20]
 x[9,21] x[9,22] x[9,23] x[9,24] x[9,25] x[10,0] x[10,1] x[10,2] x[10,3]
 x[10,4] x[10,5] x[10,6] x[10,7] x[10,8] x[10,9] x[10,10] x[10,11] x[10,12]
 x[10,13] x[10,14] x[10,15] x[10,16] x[10,17] x[10,18] x[10,19] x[10,20]
 x[10,21] x[10,22] x[10,23] x[10,24] x[10,25] x[11,0] x[11,1] x[11,2]
 x[11,3] x[11,4] x[11,5] x[11,6] x[11,7] x[11,8] x[11,9] x[11,10] x[11,11]
 x[11,12] x[11,13] x[11,14] x[11,15] x[11,16] x[11,17] x[11,18] x[11,19]
 x[11,20] x[11,21] x[11,22] x[11,23] x[11,24] x[11,25] x[12,0] x[12,1]
 x[12,2] x[12,3] x[12,4] x[12,5] x[12,6] x[12,7] x[12,8] x[12,9] x[12,10]
 x[12,11] x[12,12] x[12,13] x[12,14] x[12,15] x[12,16] x[12,17] x[12,18]
 x[12,19] x[12,20] x[12,21] x[12,22] x[12,23] x[12,24] x[12,25] x[13,0]
 x[13,1] x[13,2] x[13,3] x[13,4] x[13,5] x[13,6] x[13,7] x[13,8] x[13,9]
 x[13,10] x[13,11] x[13,12] x[13,13] x[13,14] x[13,15] x[13,16] x[13,17]
 x[13,18] x[13,19] x[13,20] x[13,21] x[13,22] x[13,23] x[13,24] x[13,25]
 x[14,0] x[14,1] x[14,2] x[14,3] x[14,4] x[14,5] x[14,6] x[14,7] x[14,8]
 x[14,9] x[14,10] x[14,11] x[14,12] x[14,13] x[14,14] x[14,15] x[14,16]
 x[14,17] x[14,18] x[14,19] x[14,20] x[14,21] x[14,22] x[14,23] x[14,24]
 x[14,25] x[15,0] x[15,1] x[15,2] x[15,3] x[15,4] x[15,5] x[15,6] x[15,7]
 x[15,8] x[15,9] x[15,10] x[15,11] x[15,12] x[15,13] x[15,14] x[15,15]
 x[15,16] x[15,17] x[15,18] x[15,19] x[15,20] x[15,21] x[15,22] x[15,23]
 x[15,24] x[15,25] x[16,0] x[16,1] x[16,2] x[16,3] x[16,4] x[16,5] x[16,6]
 x[16,7] x[16,8] x[16,9] x[16,10] x[16,11] x[16,12] x[16,13] x[16,14]
 x[16,15] x[16,16] x[16,17] x[16,18] x[16,19] x[16,20] x[16,21] x[16,22]
 x[16,23] x[16,24] x[16,25] x[17,0] x[17,1] x[17,2] x[17,3] x[17,4] x[17,5]
 x[17,6] x[17,7] x[17,8] x[17,9] x[17,10] x[17,11] x[17,12] x[17,13]
 x[17,14] x[17,15] x[17,16] x[17,17] x[17,18] x[17,19] x[17,20] x[17,21]
 x[17,22] x[17,23] x[17,24] x[17,25] x[18,0] x[18,1] x[18,2] x[18,3]
 x[18,4] x[18,5] x[18,6] x[18,7] x[18,8] x[18,9] x[18,10] x[18,11] x[18,12]
 x[18,13] x[18,14] x[18,15] x[18,16] x[18,17] x[18,18] x[18,19] x[18,20]
 x[18,21] x[18,22] x[18,23] x[18,24] x[18,25] x[19,0] x[19,1] x[19,2]
 x[19,3] x[19,4] x[19,5] x[19,6] x[19,7] x[19,8] x[19,9] x[19,10] x[19,11]
 x[19,12] x[19,13] x[19,14] x[19,15] x[19,16] x[19,17] x[19,18] x[19,19]
 x[19,20] x[19,21] x[19,22] x[19,23] x[19,24] x[19,25] x[20,0] x[20,1]
 x[20,2] x[20,3] x[20,4] x[20,5] x[20,6] x[20,7] x[20,8] x[20,9] x[20,10]
 x[20,11] x[20,12] x[20,13] x[20,14] x[20,15] x[20,16] x[20,17] x[20,18]
 x[20,19] x[20,20] x[20,21] x[20,22] x[20,23] x[20,24] x[20,25] x[21,0]
 x[21,1] x[21,2] x[21,3] x[21,4] x[21,5] x[21,6] x[21,7] x[21,8] x[21,9]
 x[21,10] x[21,11] x[21,12] x[21,13] x[21,14] x[21,15] x[21,16] x[21,17]
 x[21,18] x[21,19] x[21,20] x[21,21] x[21,22] x[21,23] x[21,24] x[21,25]
 x[22,0] x[22,1] x[22,2] x[22,3] x[22,4] x[22,5] x[22,6] x[22,7] x[22,8]
 x[22,9] x[22,10] x[22,11] x[22,12] x[22,13] x[22,14] x[22,15] x[22,16]
 x[22,17] x[22,18] x[22,19] x[22,20] x[22,21] x[22,22] x[22,23] x[22,24]
 x[22,25] x[23,0] x[23,1] x[23,2] x[23,3] x[23,4] x[23,5] x[23,6] x[23,7]
 x[23,8] x[23,9] x[23,10] x[23,11] x[23,12] x[23,13] x[23,14] x[23,15]
 x[23,16] x[23,17] x[23,18] x[23,19] x[23,20] x[23,21] x[23,22] x[23,23]
 x[23,24] x[23,25] x[24,0] x[24,1] x[24,2] x[24,3] x[24,4] x[24,5] x[24,6]
 x[24,7] x[24,8] x[24,9] x[24,10] x[24,11] x[24,12] x[24,13] x[24,14]
 x[24,15] x[24,16] x[24,17] x[24,18] x[24,19] x[24,20] x[24,21] x[24,22]
 x[24,23] x[24,24] x[24,25] x[25,0] x[25,1] x[25,2] x[25,3] x[25,4] x[25,5]
 x[25,6] x[25,7] x[25,8] x[25,9] x[25,10] x[25,11] x[25,12] x[25,13]
 x[25,14] x[25,15] x[25,16] x[25,17] x[25,18] x[25,19] x[25,20] x[25,21]
 x[25,22] x[25,23] x[25,24] x[25,25] v[0,0] v[0,1] v[0,2] v[0,3] v[0,4]
 v[0,5] v[0,6] v[0,7] v[0,8] v[0,9] v[0,10] v[0,11] v[0,12] v[0,13] v[0,14]
 v[0,15] v[0,16] v[0,17] v[0,18] v[0,19] v[0,20] v[0,21] v[0,22] v[0,23]
 v[0,24] v[0,25] v[1,0] v[1,1] v[1,2] v[1,3] v[1,4] v[1,5] v[1,6] v[1,7]
 v[1,8] v[1,9] v[1,10] v[1,11] v[1,12] v[1,13] v[1,14] v[1,15] v[1,16]
 v[1,17] v[1,18] v[1,19] v[1,20] v[1,21] v[1,22] v[1,23] v[1,24] v[1,25]
 v[2,0] v[2,1] v[2,2] v[2,3] v[2,4] v[2,5] v[2,6] v[2,7] v[2,8] v[2,9]
 v[2,10] v[2,11] v[2,12] v[2,13] v[2,14] v[2,15] v[2,16] v[2,17] v[2,18]
 v[2,19] v[2,20] v[2,21] v[2,22] v[2,23] v[2,24] v[2,25] v[3,0] v[3,1]
 v[3,2] v[3,3] v[3,4] v[3,5] v[3,6] v[3,7] v[3,8] v[3,9] v[3,10] v[3,11]
 v[3,12] v[3,13] v[3,14] v[3,15] v[3,16] v[3,17] v[3,18] v[3,19] v[3,20]
 v[3,21] v[3,22] v[3,23] v[3,24] v[3,25] v[4,0] v[4,1] v[4,2] v[4,3] v[4,4]
 v[4,5] v[4,6] v[4,7] v[4,8] v[4,9] v[4,10] v[4,11] v[4,12] v[4,13] v[4,14]
 v[4,15] v[4,16] v[4,17] v[4,18] v[4,19] v[4,20] v[4,21] v[4,22] v[4,23]
 v[4,24] v[4,25] v[5,0] v[5,1] v[5,2] v[5,3] v[5,4] v[5,5] v[5,6] v[5,7]
 v[5,8] v[5,9] v[5,10] v[5,11] v[5,12] v[5,13] v[5,14] v[5,15] v[5,16]
 v[5,17] v[5,18] v[5,19] v[5,20] v[5,21] v[5,22] v[5,23] v[5,24] v[5,25]
 v[6,0] v[6,1] v[6,2] v[6,3] v[6,4] v[6,5] v[6,6] v[6,7] v[6,8] v[6,9]
 v[6,10] v[6,11] v[6,12] v[6,13] v[6,14] v[6,15] v[6,16] v[6,17] v[6,18]
 v[6,19] v[6,20] v[6,21] v[6,22] v[6,23] v[6,24] v[6,25] v[7,0] v[7,1]
 v[7,2] v[7,3] v[7,4] v[7,5] v[7,6] v[7,7] v[7,8] v[7,9] v[7,10] v[7,11]
 v[7,12] v[7,13] v[7,14] v[7,15] v[7,16] v[7,17] v[7,18] v[7,19] v[7,20]
 v[7,21] v[7,22] v[7,23] v[7,24] v[7,25] v[8,0] v[8,1] v[8,2] v[8,3] v[8,4]
 v[8,5] v[8,6] v[8,7] v[8,8] v[8,9] v[8,10] v[8,11] v[8,12] v[8,13] v[8,14]
 v[8,15] v[8,16] v[8,17] v[8,18] v[8,19] v[8,20] v[8,21] v[8,22] v[8,23]
 v[8,24] v[8,25] v[9,0] v[9,1] v[9,2] v[9,3] v[9,4] v[9,5] v[9,6] v[9,7]
 v[9,8] v[9,9] v[9,10] v[9,11] v[9,12] v[9,13] v[9,14] v[9,15] v[9,16]
 v[9,17] v[9,18] v[9,19] v[9,20] v[9,21] v[9,22] v[9,23] v[9,24] v[9,25]
 v[10,0] v[10,1] v[10,2] v[10,3] v[10,4] v[10,5] v[10,6] v[10,7] v[10,8]
 v[10,9] v[10,10] v[10,11] v[10,12] v[10,13] v[10,14] v[10,15] v[10,16]
 v[10,17] v[10,18] v[10,19] v[10,20] v[10,21] v[10,22] v[10,23] v[10,24]
 v[10,25] v[11,0] v[11,1] v[11,2] v[11,3] v[11,4] v[11,5] v[11,6] v[11,7]
 v[11,8] v[11,9] v[11,10] v[11,11] v[11,12] v[11,13] v[11,14] v[11,15]
 v[11,16] v[11,17] v[11,18] v[11,19] v[11,20] v[11,21] v[11,22] v[11,23]
 v[11,24] v[11,25] v[12,0] v[12,1] v[12,2] v[12,3] v[12,4] v[12,5] v[12,6]
 v[12,7] v[12,8] v[12,9] v[12,10] v[12,11] v[12,12] v[12,13] v[12,14]
 v[12,15] v[12,16] v[12,17] v[12,18] v[12,19] v[12,20] v[12,21] v[12,22]
 v[12,23] v[12,24] v[12,25] v[13,0] v[13,1] v[13,2] v[13,3] v[13,4] v[13,5]
 v[13,6] v[13,7] v[13,8] v[13,9] v[13,10] v[13,11] v[13,12] v[13,13]
 v[13,14] v[13,15] v[13,16] v[13,17] v[13,18] v[13,19] v[13,20] v[13,21]
 v[13,22] v[13,23] v[13,24] v[13,25] v[14,0] v[14,1] v[14,2] v[14,3]
 v[14,4] v[14,5] v[14,6] v[14,7] v[14,8] v[14,9] v[14,10] v[14,11] v[14,12]
 v[14,13] v[14,14] v[14,15] v[14,16] v[14,17] v[14,18] v[14,19] v[14,20]
 v[14,21] v[14,22] v[14,23] v[14,24] v[14,25] v[15,0] v[15,1] v[15,2]
 v[15,3] v[15,4] v[15,5] v[15,6] v[15,7] v[15,8] v[15,9] v[15,10] v[15,11]
 v[15,12] v[15,13] v[15,14] v[15,15] v[15,16] v[15,17] v[15,18] v[15,19]
 v[15,20] v[15,21] v[15,22] v[15,23] v[15,24] v[15,25] v[16,0] v[16,1]
 v[16,2] v[16,3] v[16,4] v[16,5] v[16,6] v[16,7] v[16,8] v[16,9] v[16,10]
 v[16,11] v[16,12] v[16,13] v[16,14] v[16,15] v[16,16] v[16,17] v[16,18]
 v[16,19] v[16,20] v[16,21] v[16,22] v[16,23] v[16,24] v[16,25] v[17,0]
 v[17,1] v[17,2] v[17,3] v[17,4] v[17,5] v[17,6] v[17,7] v[17,8] v[17,9]
 v[17,10] v[17,11] v[17,12] v[17,13] v[17,14] v[17,15] v[17,16] v[17,17]
 v[17,18] v[17,19] v[17,20] v[17,21] v[17,22] v[17,23] v[17,24] v[17,25]
 v[18,0] v[18,1] v[18,2] v[18,3] v[18,4] v[18,5] v[18,6] v[18,7] v[18,8]
 v[18,9] v[18,10] v[18,11] v[18,12] v[18,13] v[18,14] v[18,15] v[18,16]
 v[18,17] v[18,18] v[18,19] v[18,20] v[18,21] v[18,22] v[18,23] v[18,24]
 v[18,25] v[19,0] v[19,1] v[19,2] v[19,3] v[19,4] v[19,5] v[19,6] v[19,7]
 v[19,8] v[19,9] v[19,10] v[19,11] v[19,12] v[19,13] v[19,14] v[19,15]
 v[19,16] v[19,17] v[19,18] v[19,19] v[19,20] v[19,21] v[19,22] v[19,23]
 v[19,24] v[19,25] v[20,0] v[20,1] v[20,2] v[20,3] v[20,4] v[20,5] v[20,6]
 v[20,7] v[20,8] v[20,9] v[20,10] v[20,11] v[20,12] v[20,13] v[20,14]
 v[20,15] v[20,16] v[20,17] v[20,18] v[20,19] v[20,20] v[20,21] v[20,22]
 v[20,23] v[20,24] v[20,25] v[21,0] v[21,1] v[21,2] v[21,3] v[21,4] v[21,5]
 v[21,6] v[21,7] v[21,8] v[21,9] v[21,10] v[21,11] v[21,12] v[21,13]
 v[21,14] v[21,15] v[21,16] v[21,17] v[21,18] v[21,19] v[21,20] v[21,21]
 v[21,22] v[21,23] v[21,24] v[21,25] v[22,0] v[22,1] v[22,2] v[22,3]
 v[22,4] v[22,5] v[22,6] v[22,7] v[22,8] v[22,9] v[22,10] v[22,11] v[22,12]
 v[22,13] v[22,14] v[22,15] v[22,16] v[22,17] v[22,18] v[22,19] v[22,20]
 v[22,21] v[22,22] v[22,23] v[22,24] v[22,25] v[23,0] v[23,1] v[23,2]
 v[23,3] v[23,4] v[23,5] v[23,6] v[23,7] v[23,8] v[23,9] v[23,10] v[23,11]
 v[23,12] v[23,13] v[23,14] v[23,15] v[23,16] v[23,17] v[23,18] v[23,19]
 v[23,20] v[23,21] v[23,22] v[23,23] v[23,24] v[23,25] v[24,0] v[24,1]
 v[24,2] v[24,3] v[24,4] v[24,5] v[24,6] v[24,7] v[24,8] v[24,9] v[24,10]
 v[24,11] v[24,12] v[24,13] v[24,14] v[24,15] v[24,16] v[24,17] v[24,18]
 v[24,19] v[24,20] v[24,21] v[24,22] v[24,23] v[24,24] v[24,25] v[25,0]
 v[25,1] v[25,2] v[25,3] v[25,4] v[25,5] v[25,6] v[25,7] v[25,8] v[25,9]
 v[25,10] v[25,11] v[25,12] v[25,13] v[25,14] v[25,15] v[25,16] v[25,17]
 v[25,18] v[25,19] v[25,20] v[25,21] v[25,22] v[25,23] v[25,24] v[25,25]
 y[3,13,1] y[3,13,2] y[3,18,1] y[3,18,2] y[4,14,1] y[4,14,2] y[4,19,1]
 y[4,19,2] y[5,15,1] y[5,15,2] y[5,20,1] y[5,20,2] y[6,16,1] y[6,16,2]
 y[6,21,1] y[6,21,2] y[7,17,1] y[7,17,2] y[7,22,1] y[7,22,2] y[8,13,1]
 y[8,13,2] y[8,18,1] y[8,18,2] y[9,14,1] y[9,14,2] y[9,19,1] y[9,19,2]
 y[10,15,1] y[10,15,2] y[10,20,1] y[10,20,2] y[11,16,1] y[11,16,2]
 y[11,21,1] y[11,21,2] y[12,17,1] y[12,17,2] y[12,22,1] y[12,22,2]
Generals
 w[0,0] w[0,1] w[0,2] w[0,3] w[0,4] w[0,5] w[0,6] w[0,7] w[0,8] w[0,9]
 w[0,10] w[0,11] w[0,12] w[0,13] w[0,14] w[0,15] w[0,16] w[0,17] w[0,18]
 w[0,19] w[0,20] w[0,21] w[0,22] w[0,23] w[0,24] w[0,25] w[1,0] w[1,1]
 w[1,2] w[1,3] w[1,4] w[1,5] w[1,6] w[1,7] w[1,8] w[1,9] w[1,10] w[1,11]
 w[1,12] w[1,13] w[1,14] w[1,15] w[1,16] w[1,17] w[1,18] w[1,19] w[1,20]
 w[1,21] w[1,22] w[1,23] w[1,24] w[1,25] w[2,0] w[2,1] w[2,2] w[2,3] w[2,4]
 w[2,5] w[2,6] w[2,7] w[2,8] w[2,9] w[2,10] w[2,11] w[2,12] w[2,13] w[2,14]
 w[2,15] w[2,16] w[2,17] w[2,18] w[2,19] w[2,20] w[2,21] w[2,22] w[2,23]
 w[2,24] w[2,25] w[3,0] w[3,1] w[3,2] w[3,3] w[3,4] w[3,5] w[3,6] w[3,7]
 w[3,8] w[3,9] w[3,10] w[3,11] w[3,12] w[3,13] w[3,14] w[3,15] w[3,16]
 w[3,17] w[3,18] w[3,19] w[3,20] w[3,21] w[3,22] w[3,23] w[3,24] w[3,25]
 w[4,0] w[4,1] w[4,2] w[4,3] w[4,4] w[4,5] w[4,6] w[4,7] w[4,8] w[4,9]
 w[4,10] w[4,11] w[4,12] w[4,13] w[4,14] w[4,15] w[4,16] w[4,17] w[4,18]
 w[4,19] w[4,20] w[4,21] w[4,22] w[4,23] w[4,24] w[4,25] w[5,0] w[5,1]
 w[5,2] w[5,3] w[5,4] w[5,5] w[5,6] w[5,7] w[5,8] w[5,9] w[5,10] w[5,11]
 w[5,12] w[5,13] w[5,14] w[5,15] w[5,16] w[5,17] w[5,18] w[5,19] w[5,20]
 w[5,21] w[5,22] w[5,23] w[5,24] w[5,25] w[6,0] w[6,1] w[6,2] w[6,3] w[6,4]
 w[6,5] w[6,6] w[6,7] w[6,8] w[6,9] w[6,10] w[6,11] w[6,12] w[6,13] w[6,14]
 w[6,15] w[6,16] w[6,17] w[6,18] w[6,19] w[6,20] w[6,21] w[6,22] w[6,23]
 w[6,24] w[6,25] w[7,0] w[7,1] w[7,2] w[7,3] w[7,4] w[7,5] w[7,6] w[7,7]
 w[7,8] w[7,9] w[7,10] w[7,11] w[7,12] w[7,13] w[7,14] w[7,15] w[7,16]
 w[7,17] w[7,18] w[7,19] w[7,20] w[7,21] w[7,22] w[7,23] w[7,24] w[7,25]
 w[8,0] w[8,1] w[8,2] w[8,3] w[8,4] w[8,5] w[8,6] w[8,7] w[8,8] w[8,9]
 w[8,10] w[8,11] w[8,12] w[8,13] w[8,14] w[8,15] w[8,16] w[8,17] w[8,18]
 w[8,19] w[8,20] w[8,21] w[8,22] w[8,23] w[8,24] w[8,25] w[9,0] w[9,1]
 w[9,2] w[9,3] w[9,4] w[9,5] w[9,6] w[9,7] w[9,8] w[9,9] w[9,10] w[9,11]
 w[9,12] w[9,13] w[9,14] w[9,15] w[9,16] w[9,17] w[9,18] w[9,19] w[9,20]
 w[9,21] w[9,22] w[9,23] w[9,24] w[9,25] w[10,0] w[10,1] w[10,2] w[10,3]
 w[10,4] w[10,5] w[10,6] w[10,7] w[10,8] w[10,9] w[10,10] w[10,11] w[10,12]
 w[10,13] w[10,14] w[10,15] w[10,16] w[10,17] w[10,18] w[10,19] w[10,20]
 w[10,21] w[10,22] w[10,23] w[10,24] w[10,25] w[11,0] w[11,1] w[11,2]
 w[11,3] w[11,4] w[11,5] w[11,6] w[11,7] w[11,8] w[11,9] w[11,10] w[11,11]
 w[11,12] w[11,13] w[11,14] w[11,15] w[11,16] w[11,17] w[11,18] w[11,19]
 w[11,20] w[11,21] w[11,22] w[11,23] w[11,24] w[11,25] w[12,0] w[12,1]
 w[12,2] w[12,3] w[12,4] w[12,5] w[12,6] w[12,7] w[12,8] w[12,9] w[12,10]
 w[12,11] w[12,12] w[12,13] w[12,14] w[12,15] w[12,16] w[12,17] w[12,18]
 w[12,19] w[12,20] w[12,21] w[12,22] w[12,23] w[12,24] w[12,25] w[13,0]
 w[13,1] w[13,2] w[13,3] w[13,4] w[13,5] w[13,6] w[13,7] w[13,8] w[13,9]
 w[13,10] w[13,11] w[13,12] w[13,13] w[13,14] w[13,15] w[13,16] w[13,17]
 w[13,18] w[13,19] w[13,20] w[13,21] w[13,22] w[13,23] w[13,24] w[13,25]
 w[14,0] w[14,1] w[14,2] w[14,3] w[14,4] w[14,5] w[14,6] w[14,7] w[14,8]
 w[14,9] w[14,10] w[14,11] w[14,12] w[14,13] w[14,14] w[14,15] w[14,16]
 w[14,17] w[14,18] w[14,19] w[14,20] w[14,21] w[14,22] w[14,23] w[14,24]
 w[14,25] w[15,0] w[15,1] w[15,2] w[15,3] w[15,4] w[15,5] w[15,6] w[15,7]
 w[15,8] w[15,9] w[15,10] w[15,11] w[15,12] w[15,13] w[15,14] w[15,15]
 w[15,16] w[15,17] w[15,18] w[15,19] w[15,20] w[15,21] w[15,22] w[15,23]
 w[15,24] w[15,25] w[16,0] w[16,1] w[16,2] w[16,3] w[16,4] w[16,5] w[16,6]
 w[16,7] w[16,8] w[16,9] w[16,10] w[16,11] w[16,12] w[16,13] w[16,14]
 w[16,15] w[16,16] w[16,17] w[16,18] w[16,19] w[16,20] w[16,21] w[16,22]
 w[16,23] w[16,24] w[16,25] w[17,0] w[17,1] w[17,2] w[17,3] w[17,4] w[17,5]
 w[17,6] w[17,7] w[17,8] w[17,9] w[17,10] w[17,11] w[17,12] w[17,13]
 w[17,14] w[17,15] w[17,16] w[17,17] w[17,18] w[17,19] w[17,20] w[17,21]
 w[17,22] w[17,23] w[17,24] w[17,25] w[18,0] w[18,1] w[18,2] w[18,3]
 w[18,4] w[18,5] w[18,6] w[18,7] w[18,8] w[18,9] w[18,10] w[18,11] w[18,12]
 w[18,13] w[18,14] w[18,15] w[18,16] w[18,17] w[18,18] w[18,19] w[18,20]
 w[18,21] w[18,22] w[18,23] w[18,24] w[18,25] w[19,0] w[19,1] w[19,2]
 w[19,3] w[19,4] w[19,5] w[19,6] w[19,7] w[19,8] w[19,9] w[19,10] w[19,11]
 w[19,12] w[19,13] w[19,14] w[19,15] w[19,16] w[19,17] w[19,18] w[19,19]
 w[19,20] w[19,21] w[19,22] w[19,23] w[19,24] w[19,25] w[20,0] w[20,1]
 w[20,2] w[20,3] w[20,4] w[20,5] w[20,6] w[20,7] w[20,8] w[20,9] w[20,10]
 w[20,11] w[20,12] w[20,13] w[20,14] w[20,15] w[20,16] w[20,17] w[20,18]
 w[20,19] w[20,20] w[20,21] w[20,22] w[20,23] w[20,24] w[20,25] w[21,0]
 w[21,1] w[21,2] w[21,3] w[21,4] w[21,5] w[21,6] w[21,7] w[21,8] w[21,9]
 w[21,10] w[21,11] w[21,12] w[21,13] w[21,14] w[21,15] w[21,16] w[21,17]
 w[21,18] w[21,19] w[21,20] w[21,21] w[21,22] w[21,23] w[21,24] w[21,25]
 w[22,0] w[22,1] w[22,2] w[22,3] w[22,4] w[22,5] w[22,6] w[22,7] w[22,8]
 w[22,9] w[22,10] w[22,11] w[22,12] w[22,13] w[22,14] w[22,15] w[22,16]
 w[22,17] w[22,18] w[22,19] w[22,20] w[22,21] w[22,22] w[22,23] w[22,24]
 w[22,25] w[23,0] w[23,1] w[23,2] w[23,3] w[23,4] w[23,5] w[23,6] w[23,7]
 w[23,8] w[23,9] w[23,10] w[23,11] w[23,12] w[23,13] w[23,14] w[23,15]
 w[23,16] w[23,17] w[23,18] w[23,19] w[23,20] w[23,21] w[23,22] w[23,23]
 w[23,24] w[23,25] w[24,0] w[24,1] w[24,2] w[24,3] w[24,4] w[24,5] w[24,6]
 w[24,7] w[24,8] w[24,9] w[24,10] w[24,11] w[24,12] w[24,13] w[24,14]
 w[24,15] w[24,16] w[24,17] w[24,18] w[24,19] w[24,20] w[24,21] w[24,22]
 w[24,23] w[24,24] w[24,25] w[25,0] w[25,1] w[25,2] w[25,3] w[25,4] w[25,5]
 w[25,6] w[25,7] w[25,8] w[25,9] w[25,10] w[25,11] w[25,12] w[25,13]
 w[25,14] w[25,15] w[25,16] w[25,17] w[25,18] w[25,19] w[25,20] w[25,21]
 w[25,22] w[25,23] w[25,24] w[25,25]
End
