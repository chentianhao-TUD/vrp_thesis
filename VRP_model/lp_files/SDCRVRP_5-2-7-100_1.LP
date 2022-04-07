\ Model SDCRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[25,0] + 0 x[25,4] + 0 x[25,5] + 0 x[25,6] + 0 x[25,7] + 0 x[25,8]
   + 0 x[25,9] + 0 x[25,10] + 0 x[25,11] + 0 x[25,12] + 0 x[25,13]
   + 0 x[25,14] + 0 x[25,15] + 0 x[25,16] + 0 x[25,17] + 0 x[25,18]
   + 0 x[25,19] + 0 x[25,20] + 0 x[25,21] + 0 x[25,22] + 0 x[25,25]
   + 0 w[0,0] + 0 w[25,0] + 0 w[25,1] + 0 w[25,2] + 0 w[25,3] + 0 w[25,4]
   + 0 w[25,5] + 0 w[25,6] + 0 w[25,7] + 0 w[25,8] + 0 w[25,9] + 0 w[25,10]
   + 0 w[25,11] + 0 w[25,12] + 0 w[25,23] + 0 w[25,24] + 0 w[25,25]
   + 0 v[0,0] + 0 v[1,0] + 0 v[2,0] + 0 v[3,0] + 0 v[4,0] + 0 v[5,0]
   + 0 v[6,0] + 0 v[7,0] + 0 v[8,0] + 0 v[9,0] + 0 v[10,0] + 0 v[11,0]
   + 0 v[12,0] + 0 v[13,0] + 0 v[14,0] + 0 v[15,0] + 0 v[16,0] + 0 v[17,0]
   + 0 v[18,0] + 0 v[19,0] + 0 v[20,0] + 0 v[21,0] + 0 v[22,0] + 0 v[23,0]
   + 0 v[24,0] + 0 v[25,0] + 0 v[25,1] + 0 v[25,2] + 0 v[25,3] + 0 v[25,4]
   + 0 v[25,5] + 0 v[25,6] + 0 v[25,7] + 0 v[25,8] + 0 v[25,9] + 0 v[25,10]
   + 0 v[25,11] + 0 v[25,12] + 0 v[25,13] + 0 v[25,14] + 0 v[25,15]
   + 0 v[25,16] + 0 v[25,17] + 0 v[25,18] + 0 v[25,19] + 0 v[25,20]
   + 0 v[25,21] + 0 v[25,22] + 0 v[25,23] + 0 v[25,24] + 0 v[25,25]
   + start_service[25]
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
 NodeArrivalVehicles[0,3]: - 1.0000563369351183e+06 x[0,3]
   - start_service[0] + start_service[3] >= -999999
 NodeArrivalVehicles[0,4]: - 1.0000371105987907e+06 x[0,4]
   - start_service[0] + start_service[4] >= -999999
 NodeArrivalVehicles[0,5]: - 1.0000331317466504e+06 x[0,5]
   - start_service[0] + start_service[5] >= -999999
 NodeArrivalVehicles[0,6]: - 1.000022879122035e+06 x[0,6]
   - start_service[0] + start_service[6] >= -999999
 NodeArrivalVehicles[0,7]: - 1.0000241592465249e+06 x[0,7]
   - start_service[0] + start_service[7] >= -999999
 NodeArrivalVehicles[0,8]: - 1.0000563369351183e+06 x[0,8]
   - start_service[0] + start_service[8] >= -999999
 NodeArrivalVehicles[0,9]: - 1.0000371105987907e+06 x[0,9]
   - start_service[0] + start_service[9] >= -999999
 NodeArrivalVehicles[0,10]: - 1.0000331317466504e+06 x[0,10]
   - start_service[0] + start_service[10] >= -999999
 NodeArrivalVehicles[0,11]: - 1.000022879122035e+06 x[0,11]
   - start_service[0] + start_service[11] >= -999999
 NodeArrivalVehicles[0,12]: - 1.0000241592465249e+06 x[0,12]
   - start_service[0] + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 1.0000563369351183e+06 x[0,13]
   - start_service[0] + start_service[13] >= -999999
 NodeArrivalVehicles[0,14]: - 1.0000371105987907e+06 x[0,14]
   - start_service[0] + start_service[14] >= -999999
 NodeArrivalVehicles[0,15]: - 1.0000331317466504e+06 x[0,15]
   - start_service[0] + start_service[15] >= -999999
 NodeArrivalVehicles[0,16]: - 1.000022879122035e+06 x[0,16]
   - start_service[0] + start_service[16] >= -999999
 NodeArrivalVehicles[0,17]: - 1.0000241592465249e+06 x[0,17]
   - start_service[0] + start_service[17] >= -999999
 NodeArrivalVehicles[0,18]: - 1.0000563369351183e+06 x[0,18]
   - start_service[0] + start_service[18] >= -999999
 NodeArrivalVehicles[0,19]: - 1.0000371105987907e+06 x[0,19]
   - start_service[0] + start_service[19] >= -999999
 NodeArrivalVehicles[0,20]: - 1.0000331317466504e+06 x[0,20]
   - start_service[0] + start_service[20] >= -999999
 NodeArrivalVehicles[0,21]: - 1.000022879122035e+06 x[0,21]
   - start_service[0] + start_service[21] >= -999999
 NodeArrivalVehicles[0,22]: - 1.0000241592465249e+06 x[0,22]
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
 NodeArrivalVehicles[1,3]: - 1.0000563369351183e+06 x[1,3]
   - start_service[1] + start_service[3] >= -999999
 NodeArrivalVehicles[1,4]: - 1.0000371105987907e+06 x[1,4]
   - start_service[1] + start_service[4] >= -999999
 NodeArrivalVehicles[1,5]: - 1.0000331317466504e+06 x[1,5]
   - start_service[1] + start_service[5] >= -999999
 NodeArrivalVehicles[1,6]: - 1.000022879122035e+06 x[1,6]
   - start_service[1] + start_service[6] >= -999999
 NodeArrivalVehicles[1,7]: - 1.0000241592465249e+06 x[1,7]
   - start_service[1] + start_service[7] >= -999999
 NodeArrivalVehicles[1,8]: - 1.0000563369351183e+06 x[1,8]
   - start_service[1] + start_service[8] >= -999999
 NodeArrivalVehicles[1,9]: - 1.0000371105987907e+06 x[1,9]
   - start_service[1] + start_service[9] >= -999999
 NodeArrivalVehicles[1,10]: - 1.0000331317466504e+06 x[1,10]
   - start_service[1] + start_service[10] >= -999999
 NodeArrivalVehicles[1,11]: - 1.000022879122035e+06 x[1,11]
   - start_service[1] + start_service[11] >= -999999
 NodeArrivalVehicles[1,12]: - 1.0000241592465249e+06 x[1,12]
   - start_service[1] + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 1.0000563369351183e+06 x[1,13]
   - start_service[1] + start_service[13] >= -999999
 NodeArrivalVehicles[1,14]: - 1.0000371105987907e+06 x[1,14]
   - start_service[1] + start_service[14] >= -999999
 NodeArrivalVehicles[1,15]: - 1.0000331317466504e+06 x[1,15]
   - start_service[1] + start_service[15] >= -999999
 NodeArrivalVehicles[1,16]: - 1.000022879122035e+06 x[1,16]
   - start_service[1] + start_service[16] >= -999999
 NodeArrivalVehicles[1,17]: - 1.0000241592465249e+06 x[1,17]
   - start_service[1] + start_service[17] >= -999999
 NodeArrivalVehicles[1,18]: - 1.0000563369351183e+06 x[1,18]
   - start_service[1] + start_service[18] >= -999999
 NodeArrivalVehicles[1,19]: - 1.0000371105987907e+06 x[1,19]
   - start_service[1] + start_service[19] >= -999999
 NodeArrivalVehicles[1,20]: - 1.0000331317466504e+06 x[1,20]
   - start_service[1] + start_service[20] >= -999999
 NodeArrivalVehicles[1,21]: - 1.000022879122035e+06 x[1,21]
   - start_service[1] + start_service[21] >= -999999
 NodeArrivalVehicles[1,22]: - 1.0000241592465249e+06 x[1,22]
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
 NodeArrivalVehicles[2,3]: - 1.0000563369351183e+06 x[2,3]
   - start_service[2] + start_service[3] >= -999999
 NodeArrivalVehicles[2,4]: - 1.0000371105987907e+06 x[2,4]
   - start_service[2] + start_service[4] >= -999999
 NodeArrivalVehicles[2,5]: - 1.0000331317466504e+06 x[2,5]
   - start_service[2] + start_service[5] >= -999999
 NodeArrivalVehicles[2,6]: - 1.000022879122035e+06 x[2,6]
   - start_service[2] + start_service[6] >= -999999
 NodeArrivalVehicles[2,7]: - 1.0000241592465249e+06 x[2,7]
   - start_service[2] + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 1.0000563369351183e+06 x[2,8]
   - start_service[2] + start_service[8] >= -999999
 NodeArrivalVehicles[2,9]: - 1.0000371105987907e+06 x[2,9]
   - start_service[2] + start_service[9] >= -999999
 NodeArrivalVehicles[2,10]: - 1.0000331317466504e+06 x[2,10]
   - start_service[2] + start_service[10] >= -999999
 NodeArrivalVehicles[2,11]: - 1.000022879122035e+06 x[2,11]
   - start_service[2] + start_service[11] >= -999999
 NodeArrivalVehicles[2,12]: - 1.0000241592465249e+06 x[2,12]
   - start_service[2] + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 1.0000563369351183e+06 x[2,13]
   - start_service[2] + start_service[13] >= -999999
 NodeArrivalVehicles[2,14]: - 1.0000371105987907e+06 x[2,14]
   - start_service[2] + start_service[14] >= -999999
 NodeArrivalVehicles[2,15]: - 1.0000331317466504e+06 x[2,15]
   - start_service[2] + start_service[15] >= -999999
 NodeArrivalVehicles[2,16]: - 1.000022879122035e+06 x[2,16]
   - start_service[2] + start_service[16] >= -999999
 NodeArrivalVehicles[2,17]: - 1.0000241592465249e+06 x[2,17]
   - start_service[2] + start_service[17] >= -999999
 NodeArrivalVehicles[2,18]: - 1.0000563369351183e+06 x[2,18]
   - start_service[2] + start_service[18] >= -999999
 NodeArrivalVehicles[2,19]: - 1.0000371105987907e+06 x[2,19]
   - start_service[2] + start_service[19] >= -999999
 NodeArrivalVehicles[2,20]: - 1.0000331317466504e+06 x[2,20]
   - start_service[2] + start_service[20] >= -999999
 NodeArrivalVehicles[2,21]: - 1.000022879122035e+06 x[2,21]
   - start_service[2] + start_service[21] >= -999999
 NodeArrivalVehicles[2,22]: - 1.0000241592465249e+06 x[2,22]
   - start_service[2] + start_service[22] >= -999999
 NodeArrivalVehicles[2,23]: - 999999.01 x[2,23] - start_service[2]
   + start_service[23] >= -999999
 NodeArrivalVehicles[2,24]: - 999999.01 x[2,24] - start_service[2]
   + start_service[24] >= -999999
 NodeArrivalVehicles[2,25]: - 999999.01 x[2,25] - start_service[2]
   + start_service[25] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000563369351183e+06 x[3,1]
   + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.0000563369351183e+06 x[3,2]
   + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999.01 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0000456469522376e+06 x[3,4]
   - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.000028055364666e+06 x[3,5]
   - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.0000341454712306e+06 x[3,6]
   - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.0000688935928658e+06 x[3,7]
   - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 999999.01 x[3,8] - start_service[3]
   + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0000456469522376e+06 x[3,9]
   - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.000028055364666e+06 x[3,10]
   - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.0000341454712306e+06 x[3,11]
   - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 1.0000688935928658e+06 x[3,12]
   - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 999999.01 x[3,13] - start_service[3]
   + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 1.0000456469522376e+06 x[3,14]
   - start_service[3] + start_service[14] >= -999999
 NodeArrivalVehicles[3,15]: - 1.000028055364666e+06 x[3,15]
   - start_service[3] + start_service[15] >= -999999
 NodeArrivalVehicles[3,16]: - 1.0000341454712306e+06 x[3,16]
   - start_service[3] + start_service[16] >= -999999
 NodeArrivalVehicles[3,17]: - 1.0000688935928658e+06 x[3,17]
   - start_service[3] + start_service[17] >= -999999
 NodeArrivalVehicles[3,18]: - 999999.01 x[3,18] - start_service[3]
   + start_service[18] >= -999999
 NodeArrivalVehicles[3,19]: - 1.0000456469522376e+06 x[3,19]
   - start_service[3] + start_service[19] >= -999999
 NodeArrivalVehicles[3,20]: - 1.000028055364666e+06 x[3,20]
   - start_service[3] + start_service[20] >= -999999
 NodeArrivalVehicles[3,21]: - 1.0000341454712306e+06 x[3,21]
   - start_service[3] + start_service[21] >= -999999
 NodeArrivalVehicles[3,22]: - 1.0000688935928658e+06 x[3,22]
   - start_service[3] + start_service[22] >= -999999
 NodeArrivalVehicles[3,23]: - 1.0000563369351183e+06 x[3,23]
   - start_service[3] + start_service[23] >= -999999
 NodeArrivalVehicles[3,24]: - 1.0000563369351183e+06 x[3,24]
   - start_service[3] + start_service[24] >= -999999
 NodeArrivalVehicles[3,25]: - 1.0000563369351183e+06 x[3,25]
   - start_service[3] + start_service[25] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000371105987907e+06 x[4,1]
   + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.0000371105987907e+06 x[4,2]
   + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0000456469522376e+06 x[4,3]
   + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999.01 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.0000188499797942e+06 x[4,5]
   - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000229976847993e+06 x[4,6]
   - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000305908017206e+06 x[4,7]
   - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0000456469522376e+06 x[4,8]
   - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 999999.01 x[4,9] - start_service[4]
   + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.0000188499797942e+06 x[4,10]
   - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000229976847993e+06 x[4,11]
   - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 1.0000305908017206e+06 x[4,12]
   - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 1.0000456469522376e+06 x[4,13]
   - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[4,14]: - 999999.01 x[4,14] - start_service[4]
   + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 1.0000188499797942e+06 x[4,15]
   - start_service[4] + start_service[15] >= -999999
 NodeArrivalVehicles[4,16]: - 1.0000229976847993e+06 x[4,16]
   - start_service[4] + start_service[16] >= -999999
 NodeArrivalVehicles[4,17]: - 1.0000305908017206e+06 x[4,17]
   - start_service[4] + start_service[17] >= -999999
 NodeArrivalVehicles[4,18]: - 1.0000456469522376e+06 x[4,18]
   - start_service[4] + start_service[18] >= -999999
 NodeArrivalVehicles[4,19]: - 999999.01 x[4,19] - start_service[4]
   + start_service[19] >= -999999
 NodeArrivalVehicles[4,20]: - 1.0000188499797942e+06 x[4,20]
   - start_service[4] + start_service[20] >= -999999
 NodeArrivalVehicles[4,21]: - 1.0000229976847993e+06 x[4,21]
   - start_service[4] + start_service[21] >= -999999
 NodeArrivalVehicles[4,22]: - 1.0000305908017206e+06 x[4,22]
   - start_service[4] + start_service[22] >= -999999
 NodeArrivalVehicles[4,23]: - 1.0000371105987907e+06 x[4,23]
   - start_service[4] + start_service[23] >= -999999
 NodeArrivalVehicles[4,24]: - 1.0000371105987907e+06 x[4,24]
   - start_service[4] + start_service[24] >= -999999
 NodeArrivalVehicles[4,25]: - 1.0000371105987907e+06 x[4,25]
   - start_service[4] + start_service[25] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000331317466504e+06 x[5,1]
   + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000331317466504e+06 x[5,2]
   + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.000028055364666e+06 x[5,3]
   + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.0000188499797942e+06 x[5,4]
   + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999.01 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000096898391704e+06 x[5,6]
   - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000402331808718e+06 x[5,7]
   - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.000028055364666e+06 x[5,8]
   - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.0000188499797942e+06 x[5,9]
   - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 999999.01 x[5,10] - start_service[5]
   + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000096898391704e+06 x[5,11]
   - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 1.0000402331808718e+06 x[5,12]
   - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 1.000028055364666e+06 x[5,13]
   - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[5,14]: - 1.0000188499797942e+06 x[5,14]
   - start_service[5] + start_service[14] >= -999999
 NodeArrivalVehicles[5,15]: - 999999.01 x[5,15] - start_service[5]
   + start_service[15] >= -999999
 NodeArrivalVehicles[5,16]: - 1.0000096898391704e+06 x[5,16]
   - start_service[5] + start_service[16] >= -999999
 NodeArrivalVehicles[5,17]: - 1.0000402331808718e+06 x[5,17]
   - start_service[5] + start_service[17] >= -999999
 NodeArrivalVehicles[5,18]: - 1.000028055364666e+06 x[5,18]
   - start_service[5] + start_service[18] >= -999999
 NodeArrivalVehicles[5,19]: - 1.0000188499797942e+06 x[5,19]
   - start_service[5] + start_service[19] >= -999999
 NodeArrivalVehicles[5,20]: - 999999.01 x[5,20] - start_service[5]
   + start_service[20] >= -999999
 NodeArrivalVehicles[5,21]: - 1.0000096898391704e+06 x[5,21]
   - start_service[5] + start_service[21] >= -999999
 NodeArrivalVehicles[5,22]: - 1.0000402331808718e+06 x[5,22]
   - start_service[5] + start_service[22] >= -999999
 NodeArrivalVehicles[5,23]: - 1.0000331317466504e+06 x[5,23]
   - start_service[5] + start_service[23] >= -999999
 NodeArrivalVehicles[5,24]: - 1.0000331317466504e+06 x[5,24]
   - start_service[5] + start_service[24] >= -999999
 NodeArrivalVehicles[5,25]: - 1.0000331317466504e+06 x[5,25]
   - start_service[5] + start_service[25] >= -999999
 NodeArrivalVehicles[6,1]: - 1.000022879122035e+06 x[6,1]
   + start_service[1] - start_service[6] >= -999999
 NodeArrivalVehicles[6,2]: - 1.000022879122035e+06 x[6,2]
   + start_service[2] - start_service[6] >= -999999
 NodeArrivalVehicles[6,3]: - 1.0000341454712306e+06 x[6,3]
   + start_service[3] - start_service[6] >= -999999
 NodeArrivalVehicles[6,4]: - 1.0000229976847993e+06 x[6,4]
   + start_service[4] - start_service[6] >= -999999
 NodeArrivalVehicles[6,5]: - 1.0000096898391704e+06 x[6,5]
   + start_service[5] - start_service[6] >= -999999
 NodeArrivalVehicles[6,6]: - 999999.01 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000345266163895e+06 x[6,7]
   - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.0000341454712306e+06 x[6,8]
   - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000229976847993e+06 x[6,9]
   - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000096898391704e+06 x[6,10]
   - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 999999.01 x[6,11] - start_service[6]
   + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 1.0000345266163895e+06 x[6,12]
   - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 1.0000341454712306e+06 x[6,13]
   - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[6,14]: - 1.0000229976847993e+06 x[6,14]
   - start_service[6] + start_service[14] >= -999999
 NodeArrivalVehicles[6,15]: - 1.0000096898391704e+06 x[6,15]
   - start_service[6] + start_service[15] >= -999999
 NodeArrivalVehicles[6,16]: - 999999.01 x[6,16] - start_service[6]
   + start_service[16] >= -999999
 NodeArrivalVehicles[6,17]: - 1.0000345266163895e+06 x[6,17]
   - start_service[6] + start_service[17] >= -999999
 NodeArrivalVehicles[6,18]: - 1.0000341454712306e+06 x[6,18]
   - start_service[6] + start_service[18] >= -999999
 NodeArrivalVehicles[6,19]: - 1.0000229976847993e+06 x[6,19]
   - start_service[6] + start_service[19] >= -999999
 NodeArrivalVehicles[6,20]: - 1.0000096898391704e+06 x[6,20]
   - start_service[6] + start_service[20] >= -999999
 NodeArrivalVehicles[6,21]: - 999999.01 x[6,21] - start_service[6]
   + start_service[21] >= -999999
 NodeArrivalVehicles[6,22]: - 1.0000345266163895e+06 x[6,22]
   - start_service[6] + start_service[22] >= -999999
 NodeArrivalVehicles[6,23]: - 1.000022879122035e+06 x[6,23]
   - start_service[6] + start_service[23] >= -999999
 NodeArrivalVehicles[6,24]: - 1.000022879122035e+06 x[6,24]
   - start_service[6] + start_service[24] >= -999999
 NodeArrivalVehicles[6,25]: - 1.000022879122035e+06 x[6,25]
   - start_service[6] + start_service[25] >= -999999
 NodeArrivalVehicles[7,1]: - 1.0000241592465249e+06 x[7,1]
   + start_service[1] - start_service[7] >= -999999
 NodeArrivalVehicles[7,2]: - 1.0000241592465249e+06 x[7,2]
   + start_service[2] - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 1.0000688935928658e+06 x[7,3]
   + start_service[3] - start_service[7] >= -999999
 NodeArrivalVehicles[7,4]: - 1.0000305908017206e+06 x[7,4]
   + start_service[4] - start_service[7] >= -999999
 NodeArrivalVehicles[7,5]: - 1.0000402331808718e+06 x[7,5]
   + start_service[5] - start_service[7] >= -999999
 NodeArrivalVehicles[7,6]: - 1.0000345266163895e+06 x[7,6]
   + start_service[6] - start_service[7] >= -999999
 NodeArrivalVehicles[7,7]: - 999999.01 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.0000688935928658e+06 x[7,8]
   - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 1.0000305908017206e+06 x[7,9]
   - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.0000402331808718e+06 x[7,10]
   - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000345266163895e+06 x[7,11]
   - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 999999.01 x[7,12] - start_service[7]
   + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 1.0000688935928658e+06 x[7,13]
   - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[7,14]: - 1.0000305908017206e+06 x[7,14]
   - start_service[7] + start_service[14] >= -999999
 NodeArrivalVehicles[7,15]: - 1.0000402331808718e+06 x[7,15]
   - start_service[7] + start_service[15] >= -999999
 NodeArrivalVehicles[7,16]: - 1.0000345266163895e+06 x[7,16]
   - start_service[7] + start_service[16] >= -999999
 NodeArrivalVehicles[7,17]: - 999999.01 x[7,17] - start_service[7]
   + start_service[17] >= -999999
 NodeArrivalVehicles[7,18]: - 1.0000688935928658e+06 x[7,18]
   - start_service[7] + start_service[18] >= -999999
 NodeArrivalVehicles[7,19]: - 1.0000305908017206e+06 x[7,19]
   - start_service[7] + start_service[19] >= -999999
 NodeArrivalVehicles[7,20]: - 1.0000402331808718e+06 x[7,20]
   - start_service[7] + start_service[20] >= -999999
 NodeArrivalVehicles[7,21]: - 1.0000345266163895e+06 x[7,21]
   - start_service[7] + start_service[21] >= -999999
 NodeArrivalVehicles[7,22]: - 999999.01 x[7,22] - start_service[7]
   + start_service[22] >= -999999
 NodeArrivalVehicles[7,23]: - 1.0000241592465249e+06 x[7,23]
   - start_service[7] + start_service[23] >= -999999
 NodeArrivalVehicles[7,24]: - 1.0000241592465249e+06 x[7,24]
   - start_service[7] + start_service[24] >= -999999
 NodeArrivalVehicles[7,25]: - 1.0000241592465249e+06 x[7,25]
   - start_service[7] + start_service[25] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000563369351183e+06 x[8,1]
   + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.0000563369351183e+06 x[8,2]
   + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 999999.01 x[8,3] + start_service[3]
   - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0000456469522376e+06 x[8,4]
   + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.000028055364666e+06 x[8,5]
   + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.0000341454712306e+06 x[8,6]
   + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.0000688935928658e+06 x[8,7]
   + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999.01 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0000456469522376e+06 x[8,9]
   - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.000028055364666e+06 x[8,10]
   - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.0000341454712306e+06 x[8,11]
   - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 1.0000688935928658e+06 x[8,12]
   - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 999999.01 x[8,13] - start_service[8]
   + start_service[13] >= -999999
 NodeArrivalVehicles[8,14]: - 1.0000456469522376e+06 x[8,14]
   - start_service[8] + start_service[14] >= -999999
 NodeArrivalVehicles[8,15]: - 1.000028055364666e+06 x[8,15]
   - start_service[8] + start_service[15] >= -999999
 NodeArrivalVehicles[8,16]: - 1.0000341454712306e+06 x[8,16]
   - start_service[8] + start_service[16] >= -999999
 NodeArrivalVehicles[8,17]: - 1.0000688935928658e+06 x[8,17]
   - start_service[8] + start_service[17] >= -999999
 NodeArrivalVehicles[8,18]: - 999999.01 x[8,18] - start_service[8]
   + start_service[18] >= -999999
 NodeArrivalVehicles[8,19]: - 1.0000456469522376e+06 x[8,19]
   - start_service[8] + start_service[19] >= -999999
 NodeArrivalVehicles[8,20]: - 1.000028055364666e+06 x[8,20]
   - start_service[8] + start_service[20] >= -999999
 NodeArrivalVehicles[8,21]: - 1.0000341454712306e+06 x[8,21]
   - start_service[8] + start_service[21] >= -999999
 NodeArrivalVehicles[8,22]: - 1.0000688935928658e+06 x[8,22]
   - start_service[8] + start_service[22] >= -999999
 NodeArrivalVehicles[8,23]: - 1.0000563369351183e+06 x[8,23]
   - start_service[8] + start_service[23] >= -999999
 NodeArrivalVehicles[8,24]: - 1.0000563369351183e+06 x[8,24]
   - start_service[8] + start_service[24] >= -999999
 NodeArrivalVehicles[8,25]: - 1.0000563369351183e+06 x[8,25]
   - start_service[8] + start_service[25] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0000371105987907e+06 x[9,1]
   + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000371105987907e+06 x[9,2]
   + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0000456469522376e+06 x[9,3]
   + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 999999.01 x[9,4] + start_service[4]
   - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.0000188499797942e+06 x[9,5]
   + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000229976847993e+06 x[9,6]
   + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 1.0000305908017206e+06 x[9,7]
   + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0000456469522376e+06 x[9,8]
   + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999.01 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.0000188499797942e+06 x[9,10]
   - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000229976847993e+06 x[9,11]
   - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 1.0000305908017206e+06 x[9,12]
   - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 1.0000456469522376e+06 x[9,13]
   - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[9,14]: - 999999.01 x[9,14] - start_service[9]
   + start_service[14] >= -999999
 NodeArrivalVehicles[9,15]: - 1.0000188499797942e+06 x[9,15]
   - start_service[9] + start_service[15] >= -999999
 NodeArrivalVehicles[9,16]: - 1.0000229976847993e+06 x[9,16]
   - start_service[9] + start_service[16] >= -999999
 NodeArrivalVehicles[9,17]: - 1.0000305908017206e+06 x[9,17]
   - start_service[9] + start_service[17] >= -999999
 NodeArrivalVehicles[9,18]: - 1.0000456469522376e+06 x[9,18]
   - start_service[9] + start_service[18] >= -999999
 NodeArrivalVehicles[9,19]: - 999999.01 x[9,19] - start_service[9]
   + start_service[19] >= -999999
 NodeArrivalVehicles[9,20]: - 1.0000188499797942e+06 x[9,20]
   - start_service[9] + start_service[20] >= -999999
 NodeArrivalVehicles[9,21]: - 1.0000229976847993e+06 x[9,21]
   - start_service[9] + start_service[21] >= -999999
 NodeArrivalVehicles[9,22]: - 1.0000305908017206e+06 x[9,22]
   - start_service[9] + start_service[22] >= -999999
 NodeArrivalVehicles[9,23]: - 1.0000371105987907e+06 x[9,23]
   - start_service[9] + start_service[23] >= -999999
 NodeArrivalVehicles[9,24]: - 1.0000371105987907e+06 x[9,24]
   - start_service[9] + start_service[24] >= -999999
 NodeArrivalVehicles[9,25]: - 1.0000371105987907e+06 x[9,25]
   - start_service[9] + start_service[25] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000331317466504e+06 x[10,1]
   + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000331317466504e+06 x[10,2]
   + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.000028055364666e+06 x[10,3]
   + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.0000188499797942e+06 x[10,4]
   + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 999999.01 x[10,5] + start_service[5]
   - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000096898391704e+06 x[10,6]
   + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.0000402331808718e+06 x[10,7]
   + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.000028055364666e+06 x[10,8]
   + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.0000188499797942e+06 x[10,9]
   + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999.01 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000096898391704e+06 x[10,11]
   - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 1.0000402331808718e+06 x[10,12]
   - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 1.000028055364666e+06 x[10,13]
   - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[10,14]: - 1.0000188499797942e+06 x[10,14]
   - start_service[10] + start_service[14] >= -999999
 NodeArrivalVehicles[10,15]: - 999999.01 x[10,15] - start_service[10]
   + start_service[15] >= -999999
 NodeArrivalVehicles[10,16]: - 1.0000096898391704e+06 x[10,16]
   - start_service[10] + start_service[16] >= -999999
 NodeArrivalVehicles[10,17]: - 1.0000402331808718e+06 x[10,17]
   - start_service[10] + start_service[17] >= -999999
 NodeArrivalVehicles[10,18]: - 1.000028055364666e+06 x[10,18]
   - start_service[10] + start_service[18] >= -999999
 NodeArrivalVehicles[10,19]: - 1.0000188499797942e+06 x[10,19]
   - start_service[10] + start_service[19] >= -999999
 NodeArrivalVehicles[10,20]: - 999999.01 x[10,20] - start_service[10]
   + start_service[20] >= -999999
 NodeArrivalVehicles[10,21]: - 1.0000096898391704e+06 x[10,21]
   - start_service[10] + start_service[21] >= -999999
 NodeArrivalVehicles[10,22]: - 1.0000402331808718e+06 x[10,22]
   - start_service[10] + start_service[22] >= -999999
 NodeArrivalVehicles[10,23]: - 1.0000331317466504e+06 x[10,23]
   - start_service[10] + start_service[23] >= -999999
 NodeArrivalVehicles[10,24]: - 1.0000331317466504e+06 x[10,24]
   - start_service[10] + start_service[24] >= -999999
 NodeArrivalVehicles[10,25]: - 1.0000331317466504e+06 x[10,25]
   - start_service[10] + start_service[25] >= -999999
 NodeArrivalVehicles[11,1]: - 1.000022879122035e+06 x[11,1]
   + start_service[1] - start_service[11] >= -999999
 NodeArrivalVehicles[11,2]: - 1.000022879122035e+06 x[11,2]
   + start_service[2] - start_service[11] >= -999999
 NodeArrivalVehicles[11,3]: - 1.0000341454712306e+06 x[11,3]
   + start_service[3] - start_service[11] >= -999999
 NodeArrivalVehicles[11,4]: - 1.0000229976847993e+06 x[11,4]
   + start_service[4] - start_service[11] >= -999999
 NodeArrivalVehicles[11,5]: - 1.0000096898391704e+06 x[11,5]
   + start_service[5] - start_service[11] >= -999999
 NodeArrivalVehicles[11,6]: - 999999.01 x[11,6] + start_service[6]
   - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000345266163895e+06 x[11,7]
   + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.0000341454712306e+06 x[11,8]
   + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000229976847993e+06 x[11,9]
   + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000096898391704e+06 x[11,10]
   + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999.01 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 1.0000345266163895e+06 x[11,12]
   - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 1.0000341454712306e+06 x[11,13]
   - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[11,14]: - 1.0000229976847993e+06 x[11,14]
   - start_service[11] + start_service[14] >= -999999
 NodeArrivalVehicles[11,15]: - 1.0000096898391704e+06 x[11,15]
   - start_service[11] + start_service[15] >= -999999
 NodeArrivalVehicles[11,16]: - 999999.01 x[11,16] - start_service[11]
   + start_service[16] >= -999999
 NodeArrivalVehicles[11,17]: - 1.0000345266163895e+06 x[11,17]
   - start_service[11] + start_service[17] >= -999999
 NodeArrivalVehicles[11,18]: - 1.0000341454712306e+06 x[11,18]
   - start_service[11] + start_service[18] >= -999999
 NodeArrivalVehicles[11,19]: - 1.0000229976847993e+06 x[11,19]
   - start_service[11] + start_service[19] >= -999999
 NodeArrivalVehicles[11,20]: - 1.0000096898391704e+06 x[11,20]
   - start_service[11] + start_service[20] >= -999999
 NodeArrivalVehicles[11,21]: - 999999.01 x[11,21] - start_service[11]
   + start_service[21] >= -999999
 NodeArrivalVehicles[11,22]: - 1.0000345266163895e+06 x[11,22]
   - start_service[11] + start_service[22] >= -999999
 NodeArrivalVehicles[11,23]: - 1.000022879122035e+06 x[11,23]
   - start_service[11] + start_service[23] >= -999999
 NodeArrivalVehicles[11,24]: - 1.000022879122035e+06 x[11,24]
   - start_service[11] + start_service[24] >= -999999
 NodeArrivalVehicles[11,25]: - 1.000022879122035e+06 x[11,25]
   - start_service[11] + start_service[25] >= -999999
 NodeArrivalVehicles[12,1]: - 1.0000241592465249e+06 x[12,1]
   + start_service[1] - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 1.0000241592465249e+06 x[12,2]
   + start_service[2] - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 1.0000688935928658e+06 x[12,3]
   + start_service[3] - start_service[12] >= -999999
 NodeArrivalVehicles[12,4]: - 1.0000305908017206e+06 x[12,4]
   + start_service[4] - start_service[12] >= -999999
 NodeArrivalVehicles[12,5]: - 1.0000402331808718e+06 x[12,5]
   + start_service[5] - start_service[12] >= -999999
 NodeArrivalVehicles[12,6]: - 1.0000345266163895e+06 x[12,6]
   + start_service[6] - start_service[12] >= -999999
 NodeArrivalVehicles[12,7]: - 999999.01 x[12,7] + start_service[7]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000688935928658e+06 x[12,8]
   + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000305908017206e+06 x[12,9]
   + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000402331808718e+06 x[12,10]
   + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.0000345266163895e+06 x[12,11]
   + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999.01 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 1.0000688935928658e+06 x[12,13]
   - start_service[12] + start_service[13] >= -999999
 NodeArrivalVehicles[12,14]: - 1.0000305908017206e+06 x[12,14]
   - start_service[12] + start_service[14] >= -999999
 NodeArrivalVehicles[12,15]: - 1.0000402331808718e+06 x[12,15]
   - start_service[12] + start_service[15] >= -999999
 NodeArrivalVehicles[12,16]: - 1.0000345266163895e+06 x[12,16]
   - start_service[12] + start_service[16] >= -999999
 NodeArrivalVehicles[12,17]: - 999999.01 x[12,17] - start_service[12]
   + start_service[17] >= -999999
 NodeArrivalVehicles[12,18]: - 1.0000688935928658e+06 x[12,18]
   - start_service[12] + start_service[18] >= -999999
 NodeArrivalVehicles[12,19]: - 1.0000305908017206e+06 x[12,19]
   - start_service[12] + start_service[19] >= -999999
 NodeArrivalVehicles[12,20]: - 1.0000402331808718e+06 x[12,20]
   - start_service[12] + start_service[20] >= -999999
 NodeArrivalVehicles[12,21]: - 1.0000345266163895e+06 x[12,21]
   - start_service[12] + start_service[21] >= -999999
 NodeArrivalVehicles[12,22]: - 999999.01 x[12,22] - start_service[12]
   + start_service[22] >= -999999
 NodeArrivalVehicles[12,23]: - 1.0000241592465249e+06 x[12,23]
   - start_service[12] + start_service[23] >= -999999
 NodeArrivalVehicles[12,24]: - 1.0000241592465249e+06 x[12,24]
   - start_service[12] + start_service[24] >= -999999
 NodeArrivalVehicles[12,25]: - 1.0000241592465249e+06 x[12,25]
   - start_service[12] + start_service[25] >= -999999
 NodeArrivalVehicles[13,1]: - 1.0000563369351183e+06 x[13,1]
   + start_service[1] - start_service[13] >= -999999
 NodeArrivalVehicles[13,2]: - 1.0000563369351183e+06 x[13,2]
   + start_service[2] - start_service[13] >= -999999
 NodeArrivalVehicles[13,3]: - 999999.01 x[13,3] + start_service[3]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 1.0000456469522376e+06 x[13,4]
   + start_service[4] - start_service[13] >= -999999
 NodeArrivalVehicles[13,5]: - 1.000028055364666e+06 x[13,5]
   + start_service[5] - start_service[13] >= -999999
 NodeArrivalVehicles[13,6]: - 1.0000341454712306e+06 x[13,6]
   + start_service[6] - start_service[13] >= -999999
 NodeArrivalVehicles[13,7]: - 1.0000688935928658e+06 x[13,7]
   + start_service[7] - start_service[13] >= -999999
 NodeArrivalVehicles[13,8]: - 999999.01 x[13,8] + start_service[8]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,9]: - 1.0000456469522376e+06 x[13,9]
   + start_service[9] - start_service[13] >= -999999
 NodeArrivalVehicles[13,10]: - 1.000028055364666e+06 x[13,10]
   + start_service[10] - start_service[13] >= -999999
 NodeArrivalVehicles[13,11]: - 1.0000341454712306e+06 x[13,11]
   + start_service[11] - start_service[13] >= -999999
 NodeArrivalVehicles[13,12]: - 1.0000688935928658e+06 x[13,12]
   + start_service[12] - start_service[13] >= -999999
 NodeArrivalVehicles[13,13]: - 999999.01 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 1.0000456469522376e+06 x[13,14]
   - start_service[13] + start_service[14] >= -999999
 NodeArrivalVehicles[13,15]: - 1.000028055364666e+06 x[13,15]
   - start_service[13] + start_service[15] >= -999999
 NodeArrivalVehicles[13,16]: - 1.0000341454712306e+06 x[13,16]
   - start_service[13] + start_service[16] >= -999999
 NodeArrivalVehicles[13,17]: - 1.0000688935928658e+06 x[13,17]
   - start_service[13] + start_service[17] >= -999999
 NodeArrivalVehicles[13,18]: - 999999.01 x[13,18] - start_service[13]
   + start_service[18] >= -999999
 NodeArrivalVehicles[13,19]: - 1.0000456469522376e+06 x[13,19]
   - start_service[13] + start_service[19] >= -999999
 NodeArrivalVehicles[13,20]: - 1.000028055364666e+06 x[13,20]
   - start_service[13] + start_service[20] >= -999999
 NodeArrivalVehicles[13,21]: - 1.0000341454712306e+06 x[13,21]
   - start_service[13] + start_service[21] >= -999999
 NodeArrivalVehicles[13,22]: - 1.0000688935928658e+06 x[13,22]
   - start_service[13] + start_service[22] >= -999999
 NodeArrivalVehicles[13,23]: - 1.0000563369351183e+06 x[13,23]
   - start_service[13] + start_service[23] >= -999999
 NodeArrivalVehicles[13,24]: - 1.0000563369351183e+06 x[13,24]
   - start_service[13] + start_service[24] >= -999999
 NodeArrivalVehicles[13,25]: - 1.0000563369351183e+06 x[13,25]
   - start_service[13] + start_service[25] >= -999999
 NodeArrivalVehicles[14,1]: - 1.0000371105987907e+06 x[14,1]
   + start_service[1] - start_service[14] >= -999999
 NodeArrivalVehicles[14,2]: - 1.0000371105987907e+06 x[14,2]
   + start_service[2] - start_service[14] >= -999999
 NodeArrivalVehicles[14,3]: - 1.0000456469522376e+06 x[14,3]
   + start_service[3] - start_service[14] >= -999999
 NodeArrivalVehicles[14,4]: - 999999.01 x[14,4] + start_service[4]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 1.0000188499797942e+06 x[14,5]
   + start_service[5] - start_service[14] >= -999999
 NodeArrivalVehicles[14,6]: - 1.0000229976847993e+06 x[14,6]
   + start_service[6] - start_service[14] >= -999999
 NodeArrivalVehicles[14,7]: - 1.0000305908017206e+06 x[14,7]
   + start_service[7] - start_service[14] >= -999999
 NodeArrivalVehicles[14,8]: - 1.0000456469522376e+06 x[14,8]
   + start_service[8] - start_service[14] >= -999999
 NodeArrivalVehicles[14,9]: - 999999.01 x[14,9] + start_service[9]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,10]: - 1.0000188499797942e+06 x[14,10]
   + start_service[10] - start_service[14] >= -999999
 NodeArrivalVehicles[14,11]: - 1.0000229976847993e+06 x[14,11]
   + start_service[11] - start_service[14] >= -999999
 NodeArrivalVehicles[14,12]: - 1.0000305908017206e+06 x[14,12]
   + start_service[12] - start_service[14] >= -999999
 NodeArrivalVehicles[14,13]: - 1.0000456469522376e+06 x[14,13]
   + start_service[13] - start_service[14] >= -999999
 NodeArrivalVehicles[14,14]: - 999999.01 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 1.0000188499797942e+06 x[14,15]
   - start_service[14] + start_service[15] >= -999999
 NodeArrivalVehicles[14,16]: - 1.0000229976847993e+06 x[14,16]
   - start_service[14] + start_service[16] >= -999999
 NodeArrivalVehicles[14,17]: - 1.0000305908017206e+06 x[14,17]
   - start_service[14] + start_service[17] >= -999999
 NodeArrivalVehicles[14,18]: - 1.0000456469522376e+06 x[14,18]
   - start_service[14] + start_service[18] >= -999999
 NodeArrivalVehicles[14,19]: - 999999.01 x[14,19] - start_service[14]
   + start_service[19] >= -999999
 NodeArrivalVehicles[14,20]: - 1.0000188499797942e+06 x[14,20]
   - start_service[14] + start_service[20] >= -999999
 NodeArrivalVehicles[14,21]: - 1.0000229976847993e+06 x[14,21]
   - start_service[14] + start_service[21] >= -999999
 NodeArrivalVehicles[14,22]: - 1.0000305908017206e+06 x[14,22]
   - start_service[14] + start_service[22] >= -999999
 NodeArrivalVehicles[14,23]: - 1.0000371105987907e+06 x[14,23]
   - start_service[14] + start_service[23] >= -999999
 NodeArrivalVehicles[14,24]: - 1.0000371105987907e+06 x[14,24]
   - start_service[14] + start_service[24] >= -999999
 NodeArrivalVehicles[14,25]: - 1.0000371105987907e+06 x[14,25]
   - start_service[14] + start_service[25] >= -999999
 NodeArrivalVehicles[15,1]: - 1.0000331317466504e+06 x[15,1]
   + start_service[1] - start_service[15] >= -999999
 NodeArrivalVehicles[15,2]: - 1.0000331317466504e+06 x[15,2]
   + start_service[2] - start_service[15] >= -999999
 NodeArrivalVehicles[15,3]: - 1.000028055364666e+06 x[15,3]
   + start_service[3] - start_service[15] >= -999999
 NodeArrivalVehicles[15,4]: - 1.0000188499797942e+06 x[15,4]
   + start_service[4] - start_service[15] >= -999999
 NodeArrivalVehicles[15,5]: - 999999.01 x[15,5] + start_service[5]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,6]: - 1.0000096898391704e+06 x[15,6]
   + start_service[6] - start_service[15] >= -999999
 NodeArrivalVehicles[15,7]: - 1.0000402331808718e+06 x[15,7]
   + start_service[7] - start_service[15] >= -999999
 NodeArrivalVehicles[15,8]: - 1.000028055364666e+06 x[15,8]
   + start_service[8] - start_service[15] >= -999999
 NodeArrivalVehicles[15,9]: - 1.0000188499797942e+06 x[15,9]
   + start_service[9] - start_service[15] >= -999999
 NodeArrivalVehicles[15,10]: - 999999.01 x[15,10] + start_service[10]
   - start_service[15] >= -999999
 NodeArrivalVehicles[15,11]: - 1.0000096898391704e+06 x[15,11]
   + start_service[11] - start_service[15] >= -999999
 NodeArrivalVehicles[15,12]: - 1.0000402331808718e+06 x[15,12]
   + start_service[12] - start_service[15] >= -999999
 NodeArrivalVehicles[15,13]: - 1.000028055364666e+06 x[15,13]
   + start_service[13] - start_service[15] >= -999999
 NodeArrivalVehicles[15,14]: - 1.0000188499797942e+06 x[15,14]
   + start_service[14] - start_service[15] >= -999999
 NodeArrivalVehicles[15,15]: - 999999.01 x[15,15] >= -999999
 NodeArrivalVehicles[15,16]: - 1.0000096898391704e+06 x[15,16]
   - start_service[15] + start_service[16] >= -999999
 NodeArrivalVehicles[15,17]: - 1.0000402331808718e+06 x[15,17]
   - start_service[15] + start_service[17] >= -999999
 NodeArrivalVehicles[15,18]: - 1.000028055364666e+06 x[15,18]
   - start_service[15] + start_service[18] >= -999999
 NodeArrivalVehicles[15,19]: - 1.0000188499797942e+06 x[15,19]
   - start_service[15] + start_service[19] >= -999999
 NodeArrivalVehicles[15,20]: - 999999.01 x[15,20] - start_service[15]
   + start_service[20] >= -999999
 NodeArrivalVehicles[15,21]: - 1.0000096898391704e+06 x[15,21]
   - start_service[15] + start_service[21] >= -999999
 NodeArrivalVehicles[15,22]: - 1.0000402331808718e+06 x[15,22]
   - start_service[15] + start_service[22] >= -999999
 NodeArrivalVehicles[15,23]: - 1.0000331317466504e+06 x[15,23]
   - start_service[15] + start_service[23] >= -999999
 NodeArrivalVehicles[15,24]: - 1.0000331317466504e+06 x[15,24]
   - start_service[15] + start_service[24] >= -999999
 NodeArrivalVehicles[15,25]: - 1.0000331317466504e+06 x[15,25]
   - start_service[15] + start_service[25] >= -999999
 NodeArrivalVehicles[16,1]: - 1.000022879122035e+06 x[16,1]
   + start_service[1] - start_service[16] >= -999999
 NodeArrivalVehicles[16,2]: - 1.000022879122035e+06 x[16,2]
   + start_service[2] - start_service[16] >= -999999
 NodeArrivalVehicles[16,3]: - 1.0000341454712306e+06 x[16,3]
   + start_service[3] - start_service[16] >= -999999
 NodeArrivalVehicles[16,4]: - 1.0000229976847993e+06 x[16,4]
   + start_service[4] - start_service[16] >= -999999
 NodeArrivalVehicles[16,5]: - 1.0000096898391704e+06 x[16,5]
   + start_service[5] - start_service[16] >= -999999
 NodeArrivalVehicles[16,6]: - 999999.01 x[16,6] + start_service[6]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,7]: - 1.0000345266163895e+06 x[16,7]
   + start_service[7] - start_service[16] >= -999999
 NodeArrivalVehicles[16,8]: - 1.0000341454712306e+06 x[16,8]
   + start_service[8] - start_service[16] >= -999999
 NodeArrivalVehicles[16,9]: - 1.0000229976847993e+06 x[16,9]
   + start_service[9] - start_service[16] >= -999999
 NodeArrivalVehicles[16,10]: - 1.0000096898391704e+06 x[16,10]
   + start_service[10] - start_service[16] >= -999999
 NodeArrivalVehicles[16,11]: - 999999.01 x[16,11] + start_service[11]
   - start_service[16] >= -999999
 NodeArrivalVehicles[16,12]: - 1.0000345266163895e+06 x[16,12]
   + start_service[12] - start_service[16] >= -999999
 NodeArrivalVehicles[16,13]: - 1.0000341454712306e+06 x[16,13]
   + start_service[13] - start_service[16] >= -999999
 NodeArrivalVehicles[16,14]: - 1.0000229976847993e+06 x[16,14]
   + start_service[14] - start_service[16] >= -999999
 NodeArrivalVehicles[16,15]: - 1.0000096898391704e+06 x[16,15]
   + start_service[15] - start_service[16] >= -999999
 NodeArrivalVehicles[16,16]: - 999999.01 x[16,16] >= -999999
 NodeArrivalVehicles[16,17]: - 1.0000345266163895e+06 x[16,17]
   - start_service[16] + start_service[17] >= -999999
 NodeArrivalVehicles[16,18]: - 1.0000341454712306e+06 x[16,18]
   - start_service[16] + start_service[18] >= -999999
 NodeArrivalVehicles[16,19]: - 1.0000229976847993e+06 x[16,19]
   - start_service[16] + start_service[19] >= -999999
 NodeArrivalVehicles[16,20]: - 1.0000096898391704e+06 x[16,20]
   - start_service[16] + start_service[20] >= -999999
 NodeArrivalVehicles[16,21]: - 999999.01 x[16,21] - start_service[16]
   + start_service[21] >= -999999
 NodeArrivalVehicles[16,22]: - 1.0000345266163895e+06 x[16,22]
   - start_service[16] + start_service[22] >= -999999
 NodeArrivalVehicles[16,23]: - 1.000022879122035e+06 x[16,23]
   - start_service[16] + start_service[23] >= -999999
 NodeArrivalVehicles[16,24]: - 1.000022879122035e+06 x[16,24]
   - start_service[16] + start_service[24] >= -999999
 NodeArrivalVehicles[16,25]: - 1.000022879122035e+06 x[16,25]
   - start_service[16] + start_service[25] >= -999999
 NodeArrivalVehicles[17,1]: - 1.0000241592465249e+06 x[17,1]
   + start_service[1] - start_service[17] >= -999999
 NodeArrivalVehicles[17,2]: - 1.0000241592465249e+06 x[17,2]
   + start_service[2] - start_service[17] >= -999999
 NodeArrivalVehicles[17,3]: - 1.0000688935928658e+06 x[17,3]
   + start_service[3] - start_service[17] >= -999999
 NodeArrivalVehicles[17,4]: - 1.0000305908017206e+06 x[17,4]
   + start_service[4] - start_service[17] >= -999999
 NodeArrivalVehicles[17,5]: - 1.0000402331808718e+06 x[17,5]
   + start_service[5] - start_service[17] >= -999999
 NodeArrivalVehicles[17,6]: - 1.0000345266163895e+06 x[17,6]
   + start_service[6] - start_service[17] >= -999999
 NodeArrivalVehicles[17,7]: - 999999.01 x[17,7] + start_service[7]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,8]: - 1.0000688935928658e+06 x[17,8]
   + start_service[8] - start_service[17] >= -999999
 NodeArrivalVehicles[17,9]: - 1.0000305908017206e+06 x[17,9]
   + start_service[9] - start_service[17] >= -999999
 NodeArrivalVehicles[17,10]: - 1.0000402331808718e+06 x[17,10]
   + start_service[10] - start_service[17] >= -999999
 NodeArrivalVehicles[17,11]: - 1.0000345266163895e+06 x[17,11]
   + start_service[11] - start_service[17] >= -999999
 NodeArrivalVehicles[17,12]: - 999999.01 x[17,12] + start_service[12]
   - start_service[17] >= -999999
 NodeArrivalVehicles[17,13]: - 1.0000688935928658e+06 x[17,13]
   + start_service[13] - start_service[17] >= -999999
 NodeArrivalVehicles[17,14]: - 1.0000305908017206e+06 x[17,14]
   + start_service[14] - start_service[17] >= -999999
 NodeArrivalVehicles[17,15]: - 1.0000402331808718e+06 x[17,15]
   + start_service[15] - start_service[17] >= -999999
 NodeArrivalVehicles[17,16]: - 1.0000345266163895e+06 x[17,16]
   + start_service[16] - start_service[17] >= -999999
 NodeArrivalVehicles[17,17]: - 999999.01 x[17,17] >= -999999
 NodeArrivalVehicles[17,18]: - 1.0000688935928658e+06 x[17,18]
   - start_service[17] + start_service[18] >= -999999
 NodeArrivalVehicles[17,19]: - 1.0000305908017206e+06 x[17,19]
   - start_service[17] + start_service[19] >= -999999
 NodeArrivalVehicles[17,20]: - 1.0000402331808718e+06 x[17,20]
   - start_service[17] + start_service[20] >= -999999
 NodeArrivalVehicles[17,21]: - 1.0000345266163895e+06 x[17,21]
   - start_service[17] + start_service[21] >= -999999
 NodeArrivalVehicles[17,22]: - 999999.01 x[17,22] - start_service[17]
   + start_service[22] >= -999999
 NodeArrivalVehicles[17,23]: - 1.0000241592465249e+06 x[17,23]
   - start_service[17] + start_service[23] >= -999999
 NodeArrivalVehicles[17,24]: - 1.0000241592465249e+06 x[17,24]
   - start_service[17] + start_service[24] >= -999999
 NodeArrivalVehicles[17,25]: - 1.0000241592465249e+06 x[17,25]
   - start_service[17] + start_service[25] >= -999999
 NodeArrivalVehicles[18,1]: - 1.0000563369351183e+06 x[18,1]
   + start_service[1] - start_service[18] >= -999999
 NodeArrivalVehicles[18,2]: - 1.0000563369351183e+06 x[18,2]
   + start_service[2] - start_service[18] >= -999999
 NodeArrivalVehicles[18,3]: - 999999.01 x[18,3] + start_service[3]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,4]: - 1.0000456469522376e+06 x[18,4]
   + start_service[4] - start_service[18] >= -999999
 NodeArrivalVehicles[18,5]: - 1.000028055364666e+06 x[18,5]
   + start_service[5] - start_service[18] >= -999999
 NodeArrivalVehicles[18,6]: - 1.0000341454712306e+06 x[18,6]
   + start_service[6] - start_service[18] >= -999999
 NodeArrivalVehicles[18,7]: - 1.0000688935928658e+06 x[18,7]
   + start_service[7] - start_service[18] >= -999999
 NodeArrivalVehicles[18,8]: - 999999.01 x[18,8] + start_service[8]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,9]: - 1.0000456469522376e+06 x[18,9]
   + start_service[9] - start_service[18] >= -999999
 NodeArrivalVehicles[18,10]: - 1.000028055364666e+06 x[18,10]
   + start_service[10] - start_service[18] >= -999999
 NodeArrivalVehicles[18,11]: - 1.0000341454712306e+06 x[18,11]
   + start_service[11] - start_service[18] >= -999999
 NodeArrivalVehicles[18,12]: - 1.0000688935928658e+06 x[18,12]
   + start_service[12] - start_service[18] >= -999999
 NodeArrivalVehicles[18,13]: - 999999.01 x[18,13] + start_service[13]
   - start_service[18] >= -999999
 NodeArrivalVehicles[18,14]: - 1.0000456469522376e+06 x[18,14]
   + start_service[14] - start_service[18] >= -999999
 NodeArrivalVehicles[18,15]: - 1.000028055364666e+06 x[18,15]
   + start_service[15] - start_service[18] >= -999999
 NodeArrivalVehicles[18,16]: - 1.0000341454712306e+06 x[18,16]
   + start_service[16] - start_service[18] >= -999999
 NodeArrivalVehicles[18,17]: - 1.0000688935928658e+06 x[18,17]
   + start_service[17] - start_service[18] >= -999999
 NodeArrivalVehicles[18,18]: - 999999.01 x[18,18] >= -999999
 NodeArrivalVehicles[18,19]: - 1.0000456469522376e+06 x[18,19]
   - start_service[18] + start_service[19] >= -999999
 NodeArrivalVehicles[18,20]: - 1.000028055364666e+06 x[18,20]
   - start_service[18] + start_service[20] >= -999999
 NodeArrivalVehicles[18,21]: - 1.0000341454712306e+06 x[18,21]
   - start_service[18] + start_service[21] >= -999999
 NodeArrivalVehicles[18,22]: - 1.0000688935928658e+06 x[18,22]
   - start_service[18] + start_service[22] >= -999999
 NodeArrivalVehicles[18,23]: - 1.0000563369351183e+06 x[18,23]
   - start_service[18] + start_service[23] >= -999999
 NodeArrivalVehicles[18,24]: - 1.0000563369351183e+06 x[18,24]
   - start_service[18] + start_service[24] >= -999999
 NodeArrivalVehicles[18,25]: - 1.0000563369351183e+06 x[18,25]
   - start_service[18] + start_service[25] >= -999999
 NodeArrivalVehicles[19,1]: - 1.0000371105987907e+06 x[19,1]
   + start_service[1] - start_service[19] >= -999999
 NodeArrivalVehicles[19,2]: - 1.0000371105987907e+06 x[19,2]
   + start_service[2] - start_service[19] >= -999999
 NodeArrivalVehicles[19,3]: - 1.0000456469522376e+06 x[19,3]
   + start_service[3] - start_service[19] >= -999999
 NodeArrivalVehicles[19,4]: - 999999.01 x[19,4] + start_service[4]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,5]: - 1.0000188499797942e+06 x[19,5]
   + start_service[5] - start_service[19] >= -999999
 NodeArrivalVehicles[19,6]: - 1.0000229976847993e+06 x[19,6]
   + start_service[6] - start_service[19] >= -999999
 NodeArrivalVehicles[19,7]: - 1.0000305908017206e+06 x[19,7]
   + start_service[7] - start_service[19] >= -999999
 NodeArrivalVehicles[19,8]: - 1.0000456469522376e+06 x[19,8]
   + start_service[8] - start_service[19] >= -999999
 NodeArrivalVehicles[19,9]: - 999999.01 x[19,9] + start_service[9]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,10]: - 1.0000188499797942e+06 x[19,10]
   + start_service[10] - start_service[19] >= -999999
 NodeArrivalVehicles[19,11]: - 1.0000229976847993e+06 x[19,11]
   + start_service[11] - start_service[19] >= -999999
 NodeArrivalVehicles[19,12]: - 1.0000305908017206e+06 x[19,12]
   + start_service[12] - start_service[19] >= -999999
 NodeArrivalVehicles[19,13]: - 1.0000456469522376e+06 x[19,13]
   + start_service[13] - start_service[19] >= -999999
 NodeArrivalVehicles[19,14]: - 999999.01 x[19,14] + start_service[14]
   - start_service[19] >= -999999
 NodeArrivalVehicles[19,15]: - 1.0000188499797942e+06 x[19,15]
   + start_service[15] - start_service[19] >= -999999
 NodeArrivalVehicles[19,16]: - 1.0000229976847993e+06 x[19,16]
   + start_service[16] - start_service[19] >= -999999
 NodeArrivalVehicles[19,17]: - 1.0000305908017206e+06 x[19,17]
   + start_service[17] - start_service[19] >= -999999
 NodeArrivalVehicles[19,18]: - 1.0000456469522376e+06 x[19,18]
   + start_service[18] - start_service[19] >= -999999
 NodeArrivalVehicles[19,19]: - 999999.01 x[19,19] >= -999999
 NodeArrivalVehicles[19,20]: - 1.0000188499797942e+06 x[19,20]
   - start_service[19] + start_service[20] >= -999999
 NodeArrivalVehicles[19,21]: - 1.0000229976847993e+06 x[19,21]
   - start_service[19] + start_service[21] >= -999999
 NodeArrivalVehicles[19,22]: - 1.0000305908017206e+06 x[19,22]
   - start_service[19] + start_service[22] >= -999999
 NodeArrivalVehicles[19,23]: - 1.0000371105987907e+06 x[19,23]
   - start_service[19] + start_service[23] >= -999999
 NodeArrivalVehicles[19,24]: - 1.0000371105987907e+06 x[19,24]
   - start_service[19] + start_service[24] >= -999999
 NodeArrivalVehicles[19,25]: - 1.0000371105987907e+06 x[19,25]
   - start_service[19] + start_service[25] >= -999999
 NodeArrivalVehicles[20,1]: - 1.0000331317466504e+06 x[20,1]
   + start_service[1] - start_service[20] >= -999999
 NodeArrivalVehicles[20,2]: - 1.0000331317466504e+06 x[20,2]
   + start_service[2] - start_service[20] >= -999999
 NodeArrivalVehicles[20,3]: - 1.000028055364666e+06 x[20,3]
   + start_service[3] - start_service[20] >= -999999
 NodeArrivalVehicles[20,4]: - 1.0000188499797942e+06 x[20,4]
   + start_service[4] - start_service[20] >= -999999
 NodeArrivalVehicles[20,5]: - 999999.01 x[20,5] + start_service[5]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,6]: - 1.0000096898391704e+06 x[20,6]
   + start_service[6] - start_service[20] >= -999999
 NodeArrivalVehicles[20,7]: - 1.0000402331808718e+06 x[20,7]
   + start_service[7] - start_service[20] >= -999999
 NodeArrivalVehicles[20,8]: - 1.000028055364666e+06 x[20,8]
   + start_service[8] - start_service[20] >= -999999
 NodeArrivalVehicles[20,9]: - 1.0000188499797942e+06 x[20,9]
   + start_service[9] - start_service[20] >= -999999
 NodeArrivalVehicles[20,10]: - 999999.01 x[20,10] + start_service[10]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,11]: - 1.0000096898391704e+06 x[20,11]
   + start_service[11] - start_service[20] >= -999999
 NodeArrivalVehicles[20,12]: - 1.0000402331808718e+06 x[20,12]
   + start_service[12] - start_service[20] >= -999999
 NodeArrivalVehicles[20,13]: - 1.000028055364666e+06 x[20,13]
   + start_service[13] - start_service[20] >= -999999
 NodeArrivalVehicles[20,14]: - 1.0000188499797942e+06 x[20,14]
   + start_service[14] - start_service[20] >= -999999
 NodeArrivalVehicles[20,15]: - 999999.01 x[20,15] + start_service[15]
   - start_service[20] >= -999999
 NodeArrivalVehicles[20,16]: - 1.0000096898391704e+06 x[20,16]
   + start_service[16] - start_service[20] >= -999999
 NodeArrivalVehicles[20,17]: - 1.0000402331808718e+06 x[20,17]
   + start_service[17] - start_service[20] >= -999999
 NodeArrivalVehicles[20,18]: - 1.000028055364666e+06 x[20,18]
   + start_service[18] - start_service[20] >= -999999
 NodeArrivalVehicles[20,19]: - 1.0000188499797942e+06 x[20,19]
   + start_service[19] - start_service[20] >= -999999
 NodeArrivalVehicles[20,20]: - 999999.01 x[20,20] >= -999999
 NodeArrivalVehicles[20,21]: - 1.0000096898391704e+06 x[20,21]
   - start_service[20] + start_service[21] >= -999999
 NodeArrivalVehicles[20,22]: - 1.0000402331808718e+06 x[20,22]
   - start_service[20] + start_service[22] >= -999999
 NodeArrivalVehicles[20,23]: - 1.0000331317466504e+06 x[20,23]
   - start_service[20] + start_service[23] >= -999999
 NodeArrivalVehicles[20,24]: - 1.0000331317466504e+06 x[20,24]
   - start_service[20] + start_service[24] >= -999999
 NodeArrivalVehicles[20,25]: - 1.0000331317466504e+06 x[20,25]
   - start_service[20] + start_service[25] >= -999999
 NodeArrivalVehicles[21,1]: - 1.000022879122035e+06 x[21,1]
   + start_service[1] - start_service[21] >= -999999
 NodeArrivalVehicles[21,2]: - 1.000022879122035e+06 x[21,2]
   + start_service[2] - start_service[21] >= -999999
 NodeArrivalVehicles[21,3]: - 1.0000341454712306e+06 x[21,3]
   + start_service[3] - start_service[21] >= -999999
 NodeArrivalVehicles[21,4]: - 1.0000229976847993e+06 x[21,4]
   + start_service[4] - start_service[21] >= -999999
 NodeArrivalVehicles[21,5]: - 1.0000096898391704e+06 x[21,5]
   + start_service[5] - start_service[21] >= -999999
 NodeArrivalVehicles[21,6]: - 999999.01 x[21,6] + start_service[6]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,7]: - 1.0000345266163895e+06 x[21,7]
   + start_service[7] - start_service[21] >= -999999
 NodeArrivalVehicles[21,8]: - 1.0000341454712306e+06 x[21,8]
   + start_service[8] - start_service[21] >= -999999
 NodeArrivalVehicles[21,9]: - 1.0000229976847993e+06 x[21,9]
   + start_service[9] - start_service[21] >= -999999
 NodeArrivalVehicles[21,10]: - 1.0000096898391704e+06 x[21,10]
   + start_service[10] - start_service[21] >= -999999
 NodeArrivalVehicles[21,11]: - 999999.01 x[21,11] + start_service[11]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,12]: - 1.0000345266163895e+06 x[21,12]
   + start_service[12] - start_service[21] >= -999999
 NodeArrivalVehicles[21,13]: - 1.0000341454712306e+06 x[21,13]
   + start_service[13] - start_service[21] >= -999999
 NodeArrivalVehicles[21,14]: - 1.0000229976847993e+06 x[21,14]
   + start_service[14] - start_service[21] >= -999999
 NodeArrivalVehicles[21,15]: - 1.0000096898391704e+06 x[21,15]
   + start_service[15] - start_service[21] >= -999999
 NodeArrivalVehicles[21,16]: - 999999.01 x[21,16] + start_service[16]
   - start_service[21] >= -999999
 NodeArrivalVehicles[21,17]: - 1.0000345266163895e+06 x[21,17]
   + start_service[17] - start_service[21] >= -999999
 NodeArrivalVehicles[21,18]: - 1.0000341454712306e+06 x[21,18]
   + start_service[18] - start_service[21] >= -999999
 NodeArrivalVehicles[21,19]: - 1.0000229976847993e+06 x[21,19]
   + start_service[19] - start_service[21] >= -999999
 NodeArrivalVehicles[21,20]: - 1.0000096898391704e+06 x[21,20]
   + start_service[20] - start_service[21] >= -999999
 NodeArrivalVehicles[21,21]: - 999999.01 x[21,21] >= -999999
 NodeArrivalVehicles[21,22]: - 1.0000345266163895e+06 x[21,22]
   - start_service[21] + start_service[22] >= -999999
 NodeArrivalVehicles[21,23]: - 1.000022879122035e+06 x[21,23]
   - start_service[21] + start_service[23] >= -999999
 NodeArrivalVehicles[21,24]: - 1.000022879122035e+06 x[21,24]
   - start_service[21] + start_service[24] >= -999999
 NodeArrivalVehicles[21,25]: - 1.000022879122035e+06 x[21,25]
   - start_service[21] + start_service[25] >= -999999
 NodeArrivalVehicles[22,1]: - 1.0000241592465249e+06 x[22,1]
   + start_service[1] - start_service[22] >= -999999
 NodeArrivalVehicles[22,2]: - 1.0000241592465249e+06 x[22,2]
   + start_service[2] - start_service[22] >= -999999
 NodeArrivalVehicles[22,3]: - 1.0000688935928658e+06 x[22,3]
   + start_service[3] - start_service[22] >= -999999
 NodeArrivalVehicles[22,4]: - 1.0000305908017206e+06 x[22,4]
   + start_service[4] - start_service[22] >= -999999
 NodeArrivalVehicles[22,5]: - 1.0000402331808718e+06 x[22,5]
   + start_service[5] - start_service[22] >= -999999
 NodeArrivalVehicles[22,6]: - 1.0000345266163895e+06 x[22,6]
   + start_service[6] - start_service[22] >= -999999
 NodeArrivalVehicles[22,7]: - 999999.01 x[22,7] + start_service[7]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,8]: - 1.0000688935928658e+06 x[22,8]
   + start_service[8] - start_service[22] >= -999999
 NodeArrivalVehicles[22,9]: - 1.0000305908017206e+06 x[22,9]
   + start_service[9] - start_service[22] >= -999999
 NodeArrivalVehicles[22,10]: - 1.0000402331808718e+06 x[22,10]
   + start_service[10] - start_service[22] >= -999999
 NodeArrivalVehicles[22,11]: - 1.0000345266163895e+06 x[22,11]
   + start_service[11] - start_service[22] >= -999999
 NodeArrivalVehicles[22,12]: - 999999.01 x[22,12] + start_service[12]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,13]: - 1.0000688935928658e+06 x[22,13]
   + start_service[13] - start_service[22] >= -999999
 NodeArrivalVehicles[22,14]: - 1.0000305908017206e+06 x[22,14]
   + start_service[14] - start_service[22] >= -999999
 NodeArrivalVehicles[22,15]: - 1.0000402331808718e+06 x[22,15]
   + start_service[15] - start_service[22] >= -999999
 NodeArrivalVehicles[22,16]: - 1.0000345266163895e+06 x[22,16]
   + start_service[16] - start_service[22] >= -999999
 NodeArrivalVehicles[22,17]: - 999999.01 x[22,17] + start_service[17]
   - start_service[22] >= -999999
 NodeArrivalVehicles[22,18]: - 1.0000688935928658e+06 x[22,18]
   + start_service[18] - start_service[22] >= -999999
 NodeArrivalVehicles[22,19]: - 1.0000305908017206e+06 x[22,19]
   + start_service[19] - start_service[22] >= -999999
 NodeArrivalVehicles[22,20]: - 1.0000402331808718e+06 x[22,20]
   + start_service[20] - start_service[22] >= -999999
 NodeArrivalVehicles[22,21]: - 1.0000345266163895e+06 x[22,21]
   + start_service[21] - start_service[22] >= -999999
 NodeArrivalVehicles[22,22]: - 999999.01 x[22,22] >= -999999
 NodeArrivalVehicles[22,23]: - 1.0000241592465249e+06 x[22,23]
   - start_service[22] + start_service[23] >= -999999
 NodeArrivalVehicles[22,24]: - 1.0000241592465249e+06 x[22,24]
   - start_service[22] + start_service[24] >= -999999
 NodeArrivalVehicles[22,25]: - 1.0000241592465249e+06 x[22,25]
   - start_service[22] + start_service[25] >= -999999
 NodeArrivalVehicles[23,1]: - 999999.01 x[23,1] + start_service[1]
   - start_service[23] >= -999999
 NodeArrivalVehicles[23,2]: - 999999.01 x[23,2] + start_service[2]
   - start_service[23] >= -999999
 NodeArrivalVehicles[23,3]: - 1.0000563369351183e+06 x[23,3]
   + start_service[3] - start_service[23] >= -999999
 NodeArrivalVehicles[23,4]: - 1.0000371105987907e+06 x[23,4]
   + start_service[4] - start_service[23] >= -999999
 NodeArrivalVehicles[23,5]: - 1.0000331317466504e+06 x[23,5]
   + start_service[5] - start_service[23] >= -999999
 NodeArrivalVehicles[23,6]: - 1.000022879122035e+06 x[23,6]
   + start_service[6] - start_service[23] >= -999999
 NodeArrivalVehicles[23,7]: - 1.0000241592465249e+06 x[23,7]
   + start_service[7] - start_service[23] >= -999999
 NodeArrivalVehicles[23,8]: - 1.0000563369351183e+06 x[23,8]
   + start_service[8] - start_service[23] >= -999999
 NodeArrivalVehicles[23,9]: - 1.0000371105987907e+06 x[23,9]
   + start_service[9] - start_service[23] >= -999999
 NodeArrivalVehicles[23,10]: - 1.0000331317466504e+06 x[23,10]
   + start_service[10] - start_service[23] >= -999999
 NodeArrivalVehicles[23,11]: - 1.000022879122035e+06 x[23,11]
   + start_service[11] - start_service[23] >= -999999
 NodeArrivalVehicles[23,12]: - 1.0000241592465249e+06 x[23,12]
   + start_service[12] - start_service[23] >= -999999
 NodeArrivalVehicles[23,13]: - 1.0000563369351183e+06 x[23,13]
   + start_service[13] - start_service[23] >= -999999
 NodeArrivalVehicles[23,14]: - 1.0000371105987907e+06 x[23,14]
   + start_service[14] - start_service[23] >= -999999
 NodeArrivalVehicles[23,15]: - 1.0000331317466504e+06 x[23,15]
   + start_service[15] - start_service[23] >= -999999
 NodeArrivalVehicles[23,16]: - 1.000022879122035e+06 x[23,16]
   + start_service[16] - start_service[23] >= -999999
 NodeArrivalVehicles[23,17]: - 1.0000241592465249e+06 x[23,17]
   + start_service[17] - start_service[23] >= -999999
 NodeArrivalVehicles[23,18]: - 1.0000563369351183e+06 x[23,18]
   + start_service[18] - start_service[23] >= -999999
 NodeArrivalVehicles[23,19]: - 1.0000371105987907e+06 x[23,19]
   + start_service[19] - start_service[23] >= -999999
 NodeArrivalVehicles[23,20]: - 1.0000331317466504e+06 x[23,20]
   + start_service[20] - start_service[23] >= -999999
 NodeArrivalVehicles[23,21]: - 1.000022879122035e+06 x[23,21]
   + start_service[21] - start_service[23] >= -999999
 NodeArrivalVehicles[23,22]: - 1.0000241592465249e+06 x[23,22]
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
 NodeArrivalVehicles[24,3]: - 1.0000563369351183e+06 x[24,3]
   + start_service[3] - start_service[24] >= -999999
 NodeArrivalVehicles[24,4]: - 1.0000371105987907e+06 x[24,4]
   + start_service[4] - start_service[24] >= -999999
 NodeArrivalVehicles[24,5]: - 1.0000331317466504e+06 x[24,5]
   + start_service[5] - start_service[24] >= -999999
 NodeArrivalVehicles[24,6]: - 1.000022879122035e+06 x[24,6]
   + start_service[6] - start_service[24] >= -999999
 NodeArrivalVehicles[24,7]: - 1.0000241592465249e+06 x[24,7]
   + start_service[7] - start_service[24] >= -999999
 NodeArrivalVehicles[24,8]: - 1.0000563369351183e+06 x[24,8]
   + start_service[8] - start_service[24] >= -999999
 NodeArrivalVehicles[24,9]: - 1.0000371105987907e+06 x[24,9]
   + start_service[9] - start_service[24] >= -999999
 NodeArrivalVehicles[24,10]: - 1.0000331317466504e+06 x[24,10]
   + start_service[10] - start_service[24] >= -999999
 NodeArrivalVehicles[24,11]: - 1.000022879122035e+06 x[24,11]
   + start_service[11] - start_service[24] >= -999999
 NodeArrivalVehicles[24,12]: - 1.0000241592465249e+06 x[24,12]
   + start_service[12] - start_service[24] >= -999999
 NodeArrivalVehicles[24,13]: - 1.0000563369351183e+06 x[24,13]
   + start_service[13] - start_service[24] >= -999999
 NodeArrivalVehicles[24,14]: - 1.0000371105987907e+06 x[24,14]
   + start_service[14] - start_service[24] >= -999999
 NodeArrivalVehicles[24,15]: - 1.0000331317466504e+06 x[24,15]
   + start_service[15] - start_service[24] >= -999999
 NodeArrivalVehicles[24,16]: - 1.000022879122035e+06 x[24,16]
   + start_service[16] - start_service[24] >= -999999
 NodeArrivalVehicles[24,17]: - 1.0000241592465249e+06 x[24,17]
   + start_service[17] - start_service[24] >= -999999
 NodeArrivalVehicles[24,18]: - 1.0000563369351183e+06 x[24,18]
   + start_service[18] - start_service[24] >= -999999
 NodeArrivalVehicles[24,19]: - 1.0000371105987907e+06 x[24,19]
   + start_service[19] - start_service[24] >= -999999
 NodeArrivalVehicles[24,20]: - 1.0000331317466504e+06 x[24,20]
   + start_service[20] - start_service[24] >= -999999
 NodeArrivalVehicles[24,21]: - 1.000022879122035e+06 x[24,21]
   + start_service[21] - start_service[24] >= -999999
 NodeArrivalVehicles[24,22]: - 1.0000241592465249e+06 x[24,22]
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
 StartServiceRobots[0,3]: - 1.0000563369351183e+06 v[0,3]
   - start_service[0] + start_service[3] >= -999999
 StartServiceRobots[0,4]: - 1.0000371105987907e+06 v[0,4]
   - start_service[0] + start_service[4] >= -999999
 StartServiceRobots[0,5]: - 1.0000331317466504e+06 v[0,5]
   - start_service[0] + start_service[5] >= -999999
 StartServiceRobots[0,6]: - 1.000022879122035e+06 v[0,6] - start_service[0]
   + start_service[6] >= -999999
 StartServiceRobots[0,7]: - 1.0000241592465249e+06 v[0,7]
   - start_service[0] + start_service[7] >= -999999
 StartServiceRobots[0,8]: - 1.0000563369351183e+06 v[0,8]
   - start_service[0] + start_service[8] >= -999999
 StartServiceRobots[0,9]: - 1.0000371105987907e+06 v[0,9]
   - start_service[0] + start_service[9] >= -999999
 StartServiceRobots[0,10]: - 1.0000331317466504e+06 v[0,10]
   - start_service[0] + start_service[10] >= -999999
 StartServiceRobots[0,11]: - 1.000022879122035e+06 v[0,11]
   - start_service[0] + start_service[11] >= -999999
 StartServiceRobots[0,12]: - 1.0000241592465249e+06 v[0,12]
   - start_service[0] + start_service[12] >= -999999
 StartServiceRobots[0,13]: - 1.0000563369351183e+06 v[0,13]
   - start_service[0] + start_service[13] >= -999999
 StartServiceRobots[0,14]: - 1.0000371105987907e+06 v[0,14]
   - start_service[0] + start_service[14] >= -999999
 StartServiceRobots[0,15]: - 1.0000331317466504e+06 v[0,15]
   - start_service[0] + start_service[15] >= -999999
 StartServiceRobots[0,16]: - 1.000022879122035e+06 v[0,16]
   - start_service[0] + start_service[16] >= -999999
 StartServiceRobots[0,17]: - 1.0000241592465249e+06 v[0,17]
   - start_service[0] + start_service[17] >= -999999
 StartServiceRobots[0,18]: - 1.0000563369351183e+06 v[0,18]
   - start_service[0] + start_service[18] >= -999999
 StartServiceRobots[0,19]: - 1.0000371105987907e+06 v[0,19]
   - start_service[0] + start_service[19] >= -999999
 StartServiceRobots[0,20]: - 1.0000331317466504e+06 v[0,20]
   - start_service[0] + start_service[20] >= -999999
 StartServiceRobots[0,21]: - 1.000022879122035e+06 v[0,21]
   - start_service[0] + start_service[21] >= -999999
 StartServiceRobots[0,22]: - 1.0000241592465249e+06 v[0,22]
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
 StartServiceRobots[1,3]: - 1.0000563369351183e+06 v[1,3]
   - start_service[1] + start_service[3] >= -999999
 StartServiceRobots[1,4]: - 1.0000371105987907e+06 v[1,4]
   - start_service[1] + start_service[4] >= -999999
 StartServiceRobots[1,5]: - 1.0000331317466504e+06 v[1,5]
   - start_service[1] + start_service[5] >= -999999
 StartServiceRobots[1,6]: - 1.000022879122035e+06 v[1,6] - start_service[1]
   + start_service[6] >= -999999
 StartServiceRobots[1,7]: - 1.0000241592465249e+06 v[1,7]
   - start_service[1] + start_service[7] >= -999999
 StartServiceRobots[1,8]: - 1.0000563369351183e+06 v[1,8]
   - start_service[1] + start_service[8] >= -999999
 StartServiceRobots[1,9]: - 1.0000371105987907e+06 v[1,9]
   - start_service[1] + start_service[9] >= -999999
 StartServiceRobots[1,10]: - 1.0000331317466504e+06 v[1,10]
   - start_service[1] + start_service[10] >= -999999
 StartServiceRobots[1,11]: - 1.000022879122035e+06 v[1,11]
   - start_service[1] + start_service[11] >= -999999
 StartServiceRobots[1,12]: - 1.0000241592465249e+06 v[1,12]
   - start_service[1] + start_service[12] >= -999999
 StartServiceRobots[1,13]: - 1.0000563369351183e+06 v[1,13]
   - start_service[1] + start_service[13] >= -999999
 StartServiceRobots[1,14]: - 1.0000371105987907e+06 v[1,14]
   - start_service[1] + start_service[14] >= -999999
 StartServiceRobots[1,15]: - 1.0000331317466504e+06 v[1,15]
   - start_service[1] + start_service[15] >= -999999
 StartServiceRobots[1,16]: - 1.000022879122035e+06 v[1,16]
   - start_service[1] + start_service[16] >= -999999
 StartServiceRobots[1,17]: - 1.0000241592465249e+06 v[1,17]
   - start_service[1] + start_service[17] >= -999999
 StartServiceRobots[1,18]: - 1.0000563369351183e+06 v[1,18]
   - start_service[1] + start_service[18] >= -999999
 StartServiceRobots[1,19]: - 1.0000371105987907e+06 v[1,19]
   - start_service[1] + start_service[19] >= -999999
 StartServiceRobots[1,20]: - 1.0000331317466504e+06 v[1,20]
   - start_service[1] + start_service[20] >= -999999
 StartServiceRobots[1,21]: - 1.000022879122035e+06 v[1,21]
   - start_service[1] + start_service[21] >= -999999
 StartServiceRobots[1,22]: - 1.0000241592465249e+06 v[1,22]
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
 StartServiceRobots[2,3]: - 1.0000563369351183e+06 v[2,3]
   - start_service[2] + start_service[3] >= -999999
 StartServiceRobots[2,4]: - 1.0000371105987907e+06 v[2,4]
   - start_service[2] + start_service[4] >= -999999
 StartServiceRobots[2,5]: - 1.0000331317466504e+06 v[2,5]
   - start_service[2] + start_service[5] >= -999999
 StartServiceRobots[2,6]: - 1.000022879122035e+06 v[2,6] - start_service[2]
   + start_service[6] >= -999999
 StartServiceRobots[2,7]: - 1.0000241592465249e+06 v[2,7]
   - start_service[2] + start_service[7] >= -999999
 StartServiceRobots[2,8]: - 1.0000563369351183e+06 v[2,8]
   - start_service[2] + start_service[8] >= -999999
 StartServiceRobots[2,9]: - 1.0000371105987907e+06 v[2,9]
   - start_service[2] + start_service[9] >= -999999
 StartServiceRobots[2,10]: - 1.0000331317466504e+06 v[2,10]
   - start_service[2] + start_service[10] >= -999999
 StartServiceRobots[2,11]: - 1.000022879122035e+06 v[2,11]
   - start_service[2] + start_service[11] >= -999999
 StartServiceRobots[2,12]: - 1.0000241592465249e+06 v[2,12]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 1.0000563369351183e+06 v[2,13]
   - start_service[2] + start_service[13] >= -999999
 StartServiceRobots[2,14]: - 1.0000371105987907e+06 v[2,14]
   - start_service[2] + start_service[14] >= -999999
 StartServiceRobots[2,15]: - 1.0000331317466504e+06 v[2,15]
   - start_service[2] + start_service[15] >= -999999
 StartServiceRobots[2,16]: - 1.000022879122035e+06 v[2,16]
   - start_service[2] + start_service[16] >= -999999
 StartServiceRobots[2,17]: - 1.0000241592465249e+06 v[2,17]
   - start_service[2] + start_service[17] >= -999999
 StartServiceRobots[2,18]: - 1.0000563369351183e+06 v[2,18]
   - start_service[2] + start_service[18] >= -999999
 StartServiceRobots[2,19]: - 1.0000371105987907e+06 v[2,19]
   - start_service[2] + start_service[19] >= -999999
 StartServiceRobots[2,20]: - 1.0000331317466504e+06 v[2,20]
   - start_service[2] + start_service[20] >= -999999
 StartServiceRobots[2,21]: - 1.000022879122035e+06 v[2,21]
   - start_service[2] + start_service[21] >= -999999
 StartServiceRobots[2,22]: - 1.0000241592465249e+06 v[2,22]
   - start_service[2] + start_service[22] >= -999999
 StartServiceRobots[2,23]: - 999999.01 v[2,23] - start_service[2]
   + start_service[23] >= -999999
 StartServiceRobots[2,24]: - 999999.01 v[2,24] - start_service[2]
   + start_service[24] >= -999999
 StartServiceRobots[2,25]: - 999999.01 v[2,25] - start_service[2]
   + start_service[25] >= -999999
 StartServiceRobots[3,1]: - 1.0000563369351183e+06 v[3,1]
   + start_service[1] - start_service[3] >= -999999
 StartServiceRobots[3,2]: - 1.0000563369351183e+06 v[3,2]
   + start_service[2] - start_service[3] >= -999999
 StartServiceRobots[3,3]: - 999999.01 v[3,3] >= -999999
 StartServiceRobots[3,4]: - 1.0000456469522376e+06 v[3,4]
   - start_service[3] + start_service[4] >= -999999
 StartServiceRobots[3,5]: - 1.000028055364666e+06 v[3,5] - start_service[3]
   + start_service[5] >= -999999
 StartServiceRobots[3,6]: - 1.0000341454712306e+06 v[3,6]
   - start_service[3] + start_service[6] >= -999999
 StartServiceRobots[3,7]: - 1.0000688935928658e+06 v[3,7]
   - start_service[3] + start_service[7] >= -999999
 StartServiceRobots[3,8]: - 999999.01 v[3,8] - start_service[3]
   + start_service[8] >= -999999
 StartServiceRobots[3,9]: - 1.0000456469522376e+06 v[3,9]
   - start_service[3] + start_service[9] >= -999999
 StartServiceRobots[3,10]: - 1.000028055364666e+06 v[3,10]
   - start_service[3] + start_service[10] >= -999999
 StartServiceRobots[3,11]: - 1.0000341454712306e+06 v[3,11]
   - start_service[3] + start_service[11] >= -999999
 StartServiceRobots[3,12]: - 1.0000688935928658e+06 v[3,12]
   - start_service[3] + start_service[12] >= -999999
 StartServiceRobots[3,13]: - 999999.01 v[3,13] - start_service[3]
   + start_service[13] >= -999999
 StartServiceRobots[3,14]: - 1.0000456469522376e+06 v[3,14]
   - start_service[3] + start_service[14] >= -999999
 StartServiceRobots[3,15]: - 1.000028055364666e+06 v[3,15]
   - start_service[3] + start_service[15] >= -999999
 StartServiceRobots[3,16]: - 1.0000341454712306e+06 v[3,16]
   - start_service[3] + start_service[16] >= -999999
 StartServiceRobots[3,17]: - 1.0000688935928658e+06 v[3,17]
   - start_service[3] + start_service[17] >= -999999
 StartServiceRobots[3,18]: - 999999.01 v[3,18] - start_service[3]
   + start_service[18] >= -999999
 StartServiceRobots[3,19]: - 1.0000456469522376e+06 v[3,19]
   - start_service[3] + start_service[19] >= -999999
 StartServiceRobots[3,20]: - 1.000028055364666e+06 v[3,20]
   - start_service[3] + start_service[20] >= -999999
 StartServiceRobots[3,21]: - 1.0000341454712306e+06 v[3,21]
   - start_service[3] + start_service[21] >= -999999
 StartServiceRobots[3,22]: - 1.0000688935928658e+06 v[3,22]
   - start_service[3] + start_service[22] >= -999999
 StartServiceRobots[3,23]: - 1.0000563369351183e+06 v[3,23]
   - start_service[3] + start_service[23] >= -999999
 StartServiceRobots[3,24]: - 1.0000563369351183e+06 v[3,24]
   - start_service[3] + start_service[24] >= -999999
 StartServiceRobots[3,25]: - 1.0000563369351183e+06 v[3,25]
   - start_service[3] + start_service[25] >= -999999
 StartServiceRobots[4,1]: - 1.0000371105987907e+06 v[4,1]
   + start_service[1] - start_service[4] >= -999999
 StartServiceRobots[4,2]: - 1.0000371105987907e+06 v[4,2]
   + start_service[2] - start_service[4] >= -999999
 StartServiceRobots[4,3]: - 1.0000456469522376e+06 v[4,3]
   + start_service[3] - start_service[4] >= -999999
 StartServiceRobots[4,4]: - 999999.01 v[4,4] >= -999999
 StartServiceRobots[4,5]: - 1.0000188499797942e+06 v[4,5]
   - start_service[4] + start_service[5] >= -999999
 StartServiceRobots[4,6]: - 1.0000229976847993e+06 v[4,6]
   - start_service[4] + start_service[6] >= -999999
 StartServiceRobots[4,7]: - 1.0000305908017206e+06 v[4,7]
   - start_service[4] + start_service[7] >= -999999
 StartServiceRobots[4,8]: - 1.0000456469522376e+06 v[4,8]
   - start_service[4] + start_service[8] >= -999999
 StartServiceRobots[4,9]: - 999999.01 v[4,9] - start_service[4]
   + start_service[9] >= -999999
 StartServiceRobots[4,10]: - 1.0000188499797942e+06 v[4,10]
   - start_service[4] + start_service[10] >= -999999
 StartServiceRobots[4,11]: - 1.0000229976847993e+06 v[4,11]
   - start_service[4] + start_service[11] >= -999999
 StartServiceRobots[4,12]: - 1.0000305908017206e+06 v[4,12]
   - start_service[4] + start_service[12] >= -999999
 StartServiceRobots[4,13]: - 1.0000456469522376e+06 v[4,13]
   - start_service[4] + start_service[13] >= -999999
 StartServiceRobots[4,14]: - 999999.01 v[4,14] - start_service[4]
   + start_service[14] >= -999999
 StartServiceRobots[4,15]: - 1.0000188499797942e+06 v[4,15]
   - start_service[4] + start_service[15] >= -999999
 StartServiceRobots[4,16]: - 1.0000229976847993e+06 v[4,16]
   - start_service[4] + start_service[16] >= -999999
 StartServiceRobots[4,17]: - 1.0000305908017206e+06 v[4,17]
   - start_service[4] + start_service[17] >= -999999
 StartServiceRobots[4,18]: - 1.0000456469522376e+06 v[4,18]
   - start_service[4] + start_service[18] >= -999999
 StartServiceRobots[4,19]: - 999999.01 v[4,19] - start_service[4]
   + start_service[19] >= -999999
 StartServiceRobots[4,20]: - 1.0000188499797942e+06 v[4,20]
   - start_service[4] + start_service[20] >= -999999
 StartServiceRobots[4,21]: - 1.0000229976847993e+06 v[4,21]
   - start_service[4] + start_service[21] >= -999999
 StartServiceRobots[4,22]: - 1.0000305908017206e+06 v[4,22]
   - start_service[4] + start_service[22] >= -999999
 StartServiceRobots[4,23]: - 1.0000371105987907e+06 v[4,23]
   - start_service[4] + start_service[23] >= -999999
 StartServiceRobots[4,24]: - 1.0000371105987907e+06 v[4,24]
   - start_service[4] + start_service[24] >= -999999
 StartServiceRobots[4,25]: - 1.0000371105987907e+06 v[4,25]
   - start_service[4] + start_service[25] >= -999999
 StartServiceRobots[5,1]: - 1.0000331317466504e+06 v[5,1]
   + start_service[1] - start_service[5] >= -999999
 StartServiceRobots[5,2]: - 1.0000331317466504e+06 v[5,2]
   + start_service[2] - start_service[5] >= -999999
 StartServiceRobots[5,3]: - 1.000028055364666e+06 v[5,3] + start_service[3]
   - start_service[5] >= -999999
 StartServiceRobots[5,4]: - 1.0000188499797942e+06 v[5,4]
   + start_service[4] - start_service[5] >= -999999
 StartServiceRobots[5,5]: - 999999.01 v[5,5] >= -999999
 StartServiceRobots[5,6]: - 1.0000096898391704e+06 v[5,6]
   - start_service[5] + start_service[6] >= -999999
 StartServiceRobots[5,7]: - 1.0000402331808718e+06 v[5,7]
   - start_service[5] + start_service[7] >= -999999
 StartServiceRobots[5,8]: - 1.000028055364666e+06 v[5,8] - start_service[5]
   + start_service[8] >= -999999
 StartServiceRobots[5,9]: - 1.0000188499797942e+06 v[5,9]
   - start_service[5] + start_service[9] >= -999999
 StartServiceRobots[5,10]: - 999999.01 v[5,10] - start_service[5]
   + start_service[10] >= -999999
 StartServiceRobots[5,11]: - 1.0000096898391704e+06 v[5,11]
   - start_service[5] + start_service[11] >= -999999
 StartServiceRobots[5,12]: - 1.0000402331808718e+06 v[5,12]
   - start_service[5] + start_service[12] >= -999999
 StartServiceRobots[5,13]: - 1.000028055364666e+06 v[5,13]
   - start_service[5] + start_service[13] >= -999999
 StartServiceRobots[5,14]: - 1.0000188499797942e+06 v[5,14]
   - start_service[5] + start_service[14] >= -999999
 StartServiceRobots[5,15]: - 999999.01 v[5,15] - start_service[5]
   + start_service[15] >= -999999
 StartServiceRobots[5,16]: - 1.0000096898391704e+06 v[5,16]
   - start_service[5] + start_service[16] >= -999999
 StartServiceRobots[5,17]: - 1.0000402331808718e+06 v[5,17]
   - start_service[5] + start_service[17] >= -999999
 StartServiceRobots[5,18]: - 1.000028055364666e+06 v[5,18]
   - start_service[5] + start_service[18] >= -999999
 StartServiceRobots[5,19]: - 1.0000188499797942e+06 v[5,19]
   - start_service[5] + start_service[19] >= -999999
 StartServiceRobots[5,20]: - 999999.01 v[5,20] - start_service[5]
   + start_service[20] >= -999999
 StartServiceRobots[5,21]: - 1.0000096898391704e+06 v[5,21]
   - start_service[5] + start_service[21] >= -999999
 StartServiceRobots[5,22]: - 1.0000402331808718e+06 v[5,22]
   - start_service[5] + start_service[22] >= -999999
 StartServiceRobots[5,23]: - 1.0000331317466504e+06 v[5,23]
   - start_service[5] + start_service[23] >= -999999
 StartServiceRobots[5,24]: - 1.0000331317466504e+06 v[5,24]
   - start_service[5] + start_service[24] >= -999999
 StartServiceRobots[5,25]: - 1.0000331317466504e+06 v[5,25]
   - start_service[5] + start_service[25] >= -999999
 StartServiceRobots[6,1]: - 1.000022879122035e+06 v[6,1] + start_service[1]
   - start_service[6] >= -999999
 StartServiceRobots[6,2]: - 1.000022879122035e+06 v[6,2] + start_service[2]
   - start_service[6] >= -999999
 StartServiceRobots[6,3]: - 1.0000341454712306e+06 v[6,3]
   + start_service[3] - start_service[6] >= -999999
 StartServiceRobots[6,4]: - 1.0000229976847993e+06 v[6,4]
   + start_service[4] - start_service[6] >= -999999
 StartServiceRobots[6,5]: - 1.0000096898391704e+06 v[6,5]
   + start_service[5] - start_service[6] >= -999999
 StartServiceRobots[6,6]: - 999999.01 v[6,6] >= -999999
 StartServiceRobots[6,7]: - 1.0000345266163895e+06 v[6,7]
   - start_service[6] + start_service[7] >= -999999
 StartServiceRobots[6,8]: - 1.0000341454712306e+06 v[6,8]
   - start_service[6] + start_service[8] >= -999999
 StartServiceRobots[6,9]: - 1.0000229976847993e+06 v[6,9]
   - start_service[6] + start_service[9] >= -999999
 StartServiceRobots[6,10]: - 1.0000096898391704e+06 v[6,10]
   - start_service[6] + start_service[10] >= -999999
 StartServiceRobots[6,11]: - 999999.01 v[6,11] - start_service[6]
   + start_service[11] >= -999999
 StartServiceRobots[6,12]: - 1.0000345266163895e+06 v[6,12]
   - start_service[6] + start_service[12] >= -999999
 StartServiceRobots[6,13]: - 1.0000341454712306e+06 v[6,13]
   - start_service[6] + start_service[13] >= -999999
 StartServiceRobots[6,14]: - 1.0000229976847993e+06 v[6,14]
   - start_service[6] + start_service[14] >= -999999
 StartServiceRobots[6,15]: - 1.0000096898391704e+06 v[6,15]
   - start_service[6] + start_service[15] >= -999999
 StartServiceRobots[6,16]: - 999999.01 v[6,16] - start_service[6]
   + start_service[16] >= -999999
 StartServiceRobots[6,17]: - 1.0000345266163895e+06 v[6,17]
   - start_service[6] + start_service[17] >= -999999
 StartServiceRobots[6,18]: - 1.0000341454712306e+06 v[6,18]
   - start_service[6] + start_service[18] >= -999999
 StartServiceRobots[6,19]: - 1.0000229976847993e+06 v[6,19]
   - start_service[6] + start_service[19] >= -999999
 StartServiceRobots[6,20]: - 1.0000096898391704e+06 v[6,20]
   - start_service[6] + start_service[20] >= -999999
 StartServiceRobots[6,21]: - 999999.01 v[6,21] - start_service[6]
   + start_service[21] >= -999999
 StartServiceRobots[6,22]: - 1.0000345266163895e+06 v[6,22]
   - start_service[6] + start_service[22] >= -999999
 StartServiceRobots[6,23]: - 1.000022879122035e+06 v[6,23]
   - start_service[6] + start_service[23] >= -999999
 StartServiceRobots[6,24]: - 1.000022879122035e+06 v[6,24]
   - start_service[6] + start_service[24] >= -999999
 StartServiceRobots[6,25]: - 1.000022879122035e+06 v[6,25]
   - start_service[6] + start_service[25] >= -999999
 StartServiceRobots[7,1]: - 1.0000241592465249e+06 v[7,1]
   + start_service[1] - start_service[7] >= -999999
 StartServiceRobots[7,2]: - 1.0000241592465249e+06 v[7,2]
   + start_service[2] - start_service[7] >= -999999
 StartServiceRobots[7,3]: - 1.0000688935928658e+06 v[7,3]
   + start_service[3] - start_service[7] >= -999999
 StartServiceRobots[7,4]: - 1.0000305908017206e+06 v[7,4]
   + start_service[4] - start_service[7] >= -999999
 StartServiceRobots[7,5]: - 1.0000402331808718e+06 v[7,5]
   + start_service[5] - start_service[7] >= -999999
 StartServiceRobots[7,6]: - 1.0000345266163895e+06 v[7,6]
   + start_service[6] - start_service[7] >= -999999
 StartServiceRobots[7,7]: - 999999.01 v[7,7] >= -999999
 StartServiceRobots[7,8]: - 1.0000688935928658e+06 v[7,8]
   - start_service[7] + start_service[8] >= -999999
 StartServiceRobots[7,9]: - 1.0000305908017206e+06 v[7,9]
   - start_service[7] + start_service[9] >= -999999
 StartServiceRobots[7,10]: - 1.0000402331808718e+06 v[7,10]
   - start_service[7] + start_service[10] >= -999999
 StartServiceRobots[7,11]: - 1.0000345266163895e+06 v[7,11]
   - start_service[7] + start_service[11] >= -999999
 StartServiceRobots[7,12]: - 999999.01 v[7,12] - start_service[7]
   + start_service[12] >= -999999
 StartServiceRobots[7,13]: - 1.0000688935928658e+06 v[7,13]
   - start_service[7] + start_service[13] >= -999999
 StartServiceRobots[7,14]: - 1.0000305908017206e+06 v[7,14]
   - start_service[7] + start_service[14] >= -999999
 StartServiceRobots[7,15]: - 1.0000402331808718e+06 v[7,15]
   - start_service[7] + start_service[15] >= -999999
 StartServiceRobots[7,16]: - 1.0000345266163895e+06 v[7,16]
   - start_service[7] + start_service[16] >= -999999
 StartServiceRobots[7,17]: - 999999.01 v[7,17] - start_service[7]
   + start_service[17] >= -999999
 StartServiceRobots[7,18]: - 1.0000688935928658e+06 v[7,18]
   - start_service[7] + start_service[18] >= -999999
 StartServiceRobots[7,19]: - 1.0000305908017206e+06 v[7,19]
   - start_service[7] + start_service[19] >= -999999
 StartServiceRobots[7,20]: - 1.0000402331808718e+06 v[7,20]
   - start_service[7] + start_service[20] >= -999999
 StartServiceRobots[7,21]: - 1.0000345266163895e+06 v[7,21]
   - start_service[7] + start_service[21] >= -999999
 StartServiceRobots[7,22]: - 999999.01 v[7,22] - start_service[7]
   + start_service[22] >= -999999
 StartServiceRobots[7,23]: - 1.0000241592465249e+06 v[7,23]
   - start_service[7] + start_service[23] >= -999999
 StartServiceRobots[7,24]: - 1.0000241592465249e+06 v[7,24]
   - start_service[7] + start_service[24] >= -999999
 StartServiceRobots[7,25]: - 1.0000241592465249e+06 v[7,25]
   - start_service[7] + start_service[25] >= -999999
 StartServiceRobots[8,1]: - 1.0000563369351183e+06 v[8,1]
   + start_service[1] - start_service[8] >= -999999
 StartServiceRobots[8,2]: - 1.0000563369351183e+06 v[8,2]
   + start_service[2] - start_service[8] >= -999999
 StartServiceRobots[8,3]: - 999999.01 v[8,3] + start_service[3]
   - start_service[8] >= -999999
 StartServiceRobots[8,4]: - 1.0000456469522376e+06 v[8,4]
   + start_service[4] - start_service[8] >= -999999
 StartServiceRobots[8,5]: - 1.000028055364666e+06 v[8,5] + start_service[5]
   - start_service[8] >= -999999
 StartServiceRobots[8,6]: - 1.0000341454712306e+06 v[8,6]
   + start_service[6] - start_service[8] >= -999999
 StartServiceRobots[8,7]: - 1.0000688935928658e+06 v[8,7]
   + start_service[7] - start_service[8] >= -999999
 StartServiceRobots[8,8]: - 999999.01 v[8,8] >= -999999
 StartServiceRobots[8,9]: - 1.0000456469522376e+06 v[8,9]
   - start_service[8] + start_service[9] >= -999999
 StartServiceRobots[8,10]: - 1.000028055364666e+06 v[8,10]
   - start_service[8] + start_service[10] >= -999999
 StartServiceRobots[8,11]: - 1.0000341454712306e+06 v[8,11]
   - start_service[8] + start_service[11] >= -999999
 StartServiceRobots[8,12]: - 1.0000688935928658e+06 v[8,12]
   - start_service[8] + start_service[12] >= -999999
 StartServiceRobots[8,13]: - 999999.01 v[8,13] - start_service[8]
   + start_service[13] >= -999999
 StartServiceRobots[8,14]: - 1.0000456469522376e+06 v[8,14]
   - start_service[8] + start_service[14] >= -999999
 StartServiceRobots[8,15]: - 1.000028055364666e+06 v[8,15]
   - start_service[8] + start_service[15] >= -999999
 StartServiceRobots[8,16]: - 1.0000341454712306e+06 v[8,16]
   - start_service[8] + start_service[16] >= -999999
 StartServiceRobots[8,17]: - 1.0000688935928658e+06 v[8,17]
   - start_service[8] + start_service[17] >= -999999
 StartServiceRobots[8,18]: - 999999.01 v[8,18] - start_service[8]
   + start_service[18] >= -999999
 StartServiceRobots[8,19]: - 1.0000456469522376e+06 v[8,19]
   - start_service[8] + start_service[19] >= -999999
 StartServiceRobots[8,20]: - 1.000028055364666e+06 v[8,20]
   - start_service[8] + start_service[20] >= -999999
 StartServiceRobots[8,21]: - 1.0000341454712306e+06 v[8,21]
   - start_service[8] + start_service[21] >= -999999
 StartServiceRobots[8,22]: - 1.0000688935928658e+06 v[8,22]
   - start_service[8] + start_service[22] >= -999999
 StartServiceRobots[8,23]: - 1.0000563369351183e+06 v[8,23]
   - start_service[8] + start_service[23] >= -999999
 StartServiceRobots[8,24]: - 1.0000563369351183e+06 v[8,24]
   - start_service[8] + start_service[24] >= -999999
 StartServiceRobots[8,25]: - 1.0000563369351183e+06 v[8,25]
   - start_service[8] + start_service[25] >= -999999
 StartServiceRobots[9,1]: - 1.0000371105987907e+06 v[9,1]
   + start_service[1] - start_service[9] >= -999999
 StartServiceRobots[9,2]: - 1.0000371105987907e+06 v[9,2]
   + start_service[2] - start_service[9] >= -999999
 StartServiceRobots[9,3]: - 1.0000456469522376e+06 v[9,3]
   + start_service[3] - start_service[9] >= -999999
 StartServiceRobots[9,4]: - 999999.01 v[9,4] + start_service[4]
   - start_service[9] >= -999999
 StartServiceRobots[9,5]: - 1.0000188499797942e+06 v[9,5]
   + start_service[5] - start_service[9] >= -999999
 StartServiceRobots[9,6]: - 1.0000229976847993e+06 v[9,6]
   + start_service[6] - start_service[9] >= -999999
 StartServiceRobots[9,7]: - 1.0000305908017206e+06 v[9,7]
   + start_service[7] - start_service[9] >= -999999
 StartServiceRobots[9,8]: - 1.0000456469522376e+06 v[9,8]
   + start_service[8] - start_service[9] >= -999999
 StartServiceRobots[9,9]: - 999999.01 v[9,9] >= -999999
 StartServiceRobots[9,10]: - 1.0000188499797942e+06 v[9,10]
   - start_service[9] + start_service[10] >= -999999
 StartServiceRobots[9,11]: - 1.0000229976847993e+06 v[9,11]
   - start_service[9] + start_service[11] >= -999999
 StartServiceRobots[9,12]: - 1.0000305908017206e+06 v[9,12]
   - start_service[9] + start_service[12] >= -999999
 StartServiceRobots[9,13]: - 1.0000456469522376e+06 v[9,13]
   - start_service[9] + start_service[13] >= -999999
 StartServiceRobots[9,14]: - 999999.01 v[9,14] - start_service[9]
   + start_service[14] >= -999999
 StartServiceRobots[9,15]: - 1.0000188499797942e+06 v[9,15]
   - start_service[9] + start_service[15] >= -999999
 StartServiceRobots[9,16]: - 1.0000229976847993e+06 v[9,16]
   - start_service[9] + start_service[16] >= -999999
 StartServiceRobots[9,17]: - 1.0000305908017206e+06 v[9,17]
   - start_service[9] + start_service[17] >= -999999
 StartServiceRobots[9,18]: - 1.0000456469522376e+06 v[9,18]
   - start_service[9] + start_service[18] >= -999999
 StartServiceRobots[9,19]: - 999999.01 v[9,19] - start_service[9]
   + start_service[19] >= -999999
 StartServiceRobots[9,20]: - 1.0000188499797942e+06 v[9,20]
   - start_service[9] + start_service[20] >= -999999
 StartServiceRobots[9,21]: - 1.0000229976847993e+06 v[9,21]
   - start_service[9] + start_service[21] >= -999999
 StartServiceRobots[9,22]: - 1.0000305908017206e+06 v[9,22]
   - start_service[9] + start_service[22] >= -999999
 StartServiceRobots[9,23]: - 1.0000371105987907e+06 v[9,23]
   - start_service[9] + start_service[23] >= -999999
 StartServiceRobots[9,24]: - 1.0000371105987907e+06 v[9,24]
   - start_service[9] + start_service[24] >= -999999
 StartServiceRobots[9,25]: - 1.0000371105987907e+06 v[9,25]
   - start_service[9] + start_service[25] >= -999999
 StartServiceRobots[10,1]: - 1.0000331317466504e+06 v[10,1]
   + start_service[1] - start_service[10] >= -999999
 StartServiceRobots[10,2]: - 1.0000331317466504e+06 v[10,2]
   + start_service[2] - start_service[10] >= -999999
 StartServiceRobots[10,3]: - 1.000028055364666e+06 v[10,3]
   + start_service[3] - start_service[10] >= -999999
 StartServiceRobots[10,4]: - 1.0000188499797942e+06 v[10,4]
   + start_service[4] - start_service[10] >= -999999
 StartServiceRobots[10,5]: - 999999.01 v[10,5] + start_service[5]
   - start_service[10] >= -999999
 StartServiceRobots[10,6]: - 1.0000096898391704e+06 v[10,6]
   + start_service[6] - start_service[10] >= -999999
 StartServiceRobots[10,7]: - 1.0000402331808718e+06 v[10,7]
   + start_service[7] - start_service[10] >= -999999
 StartServiceRobots[10,8]: - 1.000028055364666e+06 v[10,8]
   + start_service[8] - start_service[10] >= -999999
 StartServiceRobots[10,9]: - 1.0000188499797942e+06 v[10,9]
   + start_service[9] - start_service[10] >= -999999
 StartServiceRobots[10,10]: - 999999.01 v[10,10] >= -999999
 StartServiceRobots[10,11]: - 1.0000096898391704e+06 v[10,11]
   - start_service[10] + start_service[11] >= -999999
 StartServiceRobots[10,12]: - 1.0000402331808718e+06 v[10,12]
   - start_service[10] + start_service[12] >= -999999
 StartServiceRobots[10,13]: - 1.000028055364666e+06 v[10,13]
   - start_service[10] + start_service[13] >= -999999
 StartServiceRobots[10,14]: - 1.0000188499797942e+06 v[10,14]
   - start_service[10] + start_service[14] >= -999999
 StartServiceRobots[10,15]: - 999999.01 v[10,15] - start_service[10]
   + start_service[15] >= -999999
 StartServiceRobots[10,16]: - 1.0000096898391704e+06 v[10,16]
   - start_service[10] + start_service[16] >= -999999
 StartServiceRobots[10,17]: - 1.0000402331808718e+06 v[10,17]
   - start_service[10] + start_service[17] >= -999999
 StartServiceRobots[10,18]: - 1.000028055364666e+06 v[10,18]
   - start_service[10] + start_service[18] >= -999999
 StartServiceRobots[10,19]: - 1.0000188499797942e+06 v[10,19]
   - start_service[10] + start_service[19] >= -999999
 StartServiceRobots[10,20]: - 999999.01 v[10,20] - start_service[10]
   + start_service[20] >= -999999
 StartServiceRobots[10,21]: - 1.0000096898391704e+06 v[10,21]
   - start_service[10] + start_service[21] >= -999999
 StartServiceRobots[10,22]: - 1.0000402331808718e+06 v[10,22]
   - start_service[10] + start_service[22] >= -999999
 StartServiceRobots[10,23]: - 1.0000331317466504e+06 v[10,23]
   - start_service[10] + start_service[23] >= -999999
 StartServiceRobots[10,24]: - 1.0000331317466504e+06 v[10,24]
   - start_service[10] + start_service[24] >= -999999
 StartServiceRobots[10,25]: - 1.0000331317466504e+06 v[10,25]
   - start_service[10] + start_service[25] >= -999999
 StartServiceRobots[11,1]: - 1.000022879122035e+06 v[11,1]
   + start_service[1] - start_service[11] >= -999999
 StartServiceRobots[11,2]: - 1.000022879122035e+06 v[11,2]
   + start_service[2] - start_service[11] >= -999999
 StartServiceRobots[11,3]: - 1.0000341454712306e+06 v[11,3]
   + start_service[3] - start_service[11] >= -999999
 StartServiceRobots[11,4]: - 1.0000229976847993e+06 v[11,4]
   + start_service[4] - start_service[11] >= -999999
 StartServiceRobots[11,5]: - 1.0000096898391704e+06 v[11,5]
   + start_service[5] - start_service[11] >= -999999
 StartServiceRobots[11,6]: - 999999.01 v[11,6] + start_service[6]
   - start_service[11] >= -999999
 StartServiceRobots[11,7]: - 1.0000345266163895e+06 v[11,7]
   + start_service[7] - start_service[11] >= -999999
 StartServiceRobots[11,8]: - 1.0000341454712306e+06 v[11,8]
   + start_service[8] - start_service[11] >= -999999
 StartServiceRobots[11,9]: - 1.0000229976847993e+06 v[11,9]
   + start_service[9] - start_service[11] >= -999999
 StartServiceRobots[11,10]: - 1.0000096898391704e+06 v[11,10]
   + start_service[10] - start_service[11] >= -999999
 StartServiceRobots[11,11]: - 999999.01 v[11,11] >= -999999
 StartServiceRobots[11,12]: - 1.0000345266163895e+06 v[11,12]
   - start_service[11] + start_service[12] >= -999999
 StartServiceRobots[11,13]: - 1.0000341454712306e+06 v[11,13]
   - start_service[11] + start_service[13] >= -999999
 StartServiceRobots[11,14]: - 1.0000229976847993e+06 v[11,14]
   - start_service[11] + start_service[14] >= -999999
 StartServiceRobots[11,15]: - 1.0000096898391704e+06 v[11,15]
   - start_service[11] + start_service[15] >= -999999
 StartServiceRobots[11,16]: - 999999.01 v[11,16] - start_service[11]
   + start_service[16] >= -999999
 StartServiceRobots[11,17]: - 1.0000345266163895e+06 v[11,17]
   - start_service[11] + start_service[17] >= -999999
 StartServiceRobots[11,18]: - 1.0000341454712306e+06 v[11,18]
   - start_service[11] + start_service[18] >= -999999
 StartServiceRobots[11,19]: - 1.0000229976847993e+06 v[11,19]
   - start_service[11] + start_service[19] >= -999999
 StartServiceRobots[11,20]: - 1.0000096898391704e+06 v[11,20]
   - start_service[11] + start_service[20] >= -999999
 StartServiceRobots[11,21]: - 999999.01 v[11,21] - start_service[11]
   + start_service[21] >= -999999
 StartServiceRobots[11,22]: - 1.0000345266163895e+06 v[11,22]
   - start_service[11] + start_service[22] >= -999999
 StartServiceRobots[11,23]: - 1.000022879122035e+06 v[11,23]
   - start_service[11] + start_service[23] >= -999999
 StartServiceRobots[11,24]: - 1.000022879122035e+06 v[11,24]
   - start_service[11] + start_service[24] >= -999999
 StartServiceRobots[11,25]: - 1.000022879122035e+06 v[11,25]
   - start_service[11] + start_service[25] >= -999999
 StartServiceRobots[12,1]: - 1.0000241592465249e+06 v[12,1]
   + start_service[1] - start_service[12] >= -999999
 StartServiceRobots[12,2]: - 1.0000241592465249e+06 v[12,2]
   + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 1.0000688935928658e+06 v[12,3]
   + start_service[3] - start_service[12] >= -999999
 StartServiceRobots[12,4]: - 1.0000305908017206e+06 v[12,4]
   + start_service[4] - start_service[12] >= -999999
 StartServiceRobots[12,5]: - 1.0000402331808718e+06 v[12,5]
   + start_service[5] - start_service[12] >= -999999
 StartServiceRobots[12,6]: - 1.0000345266163895e+06 v[12,6]
   + start_service[6] - start_service[12] >= -999999
 StartServiceRobots[12,7]: - 999999.01 v[12,7] + start_service[7]
   - start_service[12] >= -999999
 StartServiceRobots[12,8]: - 1.0000688935928658e+06 v[12,8]
   + start_service[8] - start_service[12] >= -999999
 StartServiceRobots[12,9]: - 1.0000305908017206e+06 v[12,9]
   + start_service[9] - start_service[12] >= -999999
 StartServiceRobots[12,10]: - 1.0000402331808718e+06 v[12,10]
   + start_service[10] - start_service[12] >= -999999
 StartServiceRobots[12,11]: - 1.0000345266163895e+06 v[12,11]
   + start_service[11] - start_service[12] >= -999999
 StartServiceRobots[12,12]: - 999999.01 v[12,12] >= -999999
 StartServiceRobots[12,13]: - 1.0000688935928658e+06 v[12,13]
   - start_service[12] + start_service[13] >= -999999
 StartServiceRobots[12,14]: - 1.0000305908017206e+06 v[12,14]
   - start_service[12] + start_service[14] >= -999999
 StartServiceRobots[12,15]: - 1.0000402331808718e+06 v[12,15]
   - start_service[12] + start_service[15] >= -999999
 StartServiceRobots[12,16]: - 1.0000345266163895e+06 v[12,16]
   - start_service[12] + start_service[16] >= -999999
 StartServiceRobots[12,17]: - 999999.01 v[12,17] - start_service[12]
   + start_service[17] >= -999999
 StartServiceRobots[12,18]: - 1.0000688935928658e+06 v[12,18]
   - start_service[12] + start_service[18] >= -999999
 StartServiceRobots[12,19]: - 1.0000305908017206e+06 v[12,19]
   - start_service[12] + start_service[19] >= -999999
 StartServiceRobots[12,20]: - 1.0000402331808718e+06 v[12,20]
   - start_service[12] + start_service[20] >= -999999
 StartServiceRobots[12,21]: - 1.0000345266163895e+06 v[12,21]
   - start_service[12] + start_service[21] >= -999999
 StartServiceRobots[12,22]: - 999999.01 v[12,22] - start_service[12]
   + start_service[22] >= -999999
 StartServiceRobots[12,23]: - 1.0000241592465249e+06 v[12,23]
   - start_service[12] + start_service[23] >= -999999
 StartServiceRobots[12,24]: - 1.0000241592465249e+06 v[12,24]
   - start_service[12] + start_service[24] >= -999999
 StartServiceRobots[12,25]: - 1.0000241592465249e+06 v[12,25]
   - start_service[12] + start_service[25] >= -999999
 StartServiceRobots[13,1]: - 1.0000563369351183e+06 v[13,1]
   + start_service[1] - start_service[13] >= -999999
 StartServiceRobots[13,2]: - 1.0000563369351183e+06 v[13,2]
   + start_service[2] - start_service[13] >= -999999
 StartServiceRobots[13,3]: - 999999.01 v[13,3] + start_service[3]
   - start_service[13] >= -999999
 StartServiceRobots[13,4]: - 1.0000456469522376e+06 v[13,4]
   + start_service[4] - start_service[13] >= -999999
 StartServiceRobots[13,5]: - 1.000028055364666e+06 v[13,5]
   + start_service[5] - start_service[13] >= -999999
 StartServiceRobots[13,6]: - 1.0000341454712306e+06 v[13,6]
   + start_service[6] - start_service[13] >= -999999
 StartServiceRobots[13,7]: - 1.0000688935928658e+06 v[13,7]
   + start_service[7] - start_service[13] >= -999999
 StartServiceRobots[13,8]: - 999999.01 v[13,8] + start_service[8]
   - start_service[13] >= -999999
 StartServiceRobots[13,9]: - 1.0000456469522376e+06 v[13,9]
   + start_service[9] - start_service[13] >= -999999
 StartServiceRobots[13,10]: - 1.000028055364666e+06 v[13,10]
   + start_service[10] - start_service[13] >= -999999
 StartServiceRobots[13,11]: - 1.0000341454712306e+06 v[13,11]
   + start_service[11] - start_service[13] >= -999999
 StartServiceRobots[13,12]: - 1.0000688935928658e+06 v[13,12]
   + start_service[12] - start_service[13] >= -999999
 StartServiceRobots[13,13]: - 999999.01 v[13,13] >= -999999
 StartServiceRobots[13,14]: - 1.0000456469522376e+06 v[13,14]
   - start_service[13] + start_service[14] >= -999999
 StartServiceRobots[13,15]: - 1.000028055364666e+06 v[13,15]
   - start_service[13] + start_service[15] >= -999999
 StartServiceRobots[13,16]: - 1.0000341454712306e+06 v[13,16]
   - start_service[13] + start_service[16] >= -999999
 StartServiceRobots[13,17]: - 1.0000688935928658e+06 v[13,17]
   - start_service[13] + start_service[17] >= -999999
 StartServiceRobots[13,18]: - 999999.01 v[13,18] - start_service[13]
   + start_service[18] >= -999999
 StartServiceRobots[13,19]: - 1.0000456469522376e+06 v[13,19]
   - start_service[13] + start_service[19] >= -999999
 StartServiceRobots[13,20]: - 1.000028055364666e+06 v[13,20]
   - start_service[13] + start_service[20] >= -999999
 StartServiceRobots[13,21]: - 1.0000341454712306e+06 v[13,21]
   - start_service[13] + start_service[21] >= -999999
 StartServiceRobots[13,22]: - 1.0000688935928658e+06 v[13,22]
   - start_service[13] + start_service[22] >= -999999
 StartServiceRobots[13,23]: - 1.0000563369351183e+06 v[13,23]
   - start_service[13] + start_service[23] >= -999999
 StartServiceRobots[13,24]: - 1.0000563369351183e+06 v[13,24]
   - start_service[13] + start_service[24] >= -999999
 StartServiceRobots[13,25]: - 1.0000563369351183e+06 v[13,25]
   - start_service[13] + start_service[25] >= -999999
 StartServiceRobots[14,1]: - 1.0000371105987907e+06 v[14,1]
   + start_service[1] - start_service[14] >= -999999
 StartServiceRobots[14,2]: - 1.0000371105987907e+06 v[14,2]
   + start_service[2] - start_service[14] >= -999999
 StartServiceRobots[14,3]: - 1.0000456469522376e+06 v[14,3]
   + start_service[3] - start_service[14] >= -999999
 StartServiceRobots[14,4]: - 999999.01 v[14,4] + start_service[4]
   - start_service[14] >= -999999
 StartServiceRobots[14,5]: - 1.0000188499797942e+06 v[14,5]
   + start_service[5] - start_service[14] >= -999999
 StartServiceRobots[14,6]: - 1.0000229976847993e+06 v[14,6]
   + start_service[6] - start_service[14] >= -999999
 StartServiceRobots[14,7]: - 1.0000305908017206e+06 v[14,7]
   + start_service[7] - start_service[14] >= -999999
 StartServiceRobots[14,8]: - 1.0000456469522376e+06 v[14,8]
   + start_service[8] - start_service[14] >= -999999
 StartServiceRobots[14,9]: - 999999.01 v[14,9] + start_service[9]
   - start_service[14] >= -999999
 StartServiceRobots[14,10]: - 1.0000188499797942e+06 v[14,10]
   + start_service[10] - start_service[14] >= -999999
 StartServiceRobots[14,11]: - 1.0000229976847993e+06 v[14,11]
   + start_service[11] - start_service[14] >= -999999
 StartServiceRobots[14,12]: - 1.0000305908017206e+06 v[14,12]
   + start_service[12] - start_service[14] >= -999999
 StartServiceRobots[14,13]: - 1.0000456469522376e+06 v[14,13]
   + start_service[13] - start_service[14] >= -999999
 StartServiceRobots[14,14]: - 999999.01 v[14,14] >= -999999
 StartServiceRobots[14,15]: - 1.0000188499797942e+06 v[14,15]
   - start_service[14] + start_service[15] >= -999999
 StartServiceRobots[14,16]: - 1.0000229976847993e+06 v[14,16]
   - start_service[14] + start_service[16] >= -999999
 StartServiceRobots[14,17]: - 1.0000305908017206e+06 v[14,17]
   - start_service[14] + start_service[17] >= -999999
 StartServiceRobots[14,18]: - 1.0000456469522376e+06 v[14,18]
   - start_service[14] + start_service[18] >= -999999
 StartServiceRobots[14,19]: - 999999.01 v[14,19] - start_service[14]
   + start_service[19] >= -999999
 StartServiceRobots[14,20]: - 1.0000188499797942e+06 v[14,20]
   - start_service[14] + start_service[20] >= -999999
 StartServiceRobots[14,21]: - 1.0000229976847993e+06 v[14,21]
   - start_service[14] + start_service[21] >= -999999
 StartServiceRobots[14,22]: - 1.0000305908017206e+06 v[14,22]
   - start_service[14] + start_service[22] >= -999999
 StartServiceRobots[14,23]: - 1.0000371105987907e+06 v[14,23]
   - start_service[14] + start_service[23] >= -999999
 StartServiceRobots[14,24]: - 1.0000371105987907e+06 v[14,24]
   - start_service[14] + start_service[24] >= -999999
 StartServiceRobots[14,25]: - 1.0000371105987907e+06 v[14,25]
   - start_service[14] + start_service[25] >= -999999
 StartServiceRobots[15,1]: - 1.0000331317466504e+06 v[15,1]
   + start_service[1] - start_service[15] >= -999999
 StartServiceRobots[15,2]: - 1.0000331317466504e+06 v[15,2]
   + start_service[2] - start_service[15] >= -999999
 StartServiceRobots[15,3]: - 1.000028055364666e+06 v[15,3]
   + start_service[3] - start_service[15] >= -999999
 StartServiceRobots[15,4]: - 1.0000188499797942e+06 v[15,4]
   + start_service[4] - start_service[15] >= -999999
 StartServiceRobots[15,5]: - 999999.01 v[15,5] + start_service[5]
   - start_service[15] >= -999999
 StartServiceRobots[15,6]: - 1.0000096898391704e+06 v[15,6]
   + start_service[6] - start_service[15] >= -999999
 StartServiceRobots[15,7]: - 1.0000402331808718e+06 v[15,7]
   + start_service[7] - start_service[15] >= -999999
 StartServiceRobots[15,8]: - 1.000028055364666e+06 v[15,8]
   + start_service[8] - start_service[15] >= -999999
 StartServiceRobots[15,9]: - 1.0000188499797942e+06 v[15,9]
   + start_service[9] - start_service[15] >= -999999
 StartServiceRobots[15,10]: - 999999.01 v[15,10] + start_service[10]
   - start_service[15] >= -999999
 StartServiceRobots[15,11]: - 1.0000096898391704e+06 v[15,11]
   + start_service[11] - start_service[15] >= -999999
 StartServiceRobots[15,12]: - 1.0000402331808718e+06 v[15,12]
   + start_service[12] - start_service[15] >= -999999
 StartServiceRobots[15,13]: - 1.000028055364666e+06 v[15,13]
   + start_service[13] - start_service[15] >= -999999
 StartServiceRobots[15,14]: - 1.0000188499797942e+06 v[15,14]
   + start_service[14] - start_service[15] >= -999999
 StartServiceRobots[15,15]: - 999999.01 v[15,15] >= -999999
 StartServiceRobots[15,16]: - 1.0000096898391704e+06 v[15,16]
   - start_service[15] + start_service[16] >= -999999
 StartServiceRobots[15,17]: - 1.0000402331808718e+06 v[15,17]
   - start_service[15] + start_service[17] >= -999999
 StartServiceRobots[15,18]: - 1.000028055364666e+06 v[15,18]
   - start_service[15] + start_service[18] >= -999999
 StartServiceRobots[15,19]: - 1.0000188499797942e+06 v[15,19]
   - start_service[15] + start_service[19] >= -999999
 StartServiceRobots[15,20]: - 999999.01 v[15,20] - start_service[15]
   + start_service[20] >= -999999
 StartServiceRobots[15,21]: - 1.0000096898391704e+06 v[15,21]
   - start_service[15] + start_service[21] >= -999999
 StartServiceRobots[15,22]: - 1.0000402331808718e+06 v[15,22]
   - start_service[15] + start_service[22] >= -999999
 StartServiceRobots[15,23]: - 1.0000331317466504e+06 v[15,23]
   - start_service[15] + start_service[23] >= -999999
 StartServiceRobots[15,24]: - 1.0000331317466504e+06 v[15,24]
   - start_service[15] + start_service[24] >= -999999
 StartServiceRobots[15,25]: - 1.0000331317466504e+06 v[15,25]
   - start_service[15] + start_service[25] >= -999999
 StartServiceRobots[16,1]: - 1.000022879122035e+06 v[16,1]
   + start_service[1] - start_service[16] >= -999999
 StartServiceRobots[16,2]: - 1.000022879122035e+06 v[16,2]
   + start_service[2] - start_service[16] >= -999999
 StartServiceRobots[16,3]: - 1.0000341454712306e+06 v[16,3]
   + start_service[3] - start_service[16] >= -999999
 StartServiceRobots[16,4]: - 1.0000229976847993e+06 v[16,4]
   + start_service[4] - start_service[16] >= -999999
 StartServiceRobots[16,5]: - 1.0000096898391704e+06 v[16,5]
   + start_service[5] - start_service[16] >= -999999
 StartServiceRobots[16,6]: - 999999.01 v[16,6] + start_service[6]
   - start_service[16] >= -999999
 StartServiceRobots[16,7]: - 1.0000345266163895e+06 v[16,7]
   + start_service[7] - start_service[16] >= -999999
 StartServiceRobots[16,8]: - 1.0000341454712306e+06 v[16,8]
   + start_service[8] - start_service[16] >= -999999
 StartServiceRobots[16,9]: - 1.0000229976847993e+06 v[16,9]
   + start_service[9] - start_service[16] >= -999999
 StartServiceRobots[16,10]: - 1.0000096898391704e+06 v[16,10]
   + start_service[10] - start_service[16] >= -999999
 StartServiceRobots[16,11]: - 999999.01 v[16,11] + start_service[11]
   - start_service[16] >= -999999
 StartServiceRobots[16,12]: - 1.0000345266163895e+06 v[16,12]
   + start_service[12] - start_service[16] >= -999999
 StartServiceRobots[16,13]: - 1.0000341454712306e+06 v[16,13]
   + start_service[13] - start_service[16] >= -999999
 StartServiceRobots[16,14]: - 1.0000229976847993e+06 v[16,14]
   + start_service[14] - start_service[16] >= -999999
 StartServiceRobots[16,15]: - 1.0000096898391704e+06 v[16,15]
   + start_service[15] - start_service[16] >= -999999
 StartServiceRobots[16,16]: - 999999.01 v[16,16] >= -999999
 StartServiceRobots[16,17]: - 1.0000345266163895e+06 v[16,17]
   - start_service[16] + start_service[17] >= -999999
 StartServiceRobots[16,18]: - 1.0000341454712306e+06 v[16,18]
   - start_service[16] + start_service[18] >= -999999
 StartServiceRobots[16,19]: - 1.0000229976847993e+06 v[16,19]
   - start_service[16] + start_service[19] >= -999999
 StartServiceRobots[16,20]: - 1.0000096898391704e+06 v[16,20]
   - start_service[16] + start_service[20] >= -999999
 StartServiceRobots[16,21]: - 999999.01 v[16,21] - start_service[16]
   + start_service[21] >= -999999
 StartServiceRobots[16,22]: - 1.0000345266163895e+06 v[16,22]
   - start_service[16] + start_service[22] >= -999999
 StartServiceRobots[16,23]: - 1.000022879122035e+06 v[16,23]
   - start_service[16] + start_service[23] >= -999999
 StartServiceRobots[16,24]: - 1.000022879122035e+06 v[16,24]
   - start_service[16] + start_service[24] >= -999999
 StartServiceRobots[16,25]: - 1.000022879122035e+06 v[16,25]
   - start_service[16] + start_service[25] >= -999999
 StartServiceRobots[17,1]: - 1.0000241592465249e+06 v[17,1]
   + start_service[1] - start_service[17] >= -999999
 StartServiceRobots[17,2]: - 1.0000241592465249e+06 v[17,2]
   + start_service[2] - start_service[17] >= -999999
 StartServiceRobots[17,3]: - 1.0000688935928658e+06 v[17,3]
   + start_service[3] - start_service[17] >= -999999
 StartServiceRobots[17,4]: - 1.0000305908017206e+06 v[17,4]
   + start_service[4] - start_service[17] >= -999999
 StartServiceRobots[17,5]: - 1.0000402331808718e+06 v[17,5]
   + start_service[5] - start_service[17] >= -999999
 StartServiceRobots[17,6]: - 1.0000345266163895e+06 v[17,6]
   + start_service[6] - start_service[17] >= -999999
 StartServiceRobots[17,7]: - 999999.01 v[17,7] + start_service[7]
   - start_service[17] >= -999999
 StartServiceRobots[17,8]: - 1.0000688935928658e+06 v[17,8]
   + start_service[8] - start_service[17] >= -999999
 StartServiceRobots[17,9]: - 1.0000305908017206e+06 v[17,9]
   + start_service[9] - start_service[17] >= -999999
 StartServiceRobots[17,10]: - 1.0000402331808718e+06 v[17,10]
   + start_service[10] - start_service[17] >= -999999
 StartServiceRobots[17,11]: - 1.0000345266163895e+06 v[17,11]
   + start_service[11] - start_service[17] >= -999999
 StartServiceRobots[17,12]: - 999999.01 v[17,12] + start_service[12]
   - start_service[17] >= -999999
 StartServiceRobots[17,13]: - 1.0000688935928658e+06 v[17,13]
   + start_service[13] - start_service[17] >= -999999
 StartServiceRobots[17,14]: - 1.0000305908017206e+06 v[17,14]
   + start_service[14] - start_service[17] >= -999999
 StartServiceRobots[17,15]: - 1.0000402331808718e+06 v[17,15]
   + start_service[15] - start_service[17] >= -999999
 StartServiceRobots[17,16]: - 1.0000345266163895e+06 v[17,16]
   + start_service[16] - start_service[17] >= -999999
 StartServiceRobots[17,17]: - 999999.01 v[17,17] >= -999999
 StartServiceRobots[17,18]: - 1.0000688935928658e+06 v[17,18]
   - start_service[17] + start_service[18] >= -999999
 StartServiceRobots[17,19]: - 1.0000305908017206e+06 v[17,19]
   - start_service[17] + start_service[19] >= -999999
 StartServiceRobots[17,20]: - 1.0000402331808718e+06 v[17,20]
   - start_service[17] + start_service[20] >= -999999
 StartServiceRobots[17,21]: - 1.0000345266163895e+06 v[17,21]
   - start_service[17] + start_service[21] >= -999999
 StartServiceRobots[17,22]: - 999999.01 v[17,22] - start_service[17]
   + start_service[22] >= -999999
 StartServiceRobots[17,23]: - 1.0000241592465249e+06 v[17,23]
   - start_service[17] + start_service[23] >= -999999
 StartServiceRobots[17,24]: - 1.0000241592465249e+06 v[17,24]
   - start_service[17] + start_service[24] >= -999999
 StartServiceRobots[17,25]: - 1.0000241592465249e+06 v[17,25]
   - start_service[17] + start_service[25] >= -999999
 StartServiceRobots[18,1]: - 1.0000563369351183e+06 v[18,1]
   + start_service[1] - start_service[18] >= -999999
 StartServiceRobots[18,2]: - 1.0000563369351183e+06 v[18,2]
   + start_service[2] - start_service[18] >= -999999
 StartServiceRobots[18,3]: - 999999.01 v[18,3] + start_service[3]
   - start_service[18] >= -999999
 StartServiceRobots[18,4]: - 1.0000456469522376e+06 v[18,4]
   + start_service[4] - start_service[18] >= -999999
 StartServiceRobots[18,5]: - 1.000028055364666e+06 v[18,5]
   + start_service[5] - start_service[18] >= -999999
 StartServiceRobots[18,6]: - 1.0000341454712306e+06 v[18,6]
   + start_service[6] - start_service[18] >= -999999
 StartServiceRobots[18,7]: - 1.0000688935928658e+06 v[18,7]
   + start_service[7] - start_service[18] >= -999999
 StartServiceRobots[18,8]: - 999999.01 v[18,8] + start_service[8]
   - start_service[18] >= -999999
 StartServiceRobots[18,9]: - 1.0000456469522376e+06 v[18,9]
   + start_service[9] - start_service[18] >= -999999
 StartServiceRobots[18,10]: - 1.000028055364666e+06 v[18,10]
   + start_service[10] - start_service[18] >= -999999
 StartServiceRobots[18,11]: - 1.0000341454712306e+06 v[18,11]
   + start_service[11] - start_service[18] >= -999999
 StartServiceRobots[18,12]: - 1.0000688935928658e+06 v[18,12]
   + start_service[12] - start_service[18] >= -999999
 StartServiceRobots[18,13]: - 999999.01 v[18,13] + start_service[13]
   - start_service[18] >= -999999
 StartServiceRobots[18,14]: - 1.0000456469522376e+06 v[18,14]
   + start_service[14] - start_service[18] >= -999999
 StartServiceRobots[18,15]: - 1.000028055364666e+06 v[18,15]
   + start_service[15] - start_service[18] >= -999999
 StartServiceRobots[18,16]: - 1.0000341454712306e+06 v[18,16]
   + start_service[16] - start_service[18] >= -999999
 StartServiceRobots[18,17]: - 1.0000688935928658e+06 v[18,17]
   + start_service[17] - start_service[18] >= -999999
 StartServiceRobots[18,18]: - 999999.01 v[18,18] >= -999999
 StartServiceRobots[18,19]: - 1.0000456469522376e+06 v[18,19]
   - start_service[18] + start_service[19] >= -999999
 StartServiceRobots[18,20]: - 1.000028055364666e+06 v[18,20]
   - start_service[18] + start_service[20] >= -999999
 StartServiceRobots[18,21]: - 1.0000341454712306e+06 v[18,21]
   - start_service[18] + start_service[21] >= -999999
 StartServiceRobots[18,22]: - 1.0000688935928658e+06 v[18,22]
   - start_service[18] + start_service[22] >= -999999
 StartServiceRobots[18,23]: - 1.0000563369351183e+06 v[18,23]
   - start_service[18] + start_service[23] >= -999999
 StartServiceRobots[18,24]: - 1.0000563369351183e+06 v[18,24]
   - start_service[18] + start_service[24] >= -999999
 StartServiceRobots[18,25]: - 1.0000563369351183e+06 v[18,25]
   - start_service[18] + start_service[25] >= -999999
 StartServiceRobots[19,1]: - 1.0000371105987907e+06 v[19,1]
   + start_service[1] - start_service[19] >= -999999
 StartServiceRobots[19,2]: - 1.0000371105987907e+06 v[19,2]
   + start_service[2] - start_service[19] >= -999999
 StartServiceRobots[19,3]: - 1.0000456469522376e+06 v[19,3]
   + start_service[3] - start_service[19] >= -999999
 StartServiceRobots[19,4]: - 999999.01 v[19,4] + start_service[4]
   - start_service[19] >= -999999
 StartServiceRobots[19,5]: - 1.0000188499797942e+06 v[19,5]
   + start_service[5] - start_service[19] >= -999999
 StartServiceRobots[19,6]: - 1.0000229976847993e+06 v[19,6]
   + start_service[6] - start_service[19] >= -999999
 StartServiceRobots[19,7]: - 1.0000305908017206e+06 v[19,7]
   + start_service[7] - start_service[19] >= -999999
 StartServiceRobots[19,8]: - 1.0000456469522376e+06 v[19,8]
   + start_service[8] - start_service[19] >= -999999
 StartServiceRobots[19,9]: - 999999.01 v[19,9] + start_service[9]
   - start_service[19] >= -999999
 StartServiceRobots[19,10]: - 1.0000188499797942e+06 v[19,10]
   + start_service[10] - start_service[19] >= -999999
 StartServiceRobots[19,11]: - 1.0000229976847993e+06 v[19,11]
   + start_service[11] - start_service[19] >= -999999
 StartServiceRobots[19,12]: - 1.0000305908017206e+06 v[19,12]
   + start_service[12] - start_service[19] >= -999999
 StartServiceRobots[19,13]: - 1.0000456469522376e+06 v[19,13]
   + start_service[13] - start_service[19] >= -999999
 StartServiceRobots[19,14]: - 999999.01 v[19,14] + start_service[14]
   - start_service[19] >= -999999
 StartServiceRobots[19,15]: - 1.0000188499797942e+06 v[19,15]
   + start_service[15] - start_service[19] >= -999999
 StartServiceRobots[19,16]: - 1.0000229976847993e+06 v[19,16]
   + start_service[16] - start_service[19] >= -999999
 StartServiceRobots[19,17]: - 1.0000305908017206e+06 v[19,17]
   + start_service[17] - start_service[19] >= -999999
 StartServiceRobots[19,18]: - 1.0000456469522376e+06 v[19,18]
   + start_service[18] - start_service[19] >= -999999
 StartServiceRobots[19,19]: - 999999.01 v[19,19] >= -999999
 StartServiceRobots[19,20]: - 1.0000188499797942e+06 v[19,20]
   - start_service[19] + start_service[20] >= -999999
 StartServiceRobots[19,21]: - 1.0000229976847993e+06 v[19,21]
   - start_service[19] + start_service[21] >= -999999
 StartServiceRobots[19,22]: - 1.0000305908017206e+06 v[19,22]
   - start_service[19] + start_service[22] >= -999999
 StartServiceRobots[19,23]: - 1.0000371105987907e+06 v[19,23]
   - start_service[19] + start_service[23] >= -999999
 StartServiceRobots[19,24]: - 1.0000371105987907e+06 v[19,24]
   - start_service[19] + start_service[24] >= -999999
 StartServiceRobots[19,25]: - 1.0000371105987907e+06 v[19,25]
   - start_service[19] + start_service[25] >= -999999
 StartServiceRobots[20,1]: - 1.0000331317466504e+06 v[20,1]
   + start_service[1] - start_service[20] >= -999999
 StartServiceRobots[20,2]: - 1.0000331317466504e+06 v[20,2]
   + start_service[2] - start_service[20] >= -999999
 StartServiceRobots[20,3]: - 1.000028055364666e+06 v[20,3]
   + start_service[3] - start_service[20] >= -999999
 StartServiceRobots[20,4]: - 1.0000188499797942e+06 v[20,4]
   + start_service[4] - start_service[20] >= -999999
 StartServiceRobots[20,5]: - 999999.01 v[20,5] + start_service[5]
   - start_service[20] >= -999999
 StartServiceRobots[20,6]: - 1.0000096898391704e+06 v[20,6]
   + start_service[6] - start_service[20] >= -999999
 StartServiceRobots[20,7]: - 1.0000402331808718e+06 v[20,7]
   + start_service[7] - start_service[20] >= -999999
 StartServiceRobots[20,8]: - 1.000028055364666e+06 v[20,8]
   + start_service[8] - start_service[20] >= -999999
 StartServiceRobots[20,9]: - 1.0000188499797942e+06 v[20,9]
   + start_service[9] - start_service[20] >= -999999
 StartServiceRobots[20,10]: - 999999.01 v[20,10] + start_service[10]
   - start_service[20] >= -999999
 StartServiceRobots[20,11]: - 1.0000096898391704e+06 v[20,11]
   + start_service[11] - start_service[20] >= -999999
 StartServiceRobots[20,12]: - 1.0000402331808718e+06 v[20,12]
   + start_service[12] - start_service[20] >= -999999
 StartServiceRobots[20,13]: - 1.000028055364666e+06 v[20,13]
   + start_service[13] - start_service[20] >= -999999
 StartServiceRobots[20,14]: - 1.0000188499797942e+06 v[20,14]
   + start_service[14] - start_service[20] >= -999999
 StartServiceRobots[20,15]: - 999999.01 v[20,15] + start_service[15]
   - start_service[20] >= -999999
 StartServiceRobots[20,16]: - 1.0000096898391704e+06 v[20,16]
   + start_service[16] - start_service[20] >= -999999
 StartServiceRobots[20,17]: - 1.0000402331808718e+06 v[20,17]
   + start_service[17] - start_service[20] >= -999999
 StartServiceRobots[20,18]: - 1.000028055364666e+06 v[20,18]
   + start_service[18] - start_service[20] >= -999999
 StartServiceRobots[20,19]: - 1.0000188499797942e+06 v[20,19]
   + start_service[19] - start_service[20] >= -999999
 StartServiceRobots[20,20]: - 999999.01 v[20,20] >= -999999
 StartServiceRobots[20,21]: - 1.0000096898391704e+06 v[20,21]
   - start_service[20] + start_service[21] >= -999999
 StartServiceRobots[20,22]: - 1.0000402331808718e+06 v[20,22]
   - start_service[20] + start_service[22] >= -999999
 StartServiceRobots[20,23]: - 1.0000331317466504e+06 v[20,23]
   - start_service[20] + start_service[23] >= -999999
 StartServiceRobots[20,24]: - 1.0000331317466504e+06 v[20,24]
   - start_service[20] + start_service[24] >= -999999
 StartServiceRobots[20,25]: - 1.0000331317466504e+06 v[20,25]
   - start_service[20] + start_service[25] >= -999999
 StartServiceRobots[21,1]: - 1.000022879122035e+06 v[21,1]
   + start_service[1] - start_service[21] >= -999999
 StartServiceRobots[21,2]: - 1.000022879122035e+06 v[21,2]
   + start_service[2] - start_service[21] >= -999999
 StartServiceRobots[21,3]: - 1.0000341454712306e+06 v[21,3]
   + start_service[3] - start_service[21] >= -999999
 StartServiceRobots[21,4]: - 1.0000229976847993e+06 v[21,4]
   + start_service[4] - start_service[21] >= -999999
 StartServiceRobots[21,5]: - 1.0000096898391704e+06 v[21,5]
   + start_service[5] - start_service[21] >= -999999
 StartServiceRobots[21,6]: - 999999.01 v[21,6] + start_service[6]
   - start_service[21] >= -999999
 StartServiceRobots[21,7]: - 1.0000345266163895e+06 v[21,7]
   + start_service[7] - start_service[21] >= -999999
 StartServiceRobots[21,8]: - 1.0000341454712306e+06 v[21,8]
   + start_service[8] - start_service[21] >= -999999
 StartServiceRobots[21,9]: - 1.0000229976847993e+06 v[21,9]
   + start_service[9] - start_service[21] >= -999999
 StartServiceRobots[21,10]: - 1.0000096898391704e+06 v[21,10]
   + start_service[10] - start_service[21] >= -999999
 StartServiceRobots[21,11]: - 999999.01 v[21,11] + start_service[11]
   - start_service[21] >= -999999
 StartServiceRobots[21,12]: - 1.0000345266163895e+06 v[21,12]
   + start_service[12] - start_service[21] >= -999999
 StartServiceRobots[21,13]: - 1.0000341454712306e+06 v[21,13]
   + start_service[13] - start_service[21] >= -999999
 StartServiceRobots[21,14]: - 1.0000229976847993e+06 v[21,14]
   + start_service[14] - start_service[21] >= -999999
 StartServiceRobots[21,15]: - 1.0000096898391704e+06 v[21,15]
   + start_service[15] - start_service[21] >= -999999
 StartServiceRobots[21,16]: - 999999.01 v[21,16] + start_service[16]
   - start_service[21] >= -999999
 StartServiceRobots[21,17]: - 1.0000345266163895e+06 v[21,17]
   + start_service[17] - start_service[21] >= -999999
 StartServiceRobots[21,18]: - 1.0000341454712306e+06 v[21,18]
   + start_service[18] - start_service[21] >= -999999
 StartServiceRobots[21,19]: - 1.0000229976847993e+06 v[21,19]
   + start_service[19] - start_service[21] >= -999999
 StartServiceRobots[21,20]: - 1.0000096898391704e+06 v[21,20]
   + start_service[20] - start_service[21] >= -999999
 StartServiceRobots[21,21]: - 999999.01 v[21,21] >= -999999
 StartServiceRobots[21,22]: - 1.0000345266163895e+06 v[21,22]
   - start_service[21] + start_service[22] >= -999999
 StartServiceRobots[21,23]: - 1.000022879122035e+06 v[21,23]
   - start_service[21] + start_service[23] >= -999999
 StartServiceRobots[21,24]: - 1.000022879122035e+06 v[21,24]
   - start_service[21] + start_service[24] >= -999999
 StartServiceRobots[21,25]: - 1.000022879122035e+06 v[21,25]
   - start_service[21] + start_service[25] >= -999999
 StartServiceRobots[22,1]: - 1.0000241592465249e+06 v[22,1]
   + start_service[1] - start_service[22] >= -999999
 StartServiceRobots[22,2]: - 1.0000241592465249e+06 v[22,2]
   + start_service[2] - start_service[22] >= -999999
 StartServiceRobots[22,3]: - 1.0000688935928658e+06 v[22,3]
   + start_service[3] - start_service[22] >= -999999
 StartServiceRobots[22,4]: - 1.0000305908017206e+06 v[22,4]
   + start_service[4] - start_service[22] >= -999999
 StartServiceRobots[22,5]: - 1.0000402331808718e+06 v[22,5]
   + start_service[5] - start_service[22] >= -999999
 StartServiceRobots[22,6]: - 1.0000345266163895e+06 v[22,6]
   + start_service[6] - start_service[22] >= -999999
 StartServiceRobots[22,7]: - 999999.01 v[22,7] + start_service[7]
   - start_service[22] >= -999999
 StartServiceRobots[22,8]: - 1.0000688935928658e+06 v[22,8]
   + start_service[8] - start_service[22] >= -999999
 StartServiceRobots[22,9]: - 1.0000305908017206e+06 v[22,9]
   + start_service[9] - start_service[22] >= -999999
 StartServiceRobots[22,10]: - 1.0000402331808718e+06 v[22,10]
   + start_service[10] - start_service[22] >= -999999
 StartServiceRobots[22,11]: - 1.0000345266163895e+06 v[22,11]
   + start_service[11] - start_service[22] >= -999999
 StartServiceRobots[22,12]: - 999999.01 v[22,12] + start_service[12]
   - start_service[22] >= -999999
 StartServiceRobots[22,13]: - 1.0000688935928658e+06 v[22,13]
   + start_service[13] - start_service[22] >= -999999
 StartServiceRobots[22,14]: - 1.0000305908017206e+06 v[22,14]
   + start_service[14] - start_service[22] >= -999999
 StartServiceRobots[22,15]: - 1.0000402331808718e+06 v[22,15]
   + start_service[15] - start_service[22] >= -999999
 StartServiceRobots[22,16]: - 1.0000345266163895e+06 v[22,16]
   + start_service[16] - start_service[22] >= -999999
 StartServiceRobots[22,17]: - 999999.01 v[22,17] + start_service[17]
   - start_service[22] >= -999999
 StartServiceRobots[22,18]: - 1.0000688935928658e+06 v[22,18]
   + start_service[18] - start_service[22] >= -999999
 StartServiceRobots[22,19]: - 1.0000305908017206e+06 v[22,19]
   + start_service[19] - start_service[22] >= -999999
 StartServiceRobots[22,20]: - 1.0000402331808718e+06 v[22,20]
   + start_service[20] - start_service[22] >= -999999
 StartServiceRobots[22,21]: - 1.0000345266163895e+06 v[22,21]
   + start_service[21] - start_service[22] >= -999999
 StartServiceRobots[22,22]: - 999999.01 v[22,22] >= -999999
 StartServiceRobots[22,23]: - 1.0000241592465249e+06 v[22,23]
   - start_service[22] + start_service[23] >= -999999
 StartServiceRobots[22,24]: - 1.0000241592465249e+06 v[22,24]
   - start_service[22] + start_service[24] >= -999999
 StartServiceRobots[22,25]: - 1.0000241592465249e+06 v[22,25]
   - start_service[22] + start_service[25] >= -999999
 StartServiceRobots[23,1]: - 999999.01 v[23,1] + start_service[1]
   - start_service[23] >= -999999
 StartServiceRobots[23,2]: - 999999.01 v[23,2] + start_service[2]
   - start_service[23] >= -999999
 StartServiceRobots[23,3]: - 1.0000563369351183e+06 v[23,3]
   + start_service[3] - start_service[23] >= -999999
 StartServiceRobots[23,4]: - 1.0000371105987907e+06 v[23,4]
   + start_service[4] - start_service[23] >= -999999
 StartServiceRobots[23,5]: - 1.0000331317466504e+06 v[23,5]
   + start_service[5] - start_service[23] >= -999999
 StartServiceRobots[23,6]: - 1.000022879122035e+06 v[23,6]
   + start_service[6] - start_service[23] >= -999999
 StartServiceRobots[23,7]: - 1.0000241592465249e+06 v[23,7]
   + start_service[7] - start_service[23] >= -999999
 StartServiceRobots[23,8]: - 1.0000563369351183e+06 v[23,8]
   + start_service[8] - start_service[23] >= -999999
 StartServiceRobots[23,9]: - 1.0000371105987907e+06 v[23,9]
   + start_service[9] - start_service[23] >= -999999
 StartServiceRobots[23,10]: - 1.0000331317466504e+06 v[23,10]
   + start_service[10] - start_service[23] >= -999999
 StartServiceRobots[23,11]: - 1.000022879122035e+06 v[23,11]
   + start_service[11] - start_service[23] >= -999999
 StartServiceRobots[23,12]: - 1.0000241592465249e+06 v[23,12]
   + start_service[12] - start_service[23] >= -999999
 StartServiceRobots[23,13]: - 1.0000563369351183e+06 v[23,13]
   + start_service[13] - start_service[23] >= -999999
 StartServiceRobots[23,14]: - 1.0000371105987907e+06 v[23,14]
   + start_service[14] - start_service[23] >= -999999
 StartServiceRobots[23,15]: - 1.0000331317466504e+06 v[23,15]
   + start_service[15] - start_service[23] >= -999999
 StartServiceRobots[23,16]: - 1.000022879122035e+06 v[23,16]
   + start_service[16] - start_service[23] >= -999999
 StartServiceRobots[23,17]: - 1.0000241592465249e+06 v[23,17]
   + start_service[17] - start_service[23] >= -999999
 StartServiceRobots[23,18]: - 1.0000563369351183e+06 v[23,18]
   + start_service[18] - start_service[23] >= -999999
 StartServiceRobots[23,19]: - 1.0000371105987907e+06 v[23,19]
   + start_service[19] - start_service[23] >= -999999
 StartServiceRobots[23,20]: - 1.0000331317466504e+06 v[23,20]
   + start_service[20] - start_service[23] >= -999999
 StartServiceRobots[23,21]: - 1.000022879122035e+06 v[23,21]
   + start_service[21] - start_service[23] >= -999999
 StartServiceRobots[23,22]: - 1.0000241592465249e+06 v[23,22]
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
 StartServiceRobots[24,3]: - 1.0000563369351183e+06 v[24,3]
   + start_service[3] - start_service[24] >= -999999
 StartServiceRobots[24,4]: - 1.0000371105987907e+06 v[24,4]
   + start_service[4] - start_service[24] >= -999999
 StartServiceRobots[24,5]: - 1.0000331317466504e+06 v[24,5]
   + start_service[5] - start_service[24] >= -999999
 StartServiceRobots[24,6]: - 1.000022879122035e+06 v[24,6]
   + start_service[6] - start_service[24] >= -999999
 StartServiceRobots[24,7]: - 1.0000241592465249e+06 v[24,7]
   + start_service[7] - start_service[24] >= -999999
 StartServiceRobots[24,8]: - 1.0000563369351183e+06 v[24,8]
   + start_service[8] - start_service[24] >= -999999
 StartServiceRobots[24,9]: - 1.0000371105987907e+06 v[24,9]
   + start_service[9] - start_service[24] >= -999999
 StartServiceRobots[24,10]: - 1.0000331317466504e+06 v[24,10]
   + start_service[10] - start_service[24] >= -999999
 StartServiceRobots[24,11]: - 1.000022879122035e+06 v[24,11]
   + start_service[11] - start_service[24] >= -999999
 StartServiceRobots[24,12]: - 1.0000241592465249e+06 v[24,12]
   + start_service[12] - start_service[24] >= -999999
 StartServiceRobots[24,13]: - 1.0000563369351183e+06 v[24,13]
   + start_service[13] - start_service[24] >= -999999
 StartServiceRobots[24,14]: - 1.0000371105987907e+06 v[24,14]
   + start_service[14] - start_service[24] >= -999999
 StartServiceRobots[24,15]: - 1.0000331317466504e+06 v[24,15]
   + start_service[15] - start_service[24] >= -999999
 StartServiceRobots[24,16]: - 1.000022879122035e+06 v[24,16]
   + start_service[16] - start_service[24] >= -999999
 StartServiceRobots[24,17]: - 1.0000241592465249e+06 v[24,17]
   + start_service[17] - start_service[24] >= -999999
 StartServiceRobots[24,18]: - 1.0000563369351183e+06 v[24,18]
   + start_service[18] - start_service[24] >= -999999
 StartServiceRobots[24,19]: - 1.0000371105987907e+06 v[24,19]
   + start_service[19] - start_service[24] >= -999999
 StartServiceRobots[24,20]: - 1.0000331317466504e+06 v[24,20]
   + start_service[20] - start_service[24] >= -999999
 StartServiceRobots[24,21]: - 1.000022879122035e+06 v[24,21]
   + start_service[21] - start_service[24] >= -999999
 StartServiceRobots[24,22]: - 1.0000241592465249e+06 v[24,22]
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
 SettingServiceTime[1,3,13]: - 290 y[3,13,1] + service_time[3,13,1] <= 0
 SettingServiceTime[1,3,18]: - 290 y[3,18,1] + service_time[3,18,1] <= 0
 SettingServiceTime[1,4,14]: - 380 y[4,14,1] + service_time[4,14,1] <= 0
 SettingServiceTime[1,4,19]: - 380 y[4,19,1] + service_time[4,19,1] <= 0
 SettingServiceTime[1,5,15]: - 470 y[5,15,1] + service_time[5,15,1] <= 0
 SettingServiceTime[1,5,20]: - 470 y[5,20,1] + service_time[5,20,1] <= 0
 SettingServiceTime[1,6,16]: - 250 y[6,16,1] + service_time[6,16,1] <= 0
 SettingServiceTime[1,6,21]: - 250 y[6,21,1] + service_time[6,21,1] <= 0
 SettingServiceTime[1,7,17]: - 450 y[7,17,1] + service_time[7,17,1] <= 0
 SettingServiceTime[1,7,22]: - 450 y[7,22,1] + service_time[7,22,1] <= 0
 SettingServiceTime[1,8,13]: - 290 y[8,13,1] + service_time[8,13,1] <= 0
 SettingServiceTime[1,8,18]: - 290 y[8,18,1] + service_time[8,18,1] <= 0
 SettingServiceTime[1,9,14]: - 380 y[9,14,1] + service_time[9,14,1] <= 0
 SettingServiceTime[1,9,19]: - 380 y[9,19,1] + service_time[9,19,1] <= 0
 SettingServiceTime[1,10,15]: - 470 y[10,15,1] + service_time[10,15,1]
   <= 0
 SettingServiceTime[1,10,20]: - 470 y[10,20,1] + service_time[10,20,1]
   <= 0
 SettingServiceTime[1,11,16]: - 250 y[11,16,1] + service_time[11,16,1]
   <= 0
 SettingServiceTime[1,11,21]: - 250 y[11,21,1] + service_time[11,21,1]
   <= 0
 SettingServiceTime[1,12,17]: - 450 y[12,17,1] + service_time[12,17,1]
   <= 0
 SettingServiceTime[1,12,22]: - 450 y[12,22,1] + service_time[12,22,1]
   <= 0
 SettingServiceTime[2,3,13]: - 145 y[3,13,2] + service_time[3,13,2] <= 0
 SettingServiceTime[2,3,18]: - 145 y[3,18,2] + service_time[3,18,2] <= 0
 SettingServiceTime[2,4,14]: - 190 y[4,14,2] + service_time[4,14,2] <= 0
 SettingServiceTime[2,4,19]: - 190 y[4,19,2] + service_time[4,19,2] <= 0
 SettingServiceTime[2,5,15]: - 235 y[5,15,2] + service_time[5,15,2] <= 0
 SettingServiceTime[2,5,20]: - 235 y[5,20,2] + service_time[5,20,2] <= 0
 SettingServiceTime[2,6,16]: - 125 y[6,16,2] + service_time[6,16,2] <= 0
 SettingServiceTime[2,6,21]: - 125 y[6,21,2] + service_time[6,21,2] <= 0
 SettingServiceTime[2,7,17]: - 225 y[7,17,2] + service_time[7,17,2] <= 0
 SettingServiceTime[2,7,22]: - 225 y[7,22,2] + service_time[7,22,2] <= 0
 SettingServiceTime[2,8,13]: - 145 y[8,13,2] + service_time[8,13,2] <= 0
 SettingServiceTime[2,8,18]: - 145 y[8,18,2] + service_time[8,18,2] <= 0
 SettingServiceTime[2,9,14]: - 190 y[9,14,2] + service_time[9,14,2] <= 0
 SettingServiceTime[2,9,19]: - 190 y[9,19,2] + service_time[9,19,2] <= 0
 SettingServiceTime[2,10,15]: - 235 y[10,15,2] + service_time[10,15,2]
   <= 0
 SettingServiceTime[2,10,20]: - 235 y[10,20,2] + service_time[10,20,2]
   <= 0
 SettingServiceTime[2,11,16]: - 125 y[11,16,2] + service_time[11,16,2]
   <= 0
 SettingServiceTime[2,11,21]: - 125 y[11,21,2] + service_time[11,21,2]
   <= 0
 SettingServiceTime[2,12,17]: - 225 y[12,17,2] + service_time[12,17,2]
   <= 0
 SettingServiceTime[2,12,22]: - 225 y[12,22,2] + service_time[12,22,2]
   <= 0
 CompleteDemandSatisfaction[3]: service_time[3,13,1]
   + 2 service_time[3,13,2] + service_time[3,18,1] + 2 service_time[3,18,2]
   + service_time[8,13,1] + 2 service_time[8,13,2] + service_time[8,18,1]
   + 2 service_time[8,18,2] = 290
 CompleteDemandSatisfaction[4]: service_time[4,14,1]
   + 2 service_time[4,14,2] + service_time[4,19,1] + 2 service_time[4,19,2]
   + service_time[9,14,1] + 2 service_time[9,14,2] + service_time[9,19,1]
   + 2 service_time[9,19,2] = 380
 CompleteDemandSatisfaction[5]: service_time[5,15,1]
   + 2 service_time[5,15,2] + service_time[5,20,1] + 2 service_time[5,20,2]
   + service_time[10,15,1] + 2 service_time[10,15,2]
   + service_time[10,20,1] + 2 service_time[10,20,2] = 470
 CompleteDemandSatisfaction[6]: service_time[6,16,1]
   + 2 service_time[6,16,2] + service_time[6,21,1] + 2 service_time[6,21,2]
   + service_time[11,16,1] + 2 service_time[11,16,2]
   + service_time[11,21,1] + 2 service_time[11,21,2] = 250
 CompleteDemandSatisfaction[7]: service_time[7,17,1]
   + 2 service_time[7,17,2] + service_time[7,22,1] + 2 service_time[7,22,2]
   + service_time[12,17,1] + 2 service_time[12,17,2]
   + service_time[12,22,1] + 2 service_time[12,22,2] = 450
 AverageTimeRobots: - 0.00142857142857143 w[0,1]
   - 0.00142857142857143 w[0,2] - 8.190990731192418 w[0,3]
   - 5.444371255805486 w[0,4] - 4.87596380720833 w[0,5]
   - 3.411303147851021 w[0,6] - 3.594178074992532 w[0,7]
   - 8.190990731192418 w[0,8] - 5.444371255805486 w[0,9]
   - 4.87596380720833 w[0,10] - 3.411303147851021 w[0,11]
   - 3.594178074992532 w[0,12] - 8.190990731192418 w[0,13]
   - 5.444371255805486 w[0,14] - 4.87596380720833 w[0,15]
   - 3.411303147851021 w[0,16] - 3.594178074992532 w[0,17]
   - 8.190990731192418 w[0,18] - 5.444371255805486 w[0,19]
   - 4.87596380720833 w[0,20] - 3.411303147851021 w[0,21]
   - 3.594178074992532 w[0,22] - 0.00142857142857143 w[0,23]
   - 0.00142857142857143 w[0,24] - 0.00142857142857143 w[0,25]
   - 0.00142857142857143 w[1,1] - 0.00142857142857143 w[1,2]
   - 8.190990731192418 w[1,3] - 5.444371255805486 w[1,4]
   - 4.87596380720833 w[1,5] - 3.411303147851021 w[1,6]
   - 3.594178074992532 w[1,7] - 8.190990731192418 w[1,8]
   - 5.444371255805486 w[1,9] - 4.87596380720833 w[1,10]
   - 3.411303147851021 w[1,11] - 3.594178074992532 w[1,12]
   - 8.190990731192418 w[1,13] - 5.444371255805486 w[1,14]
   - 4.87596380720833 w[1,15] - 3.411303147851021 w[1,16]
   - 3.594178074992532 w[1,17] - 8.190990731192418 w[1,18]
   - 5.444371255805486 w[1,19] - 4.87596380720833 w[1,20]
   - 3.411303147851021 w[1,21] - 3.594178074992532 w[1,22]
   - 0.00142857142857143 w[1,23] - 0.00142857142857143 w[1,24]
   - 0.00142857142857143 w[1,25] - 0.00142857142857143 w[2,1]
   - 0.00142857142857143 w[2,2] - 8.190990731192418 w[2,3]
   - 5.444371255805486 w[2,4] - 4.87596380720833 w[2,5]
   - 3.411303147851021 w[2,6] - 3.594178074992532 w[2,7]
   - 8.190990731192418 w[2,8] - 5.444371255805486 w[2,9]
   - 4.87596380720833 w[2,10] - 3.411303147851021 w[2,11]
   - 3.594178074992532 w[2,12] - 8.190990731192418 w[2,13]
   - 5.444371255805486 w[2,14] - 4.87596380720833 w[2,15]
   - 3.411303147851021 w[2,16] - 3.594178074992532 w[2,17]
   - 8.190990731192418 w[2,18] - 5.444371255805486 w[2,19]
   - 4.87596380720833 w[2,20] - 3.411303147851021 w[2,21]
   - 3.594178074992532 w[2,22] - 0.00142857142857143 w[2,23]
   - 0.00142857142857143 w[2,24] - 0.00142857142857143 w[2,25]
   - 8.190990731192418 w[3,1] - 8.190990731192418 w[3,2]
   - 0.00142857142857143 w[3,3] - 6.663850319661864 w[3,4]
   - 4.1507663808658 w[3,5] - 5.020781604374885 w[3,6]
   - 9.984798980823046 w[3,7] - 0.00142857142857143 w[3,8]
   - 6.663850319661864 w[3,9] - 4.1507663808658 w[3,10]
   - 5.020781604374885 w[3,11] - 9.984798980823046 w[3,12]
   - 0.00142857142857143 w[3,13] - 6.663850319661864 w[3,14]
   - 4.1507663808658 w[3,15] - 5.020781604374885 w[3,16]
   - 9.984798980823046 w[3,17] - 0.00142857142857143 w[3,18]
   - 6.663850319661864 w[3,19] - 4.1507663808658 w[3,20]
   - 5.020781604374885 w[3,21] - 9.984798980823046 w[3,22]
   - 8.190990731192418 w[3,23] - 8.190990731192418 w[3,24]
   - 8.190990731192418 w[3,25] - 5.444371255805486 w[4,1]
   - 5.444371255805486 w[4,2] - 6.663850319661864 w[4,3]
   - 0.00142857142857143 w[4,4] - 2.83571139917725 w[4,5]
   - 3.428240685609592 w[4,6] - 4.512971674372231 w[4,7]
   - 6.663850319661864 w[4,8] - 0.00142857142857143 w[4,9]
   - 2.83571139917725 w[4,10] - 3.428240685609592 w[4,11]
   - 4.512971674372231 w[4,12] - 6.663850319661864 w[4,13]
   - 0.00142857142857143 w[4,14] - 2.83571139917725 w[4,15]
   - 3.428240685609592 w[4,16] - 4.512971674372231 w[4,17]
   - 6.663850319661864 w[4,18] - 0.00142857142857143 w[4,19]
   - 2.83571139917725 w[4,20] - 3.428240685609592 w[4,21]
   - 4.512971674372231 w[4,22] - 5.444371255805486 w[4,23]
   - 5.444371255805486 w[4,24] - 5.444371255805486 w[4,25]
   - 4.87596380720833 w[5,1] - 4.87596380720833 w[5,2]
   - 4.1507663808658 w[5,3] - 2.83571139917725 w[5,4]
   - 0.00142857142857143 w[5,5] - 1.5271198814972 w[5,6]
   - 5.890454410264359 w[5,7] - 4.1507663808658 w[5,8]
   - 2.83571139917725 w[5,9] - 0.00142857142857143 w[5,10]
   - 1.5271198814972 w[5,11] - 5.890454410264359 w[5,12]
   - 4.1507663808658 w[5,13] - 2.83571139917725 w[5,14]
   - 0.00142857142857143 w[5,15] - 1.5271198814972 w[5,16]
   - 5.890454410264359 w[5,17] - 4.1507663808658 w[5,18]
   - 2.83571139917725 w[5,19] - 0.00142857142857143 w[5,20]
   - 1.5271198814972 w[5,21] - 5.890454410264359 w[5,22]
   - 4.87596380720833 w[5,23] - 4.87596380720833 w[5,24]
   - 4.87596380720833 w[5,25] - 3.411303147851021 w[6,1]
   - 3.411303147851021 w[6,2] - 5.020781604374885 w[6,3]
   - 3.428240685609592 w[6,4] - 1.5271198814972 w[6,5]
   - 0.00142857142857143 w[6,6] - 5.075230912791034 w[6,7]
   - 5.020781604374885 w[6,8] - 3.428240685609592 w[6,9]
   - 1.5271198814972 w[6,10] - 0.00142857142857143 w[6,11]
   - 5.075230912791034 w[6,12] - 5.020781604374885 w[6,13]
   - 3.428240685609592 w[6,14] - 1.5271198814972 w[6,15]
   - 0.00142857142857143 w[6,16] - 5.075230912791034 w[6,17]
   - 5.020781604374885 w[6,18] - 3.428240685609592 w[6,19]
   - 1.5271198814972 w[6,20] - 0.00142857142857143 w[6,21]
   - 5.075230912791034 w[6,22] - 3.411303147851021 w[6,23]
   - 3.411303147851021 w[6,24] - 3.411303147851021 w[6,25]
   - 3.594178074992532 w[7,1] - 3.594178074992532 w[7,2]
   - 9.984798980823046 w[7,3] - 4.512971674372231 w[7,4]
   - 5.890454410264359 w[7,5] - 5.075230912791034 w[7,6]
   - 0.00142857142857143 w[7,7] - 9.984798980823046 w[7,8]
   - 4.512971674372231 w[7,9] - 5.890454410264359 w[7,10]
   - 5.075230912791034 w[7,11] - 0.00142857142857143 w[7,12]
   - 9.984798980823046 w[7,13] - 4.512971674372231 w[7,14]
   - 5.890454410264359 w[7,15] - 5.075230912791034 w[7,16]
   - 0.00142857142857143 w[7,17] - 9.984798980823046 w[7,18]
   - 4.512971674372231 w[7,19] - 5.890454410264359 w[7,20]
   - 5.075230912791034 w[7,21] - 0.00142857142857143 w[7,22]
   - 3.594178074992532 w[7,23] - 3.594178074992532 w[7,24]
   - 3.594178074992532 w[7,25] - 8.190990731192418 w[8,1]
   - 8.190990731192418 w[8,2] - 0.00142857142857143 w[8,3]
   - 6.663850319661864 w[8,4] - 4.1507663808658 w[8,5]
   - 5.020781604374885 w[8,6] - 9.984798980823046 w[8,7]
   - 0.00142857142857143 w[8,8] - 6.663850319661864 w[8,9]
   - 4.1507663808658 w[8,10] - 5.020781604374885 w[8,11]
   - 9.984798980823046 w[8,12] - 0.00142857142857143 w[8,13]
   - 6.663850319661864 w[8,14] - 4.1507663808658 w[8,15]
   - 5.020781604374885 w[8,16] - 9.984798980823046 w[8,17]
   - 0.00142857142857143 w[8,18] - 6.663850319661864 w[8,19]
   - 4.1507663808658 w[8,20] - 5.020781604374885 w[8,21]
   - 9.984798980823046 w[8,22] - 8.190990731192418 w[8,23]
   - 8.190990731192418 w[8,24] - 8.190990731192418 w[8,25]
   - 5.444371255805486 w[9,1] - 5.444371255805486 w[9,2]
   - 6.663850319661864 w[9,3] - 0.00142857142857143 w[9,4]
   - 2.83571139917725 w[9,5] - 3.428240685609592 w[9,6]
   - 4.512971674372231 w[9,7] - 6.663850319661864 w[9,8]
   - 0.00142857142857143 w[9,9] - 2.83571139917725 w[9,10]
   - 3.428240685609592 w[9,11] - 4.512971674372231 w[9,12]
   - 6.663850319661864 w[9,13] - 0.00142857142857143 w[9,14]
   - 2.83571139917725 w[9,15] - 3.428240685609592 w[9,16]
   - 4.512971674372231 w[9,17] - 6.663850319661864 w[9,18]
   - 0.00142857142857143 w[9,19] - 2.83571139917725 w[9,20]
   - 3.428240685609592 w[9,21] - 4.512971674372231 w[9,22]
   - 5.444371255805486 w[9,23] - 5.444371255805486 w[9,24]
   - 5.444371255805486 w[9,25] - 4.87596380720833 w[10,1]
   - 4.87596380720833 w[10,2] - 4.1507663808658 w[10,3]
   - 2.83571139917725 w[10,4] - 0.00142857142857143 w[10,5]
   - 1.5271198814972 w[10,6] - 5.890454410264359 w[10,7]
   - 4.1507663808658 w[10,8] - 2.83571139917725 w[10,9]
   - 0.00142857142857143 w[10,10] - 1.5271198814972 w[10,11]
   - 5.890454410264359 w[10,12] - 4.1507663808658 w[10,13]
   - 2.83571139917725 w[10,14] - 0.00142857142857143 w[10,15]
   - 1.5271198814972 w[10,16] - 5.890454410264359 w[10,17]
   - 4.1507663808658 w[10,18] - 2.83571139917725 w[10,19]
   - 0.00142857142857143 w[10,20] - 1.5271198814972 w[10,21]
   - 5.890454410264359 w[10,22] - 4.87596380720833 w[10,23]
   - 4.87596380720833 w[10,24] - 4.87596380720833 w[10,25]
   - 3.411303147851021 w[11,1] - 3.411303147851021 w[11,2]
   - 5.020781604374885 w[11,3] - 3.428240685609592 w[11,4]
   - 1.5271198814972 w[11,5] - 0.00142857142857143 w[11,6]
   - 5.075230912791034 w[11,7] - 5.020781604374885 w[11,8]
   - 3.428240685609592 w[11,9] - 1.5271198814972 w[11,10]
   - 0.00142857142857143 w[11,11] - 5.075230912791034 w[11,12]
   - 5.020781604374885 w[11,13] - 3.428240685609592 w[11,14]
   - 1.5271198814972 w[11,15] - 0.00142857142857143 w[11,16]
   - 5.075230912791034 w[11,17] - 5.020781604374885 w[11,18]
   - 3.428240685609592 w[11,19] - 1.5271198814972 w[11,20]
   - 0.00142857142857143 w[11,21] - 5.075230912791034 w[11,22]
   - 3.411303147851021 w[11,23] - 3.411303147851021 w[11,24]
   - 3.411303147851021 w[11,25] - 3.594178074992532 w[12,1]
   - 3.594178074992532 w[12,2] - 9.984798980823046 w[12,3]
   - 4.512971674372231 w[12,4] - 5.890454410264359 w[12,5]
   - 5.075230912791034 w[12,6] - 0.00142857142857143 w[12,7]
   - 9.984798980823046 w[12,8] - 4.512971674372231 w[12,9]
   - 5.890454410264359 w[12,10] - 5.075230912791034 w[12,11]
   - 0.00142857142857143 w[12,12] - 9.984798980823046 w[12,13]
   - 4.512971674372231 w[12,14] - 5.890454410264359 w[12,15]
   - 5.075230912791034 w[12,16] - 0.00142857142857143 w[12,17]
   - 9.984798980823046 w[12,18] - 4.512971674372231 w[12,19]
   - 5.890454410264359 w[12,20] - 5.075230912791034 w[12,21]
   - 0.00142857142857143 w[12,22] - 3.594178074992532 w[12,23]
   - 3.594178074992532 w[12,24] - 3.594178074992532 w[12,25]
   - 8.190990731192418 w[13,1] - 8.190990731192418 w[13,2]
   - 0.00142857142857143 w[13,3] - 6.663850319661864 w[13,4]
   - 4.1507663808658 w[13,5] - 5.020781604374885 w[13,6]
   - 9.984798980823046 w[13,7] - 0.00142857142857143 w[13,8]
   - 6.663850319661864 w[13,9] - 4.1507663808658 w[13,10]
   - 5.020781604374885 w[13,11] - 9.984798980823046 w[13,12]
   - 0.00142857142857143 w[13,13] - 6.663850319661864 w[13,14]
   - 4.1507663808658 w[13,15] - 5.020781604374885 w[13,16]
   - 9.984798980823046 w[13,17] - 0.00142857142857143 w[13,18]
   - 6.663850319661864 w[13,19] - 4.1507663808658 w[13,20]
   - 5.020781604374885 w[13,21] - 9.984798980823046 w[13,22]
   - 8.190990731192418 w[13,23] - 8.190990731192418 w[13,24]
   - 8.190990731192418 w[13,25] - 5.444371255805486 w[14,1]
   - 5.444371255805486 w[14,2] - 6.663850319661864 w[14,3]
   - 0.00142857142857143 w[14,4] - 2.83571139917725 w[14,5]
   - 3.428240685609592 w[14,6] - 4.512971674372231 w[14,7]
   - 6.663850319661864 w[14,8] - 0.00142857142857143 w[14,9]
   - 2.83571139917725 w[14,10] - 3.428240685609592 w[14,11]
   - 4.512971674372231 w[14,12] - 6.663850319661864 w[14,13]
   - 0.00142857142857143 w[14,14] - 2.83571139917725 w[14,15]
   - 3.428240685609592 w[14,16] - 4.512971674372231 w[14,17]
   - 6.663850319661864 w[14,18] - 0.00142857142857143 w[14,19]
   - 2.83571139917725 w[14,20] - 3.428240685609592 w[14,21]
   - 4.512971674372231 w[14,22] - 5.444371255805486 w[14,23]
   - 5.444371255805486 w[14,24] - 5.444371255805486 w[14,25]
   - 4.87596380720833 w[15,1] - 4.87596380720833 w[15,2]
   - 4.1507663808658 w[15,3] - 2.83571139917725 w[15,4]
   - 0.00142857142857143 w[15,5] - 1.5271198814972 w[15,6]
   - 5.890454410264359 w[15,7] - 4.1507663808658 w[15,8]
   - 2.83571139917725 w[15,9] - 0.00142857142857143 w[15,10]
   - 1.5271198814972 w[15,11] - 5.890454410264359 w[15,12]
   - 4.1507663808658 w[15,13] - 2.83571139917725 w[15,14]
   - 0.00142857142857143 w[15,15] - 1.5271198814972 w[15,16]
   - 5.890454410264359 w[15,17] - 4.1507663808658 w[15,18]
   - 2.83571139917725 w[15,19] - 0.00142857142857143 w[15,20]
   - 1.5271198814972 w[15,21] - 5.890454410264359 w[15,22]
   - 4.87596380720833 w[15,23] - 4.87596380720833 w[15,24]
   - 4.87596380720833 w[15,25] - 3.411303147851021 w[16,1]
   - 3.411303147851021 w[16,2] - 5.020781604374885 w[16,3]
   - 3.428240685609592 w[16,4] - 1.5271198814972 w[16,5]
   - 0.00142857142857143 w[16,6] - 5.075230912791034 w[16,7]
   - 5.020781604374885 w[16,8] - 3.428240685609592 w[16,9]
   - 1.5271198814972 w[16,10] - 0.00142857142857143 w[16,11]
   - 5.075230912791034 w[16,12] - 5.020781604374885 w[16,13]
   - 3.428240685609592 w[16,14] - 1.5271198814972 w[16,15]
   - 0.00142857142857143 w[16,16] - 5.075230912791034 w[16,17]
   - 5.020781604374885 w[16,18] - 3.428240685609592 w[16,19]
   - 1.5271198814972 w[16,20] - 0.00142857142857143 w[16,21]
   - 5.075230912791034 w[16,22] - 3.411303147851021 w[16,23]
   - 3.411303147851021 w[16,24] - 3.411303147851021 w[16,25]
   - 3.594178074992532 w[17,1] - 3.594178074992532 w[17,2]
   - 9.984798980823046 w[17,3] - 4.512971674372231 w[17,4]
   - 5.890454410264359 w[17,5] - 5.075230912791034 w[17,6]
   - 0.00142857142857143 w[17,7] - 9.984798980823046 w[17,8]
   - 4.512971674372231 w[17,9] - 5.890454410264359 w[17,10]
   - 5.075230912791034 w[17,11] - 0.00142857142857143 w[17,12]
   - 9.984798980823046 w[17,13] - 4.512971674372231 w[17,14]
   - 5.890454410264359 w[17,15] - 5.075230912791034 w[17,16]
   - 0.00142857142857143 w[17,17] - 9.984798980823046 w[17,18]
   - 4.512971674372231 w[17,19] - 5.890454410264359 w[17,20]
   - 5.075230912791034 w[17,21] - 0.00142857142857143 w[17,22]
   - 3.594178074992532 w[17,23] - 3.594178074992532 w[17,24]
   - 3.594178074992532 w[17,25] - 8.190990731192418 w[18,1]
   - 8.190990731192418 w[18,2] - 0.00142857142857143 w[18,3]
   - 6.663850319661864 w[18,4] - 4.1507663808658 w[18,5]
   - 5.020781604374885 w[18,6] - 9.984798980823046 w[18,7]
   - 0.00142857142857143 w[18,8] - 6.663850319661864 w[18,9]
   - 4.1507663808658 w[18,10] - 5.020781604374885 w[18,11]
   - 9.984798980823046 w[18,12] - 0.00142857142857143 w[18,13]
   - 6.663850319661864 w[18,14] - 4.1507663808658 w[18,15]
   - 5.020781604374885 w[18,16] - 9.984798980823046 w[18,17]
   - 0.00142857142857143 w[18,18] - 6.663850319661864 w[18,19]
   - 4.1507663808658 w[18,20] - 5.020781604374885 w[18,21]
   - 9.984798980823046 w[18,22] - 8.190990731192418 w[18,23]
   - 8.190990731192418 w[18,24] - 8.190990731192418 w[18,25]
   - 5.444371255805486 w[19,1] - 5.444371255805486 w[19,2]
   - 6.663850319661864 w[19,3] - 0.00142857142857143 w[19,4]
   - 2.83571139917725 w[19,5] - 3.428240685609592 w[19,6]
   - 4.512971674372231 w[19,7] - 6.663850319661864 w[19,8]
   - 0.00142857142857143 w[19,9] - 2.83571139917725 w[19,10]
   - 3.428240685609592 w[19,11] - 4.512971674372231 w[19,12]
   - 6.663850319661864 w[19,13] - 0.00142857142857143 w[19,14]
   - 2.83571139917725 w[19,15] - 3.428240685609592 w[19,16]
   - 4.512971674372231 w[19,17] - 6.663850319661864 w[19,18]
   - 0.00142857142857143 w[19,19] - 2.83571139917725 w[19,20]
   - 3.428240685609592 w[19,21] - 4.512971674372231 w[19,22]
   - 5.444371255805486 w[19,23] - 5.444371255805486 w[19,24]
   - 5.444371255805486 w[19,25] - 4.87596380720833 w[20,1]
   - 4.87596380720833 w[20,2] - 4.1507663808658 w[20,3]
   - 2.83571139917725 w[20,4] - 0.00142857142857143 w[20,5]
   - 1.5271198814972 w[20,6] - 5.890454410264359 w[20,7]
   - 4.1507663808658 w[20,8] - 2.83571139917725 w[20,9]
   - 0.00142857142857143 w[20,10] - 1.5271198814972 w[20,11]
   - 5.890454410264359 w[20,12] - 4.1507663808658 w[20,13]
   - 2.83571139917725 w[20,14] - 0.00142857142857143 w[20,15]
   - 1.5271198814972 w[20,16] - 5.890454410264359 w[20,17]
   - 4.1507663808658 w[20,18] - 2.83571139917725 w[20,19]
   - 0.00142857142857143 w[20,20] - 1.5271198814972 w[20,21]
   - 5.890454410264359 w[20,22] - 4.87596380720833 w[20,23]
   - 4.87596380720833 w[20,24] - 4.87596380720833 w[20,25]
   - 3.411303147851021 w[21,1] - 3.411303147851021 w[21,2]
   - 5.020781604374885 w[21,3] - 3.428240685609592 w[21,4]
   - 1.5271198814972 w[21,5] - 0.00142857142857143 w[21,6]
   - 5.075230912791034 w[21,7] - 5.020781604374885 w[21,8]
   - 3.428240685609592 w[21,9] - 1.5271198814972 w[21,10]
   - 0.00142857142857143 w[21,11] - 5.075230912791034 w[21,12]
   - 5.020781604374885 w[21,13] - 3.428240685609592 w[21,14]
   - 1.5271198814972 w[21,15] - 0.00142857142857143 w[21,16]
   - 5.075230912791034 w[21,17] - 5.020781604374885 w[21,18]
   - 3.428240685609592 w[21,19] - 1.5271198814972 w[21,20]
   - 0.00142857142857143 w[21,21] - 5.075230912791034 w[21,22]
   - 3.411303147851021 w[21,23] - 3.411303147851021 w[21,24]
   - 3.411303147851021 w[21,25] - 3.594178074992532 w[22,1]
   - 3.594178074992532 w[22,2] - 9.984798980823046 w[22,3]
   - 4.512971674372231 w[22,4] - 5.890454410264359 w[22,5]
   - 5.075230912791034 w[22,6] - 0.00142857142857143 w[22,7]
   - 9.984798980823046 w[22,8] - 4.512971674372231 w[22,9]
   - 5.890454410264359 w[22,10] - 5.075230912791034 w[22,11]
   - 0.00142857142857143 w[22,12] - 9.984798980823046 w[22,13]
   - 4.512971674372231 w[22,14] - 5.890454410264359 w[22,15]
   - 5.075230912791034 w[22,16] - 0.00142857142857143 w[22,17]
   - 9.984798980823046 w[22,18] - 4.512971674372231 w[22,19]
   - 5.890454410264359 w[22,20] - 5.075230912791034 w[22,21]
   - 0.00142857142857143 w[22,22] - 3.594178074992532 w[22,23]
   - 3.594178074992532 w[22,24] - 3.594178074992532 w[22,25]
   - 0.00142857142857143 w[23,1] - 0.00142857142857143 w[23,2]
   - 8.190990731192418 w[23,3] - 5.444371255805486 w[23,4]
   - 4.87596380720833 w[23,5] - 3.411303147851021 w[23,6]
   - 3.594178074992532 w[23,7] - 8.190990731192418 w[23,8]
   - 5.444371255805486 w[23,9] - 4.87596380720833 w[23,10]
   - 3.411303147851021 w[23,11] - 3.594178074992532 w[23,12]
   - 8.190990731192418 w[23,13] - 5.444371255805486 w[23,14]
   - 4.87596380720833 w[23,15] - 3.411303147851021 w[23,16]
   - 3.594178074992532 w[23,17] - 8.190990731192418 w[23,18]
   - 5.444371255805486 w[23,19] - 4.87596380720833 w[23,20]
   - 3.411303147851021 w[23,21] - 3.594178074992532 w[23,22]
   - 0.00142857142857143 w[23,23] - 0.00142857142857143 w[23,24]
   - 0.00142857142857143 w[23,25] - 0.00142857142857143 w[24,1]
   - 0.00142857142857143 w[24,2] - 8.190990731192418 w[24,3]
   - 5.444371255805486 w[24,4] - 4.87596380720833 w[24,5]
   - 3.411303147851021 w[24,6] - 3.594178074992532 w[24,7]
   - 8.190990731192418 w[24,8] - 5.444371255805486 w[24,9]
   - 4.87596380720833 w[24,10] - 3.411303147851021 w[24,11]
   - 3.594178074992532 w[24,12] - 8.190990731192418 w[24,13]
   - 5.444371255805486 w[24,14] - 4.87596380720833 w[24,15]
   - 3.411303147851021 w[24,16] - 3.594178074992532 w[24,17]
   - 8.190990731192418 w[24,18] - 5.444371255805486 w[24,19]
   - 4.87596380720833 w[24,20] - 3.411303147851021 w[24,21]
   - 3.594178074992532 w[24,22] - 0.00142857142857143 w[24,23]
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
 VehicleTimes[1]: - 0.01 x[0,1] - 0.01 x[0,2] - 57.33693511834693 x[0,3]
   - 38.11059879063841 x[0,4] - 34.13174665045831 x[0,5]
   - 23.87912203495715 x[0,6] - 25.15924652494773 x[0,7]
   - 57.33693511834693 x[0,13] - 38.11059879063841 x[0,14]
   - 34.13174665045831 x[0,15] - 23.87912203495715 x[0,16]
   - 25.15924652494773 x[0,17] - 0.01 x[0,23] - 0.01 x[0,24] - 0.01 x[0,25]
   - 0.01 x[1,1] - 0.01 x[1,2] - 57.33693511834693 x[1,3]
   - 38.11059879063841 x[1,4] - 34.13174665045831 x[1,5]
   - 23.87912203495715 x[1,6] - 25.15924652494773 x[1,7]
   - 57.33693511834693 x[1,13] - 38.11059879063841 x[1,14]
   - 34.13174665045831 x[1,15] - 23.87912203495715 x[1,16]
   - 25.15924652494773 x[1,17] - 0.01 x[1,23] - 0.01 x[1,24] - 0.01 x[1,25]
   - 0.01 x[2,1] - 0.01 x[2,2] - 57.33693511834693 x[2,3]
   - 38.11059879063841 x[2,4] - 34.13174665045831 x[2,5]
   - 23.87912203495715 x[2,6] - 25.15924652494773 x[2,7]
   - 57.33693511834693 x[2,13] - 38.11059879063841 x[2,14]
   - 34.13174665045831 x[2,15] - 23.87912203495715 x[2,16]
   - 25.15924652494773 x[2,17] - 0.01 x[2,23] - 0.01 x[2,24] - 0.01 x[2,25]
   - 57.33693511834693 x[3,1] - 57.33693511834693 x[3,2] - 0.01 x[3,3]
   - 46.64695223763305 x[3,4] - 29.0553646660606 x[3,5]
   - 35.1454712306242 x[3,6] - 69.89359286576132 x[3,7] - 0.01 x[3,13]
   - 46.64695223763305 x[3,14] - 29.0553646660606 x[3,15]
   - 35.1454712306242 x[3,16] - 69.89359286576132 x[3,17]
   - 57.33693511834693 x[3,23] - 57.33693511834693 x[3,24]
   - 57.33693511834693 x[3,25] - 38.11059879063841 x[4,1]
   - 38.11059879063841 x[4,2] - 46.64695223763305 x[4,3] - 0.01 x[4,4]
   - 19.84997979424075 x[4,5] - 23.99768479926714 x[4,6]
   - 31.59080172060562 x[4,7] - 46.64695223763305 x[4,13] - 0.01 x[4,14]
   - 19.84997979424075 x[4,15] - 23.99768479926714 x[4,16]
   - 31.59080172060562 x[4,17] - 38.11059879063841 x[4,23]
   - 38.11059879063841 x[4,24] - 38.11059879063841 x[4,25]
   - 34.13174665045831 x[5,1] - 34.13174665045831 x[5,2]
   - 29.0553646660606 x[5,3] - 19.84997979424075 x[5,4] - 0.01 x[5,5]
   - 10.6898391704804 x[5,6] - 41.23318087185052 x[5,7]
   - 29.0553646660606 x[5,13] - 19.84997979424075 x[5,14] - 0.01 x[5,15]
   - 10.6898391704804 x[5,16] - 41.23318087185052 x[5,17]
   - 34.13174665045831 x[5,23] - 34.13174665045831 x[5,24]
   - 34.13174665045831 x[5,25] - 23.87912203495715 x[6,1]
   - 23.87912203495715 x[6,2] - 35.1454712306242 x[6,3]
   - 23.99768479926714 x[6,4] - 10.6898391704804 x[6,5] - 0.01 x[6,6]
   - 35.52661638953724 x[6,7] - 35.1454712306242 x[6,13]
   - 23.99768479926714 x[6,14] - 10.6898391704804 x[6,15] - 0.01 x[6,16]
   - 35.52661638953724 x[6,17] - 23.87912203495715 x[6,23]
   - 23.87912203495715 x[6,24] - 23.87912203495715 x[6,25]
   - 25.15924652494773 x[7,1] - 25.15924652494773 x[7,2]
   - 69.89359286576132 x[7,3] - 31.59080172060562 x[7,4]
   - 41.23318087185052 x[7,5] - 35.52661638953724 x[7,6] - 0.01 x[7,7]
   - 69.89359286576132 x[7,13] - 31.59080172060562 x[7,14]
   - 41.23318087185052 x[7,15] - 35.52661638953724 x[7,16] - 0.01 x[7,17]
   - 25.15924652494773 x[7,23] - 25.15924652494773 x[7,24]
   - 25.15924652494773 x[7,25] - 57.33693511834693 x[13,1]
   - 57.33693511834693 x[13,2] - 0.01 x[13,3] - 46.64695223763305 x[13,4]
   - 29.0553646660606 x[13,5] - 35.1454712306242 x[13,6]
   - 69.89359286576132 x[13,7] - 0.01 x[13,13] - 46.64695223763305 x[13,14]
   - 29.0553646660606 x[13,15] - 35.1454712306242 x[13,16]
   - 69.89359286576132 x[13,17] - 57.33693511834693 x[13,23]
   - 57.33693511834693 x[13,24] - 57.33693511834693 x[13,25]
   - 38.11059879063841 x[14,1] - 38.11059879063841 x[14,2]
   - 46.64695223763305 x[14,3] - 0.01 x[14,4] - 19.84997979424075 x[14,5]
   - 23.99768479926714 x[14,6] - 31.59080172060562 x[14,7]
   - 46.64695223763305 x[14,13] - 0.01 x[14,14]
   - 19.84997979424075 x[14,15] - 23.99768479926714 x[14,16]
   - 31.59080172060562 x[14,17] - 38.11059879063841 x[14,23]
   - 38.11059879063841 x[14,24] - 38.11059879063841 x[14,25]
   - 34.13174665045831 x[15,1] - 34.13174665045831 x[15,2]
   - 29.0553646660606 x[15,3] - 19.84997979424075 x[15,4] - 0.01 x[15,5]
   - 10.6898391704804 x[15,6] - 41.23318087185052 x[15,7]
   - 29.0553646660606 x[15,13] - 19.84997979424075 x[15,14] - 0.01 x[15,15]
   - 10.6898391704804 x[15,16] - 41.23318087185052 x[15,17]
   - 34.13174665045831 x[15,23] - 34.13174665045831 x[15,24]
   - 34.13174665045831 x[15,25] - 23.87912203495715 x[16,1]
   - 23.87912203495715 x[16,2] - 35.1454712306242 x[16,3]
   - 23.99768479926714 x[16,4] - 10.6898391704804 x[16,5] - 0.01 x[16,6]
   - 35.52661638953724 x[16,7] - 35.1454712306242 x[16,13]
   - 23.99768479926714 x[16,14] - 10.6898391704804 x[16,15] - 0.01 x[16,16]
   - 35.52661638953724 x[16,17] - 23.87912203495715 x[16,23]
   - 23.87912203495715 x[16,24] - 23.87912203495715 x[16,25]
   - 25.15924652494773 x[17,1] - 25.15924652494773 x[17,2]
   - 69.89359286576132 x[17,3] - 31.59080172060562 x[17,4]
   - 41.23318087185052 x[17,5] - 35.52661638953724 x[17,6] - 0.01 x[17,7]
   - 69.89359286576132 x[17,13] - 31.59080172060562 x[17,14]
   - 41.23318087185052 x[17,15] - 35.52661638953724 x[17,16]
   - 0.01 x[17,17] - 25.15924652494773 x[17,23]
   - 25.15924652494773 x[17,24] - 25.15924652494773 x[17,25] - 0.01 x[23,1]
   - 0.01 x[23,2] - 57.33693511834693 x[23,3] - 38.11059879063841 x[23,4]
   - 34.13174665045831 x[23,5] - 23.87912203495715 x[23,6]
   - 25.15924652494773 x[23,7] - 57.33693511834693 x[23,13]
   - 38.11059879063841 x[23,14] - 34.13174665045831 x[23,15]
   - 23.87912203495715 x[23,16] - 25.15924652494773 x[23,17]
   - 0.01 x[23,23] - 0.01 x[23,24] - 0.01 x[23,25] - 0.01 x[24,1]
   - 0.01 x[24,2] - 57.33693511834693 x[24,3] - 38.11059879063841 x[24,4]
   - 34.13174665045831 x[24,5] - 23.87912203495715 x[24,6]
   - 25.15924652494773 x[24,7] - 57.33693511834693 x[24,13]
   - 38.11059879063841 x[24,14] - 34.13174665045831 x[24,15]
   - 23.87912203495715 x[24,16] - 25.15924652494773 x[24,17]
   - 0.01 x[24,23] - 0.01 x[24,24] - 0.01 x[24,25] + start_service[25]
   >= 0
 VehicleTimes[2]: - 0.01 x[0,1] - 0.01 x[0,2] - 57.33693511834693 x[0,8]
   - 38.11059879063841 x[0,9] - 34.13174665045831 x[0,10]
   - 23.87912203495715 x[0,11] - 25.15924652494773 x[0,12]
   - 57.33693511834693 x[0,18] - 38.11059879063841 x[0,19]
   - 34.13174665045831 x[0,20] - 23.87912203495715 x[0,21]
   - 25.15924652494773 x[0,22] - 0.01 x[0,23] - 0.01 x[0,24] - 0.01 x[0,25]
   - 0.01 x[1,1] - 0.01 x[1,2] - 57.33693511834693 x[1,8]
   - 38.11059879063841 x[1,9] - 34.13174665045831 x[1,10]
   - 23.87912203495715 x[1,11] - 25.15924652494773 x[1,12]
   - 57.33693511834693 x[1,18] - 38.11059879063841 x[1,19]
   - 34.13174665045831 x[1,20] - 23.87912203495715 x[1,21]
   - 25.15924652494773 x[1,22] - 0.01 x[1,23] - 0.01 x[1,24] - 0.01 x[1,25]
   - 0.01 x[2,1] - 0.01 x[2,2] - 57.33693511834693 x[2,8]
   - 38.11059879063841 x[2,9] - 34.13174665045831 x[2,10]
   - 23.87912203495715 x[2,11] - 25.15924652494773 x[2,12]
   - 57.33693511834693 x[2,18] - 38.11059879063841 x[2,19]
   - 34.13174665045831 x[2,20] - 23.87912203495715 x[2,21]
   - 25.15924652494773 x[2,22] - 0.01 x[2,23] - 0.01 x[2,24] - 0.01 x[2,25]
   - 57.33693511834693 x[8,1] - 57.33693511834693 x[8,2] - 0.01 x[8,8]
   - 46.64695223763305 x[8,9] - 29.0553646660606 x[8,10]
   - 35.1454712306242 x[8,11] - 69.89359286576132 x[8,12] - 0.01 x[8,18]
   - 46.64695223763305 x[8,19] - 29.0553646660606 x[8,20]
   - 35.1454712306242 x[8,21] - 69.89359286576132 x[8,22]
   - 57.33693511834693 x[8,23] - 57.33693511834693 x[8,24]
   - 57.33693511834693 x[8,25] - 38.11059879063841 x[9,1]
   - 38.11059879063841 x[9,2] - 46.64695223763305 x[9,8] - 0.01 x[9,9]
   - 19.84997979424075 x[9,10] - 23.99768479926714 x[9,11]
   - 31.59080172060562 x[9,12] - 46.64695223763305 x[9,18] - 0.01 x[9,19]
   - 19.84997979424075 x[9,20] - 23.99768479926714 x[9,21]
   - 31.59080172060562 x[9,22] - 38.11059879063841 x[9,23]
   - 38.11059879063841 x[9,24] - 38.11059879063841 x[9,25]
   - 34.13174665045831 x[10,1] - 34.13174665045831 x[10,2]
   - 29.0553646660606 x[10,8] - 19.84997979424075 x[10,9] - 0.01 x[10,10]
   - 10.6898391704804 x[10,11] - 41.23318087185052 x[10,12]
   - 29.0553646660606 x[10,18] - 19.84997979424075 x[10,19] - 0.01 x[10,20]
   - 10.6898391704804 x[10,21] - 41.23318087185052 x[10,22]
   - 34.13174665045831 x[10,23] - 34.13174665045831 x[10,24]
   - 34.13174665045831 x[10,25] - 23.87912203495715 x[11,1]
   - 23.87912203495715 x[11,2] - 35.1454712306242 x[11,8]
   - 23.99768479926714 x[11,9] - 10.6898391704804 x[11,10] - 0.01 x[11,11]
   - 35.52661638953724 x[11,12] - 35.1454712306242 x[11,18]
   - 23.99768479926714 x[11,19] - 10.6898391704804 x[11,20] - 0.01 x[11,21]
   - 35.52661638953724 x[11,22] - 23.87912203495715 x[11,23]
   - 23.87912203495715 x[11,24] - 23.87912203495715 x[11,25]
   - 25.15924652494773 x[12,1] - 25.15924652494773 x[12,2]
   - 69.89359286576132 x[12,8] - 31.59080172060562 x[12,9]
   - 41.23318087185052 x[12,10] - 35.52661638953724 x[12,11]
   - 0.01 x[12,12] - 69.89359286576132 x[12,18]
   - 31.59080172060562 x[12,19] - 41.23318087185052 x[12,20]
   - 35.52661638953724 x[12,21] - 0.01 x[12,22]
   - 25.15924652494773 x[12,23] - 25.15924652494773 x[12,24]
   - 25.15924652494773 x[12,25] - 57.33693511834693 x[18,1]
   - 57.33693511834693 x[18,2] - 0.01 x[18,8] - 46.64695223763305 x[18,9]
   - 29.0553646660606 x[18,10] - 35.1454712306242 x[18,11]
   - 69.89359286576132 x[18,12] - 0.01 x[18,18]
   - 46.64695223763305 x[18,19] - 29.0553646660606 x[18,20]
   - 35.1454712306242 x[18,21] - 69.89359286576132 x[18,22]
   - 57.33693511834693 x[18,23] - 57.33693511834693 x[18,24]
   - 57.33693511834693 x[18,25] - 38.11059879063841 x[19,1]
   - 38.11059879063841 x[19,2] - 46.64695223763305 x[19,8] - 0.01 x[19,9]
   - 19.84997979424075 x[19,10] - 23.99768479926714 x[19,11]
   - 31.59080172060562 x[19,12] - 46.64695223763305 x[19,18]
   - 0.01 x[19,19] - 19.84997979424075 x[19,20]
   - 23.99768479926714 x[19,21] - 31.59080172060562 x[19,22]
   - 38.11059879063841 x[19,23] - 38.11059879063841 x[19,24]
   - 38.11059879063841 x[19,25] - 34.13174665045831 x[20,1]
   - 34.13174665045831 x[20,2] - 29.0553646660606 x[20,8]
   - 19.84997979424075 x[20,9] - 0.01 x[20,10] - 10.6898391704804 x[20,11]
   - 41.23318087185052 x[20,12] - 29.0553646660606 x[20,18]
   - 19.84997979424075 x[20,19] - 0.01 x[20,20] - 10.6898391704804 x[20,21]
   - 41.23318087185052 x[20,22] - 34.13174665045831 x[20,23]
   - 34.13174665045831 x[20,24] - 34.13174665045831 x[20,25]
   - 23.87912203495715 x[21,1] - 23.87912203495715 x[21,2]
   - 35.1454712306242 x[21,8] - 23.99768479926714 x[21,9]
   - 10.6898391704804 x[21,10] - 0.01 x[21,11] - 35.52661638953724 x[21,12]
   - 35.1454712306242 x[21,18] - 23.99768479926714 x[21,19]
   - 10.6898391704804 x[21,20] - 0.01 x[21,21] - 35.52661638953724 x[21,22]
   - 23.87912203495715 x[21,23] - 23.87912203495715 x[21,24]
   - 23.87912203495715 x[21,25] - 25.15924652494773 x[22,1]
   - 25.15924652494773 x[22,2] - 69.89359286576132 x[22,8]
   - 31.59080172060562 x[22,9] - 41.23318087185052 x[22,10]
   - 35.52661638953724 x[22,11] - 0.01 x[22,12]
   - 69.89359286576132 x[22,18] - 31.59080172060562 x[22,19]
   - 41.23318087185052 x[22,20] - 35.52661638953724 x[22,21]
   - 0.01 x[22,22] - 25.15924652494773 x[22,23]
   - 25.15924652494773 x[22,24] - 25.15924652494773 x[22,25] - 0.01 x[23,1]
   - 0.01 x[23,2] - 57.33693511834693 x[23,8] - 38.11059879063841 x[23,9]
   - 34.13174665045831 x[23,10] - 23.87912203495715 x[23,11]
   - 25.15924652494773 x[23,12] - 57.33693511834693 x[23,18]
   - 38.11059879063841 x[23,19] - 34.13174665045831 x[23,20]
   - 23.87912203495715 x[23,21] - 25.15924652494773 x[23,22]
   - 0.01 x[23,23] - 0.01 x[23,24] - 0.01 x[23,25] - 0.01 x[24,1]
   - 0.01 x[24,2] - 57.33693511834693 x[24,8] - 38.11059879063841 x[24,9]
   - 34.13174665045831 x[24,10] - 23.87912203495715 x[24,11]
   - 25.15924652494773 x[24,12] - 57.33693511834693 x[24,18]
   - 38.11059879063841 x[24,19] - 34.13174665045831 x[24,20]
   - 23.87912203495715 x[24,21] - 25.15924652494773 x[24,22]
   - 0.01 x[24,23] - 0.01 x[24,24] - 0.01 x[24,25] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[0]: - 0.02 x[0,1] - 0.02 x[0,2]
   - 114.6738702366939 x[0,3] - 76.22119758127681 x[0,4]
   - 68.26349330091662 x[0,5] - 47.7582440699143 x[0,6]
   - 50.31849304989545 x[0,7] - 114.6738702366939 x[0,8]
   - 76.22119758127681 x[0,9] - 68.26349330091662 x[0,10]
   - 47.7582440699143 x[0,11] - 50.31849304989545 x[0,12]
   - 114.6738702366939 x[0,13] - 76.22119758127681 x[0,14]
   - 68.26349330091662 x[0,15] - 47.7582440699143 x[0,16]
   - 50.31849304989545 x[0,17] - 114.6738702366939 x[0,18]
   - 76.22119758127681 x[0,19] - 68.26349330091662 x[0,20]
   - 47.7582440699143 x[0,21] - 50.31849304989545 x[0,22] - 0.02 x[0,23]
   - 0.02 x[0,24] - 0.02 x[0,25] - start_service[0] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[1]: - 0.02 x[1,1] - 0.02 x[1,2]
   - 114.6738702366939 x[1,3] - 76.22119758127681 x[1,4]
   - 68.26349330091662 x[1,5] - 47.7582440699143 x[1,6]
   - 50.31849304989545 x[1,7] - 114.6738702366939 x[1,8]
   - 76.22119758127681 x[1,9] - 68.26349330091662 x[1,10]
   - 47.7582440699143 x[1,11] - 50.31849304989545 x[1,12]
   - 114.6738702366939 x[1,13] - 76.22119758127681 x[1,14]
   - 68.26349330091662 x[1,15] - 47.7582440699143 x[1,16]
   - 50.31849304989545 x[1,17] - 114.6738702366939 x[1,18]
   - 76.22119758127681 x[1,19] - 68.26349330091662 x[1,20]
   - 47.7582440699143 x[1,21] - 50.31849304989545 x[1,22] - 0.02 x[1,23]
   - 0.02 x[1,24] - 0.02 x[1,25] - start_service[1] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[2]: - 0.02 x[2,1] - 0.02 x[2,2]
   - 114.6738702366939 x[2,3] - 76.22119758127681 x[2,4]
   - 68.26349330091662 x[2,5] - 47.7582440699143 x[2,6]
   - 50.31849304989545 x[2,7] - 114.6738702366939 x[2,8]
   - 76.22119758127681 x[2,9] - 68.26349330091662 x[2,10]
   - 47.7582440699143 x[2,11] - 50.31849304989545 x[2,12]
   - 114.6738702366939 x[2,13] - 76.22119758127681 x[2,14]
   - 68.26349330091662 x[2,15] - 47.7582440699143 x[2,16]
   - 50.31849304989545 x[2,17] - 114.6738702366939 x[2,18]
   - 76.22119758127681 x[2,19] - 68.26349330091662 x[2,20]
   - 47.7582440699143 x[2,21] - 50.31849304989545 x[2,22] - 0.02 x[2,23]
   - 0.02 x[2,24] - 0.02 x[2,25] - start_service[2] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[3]: - 57.34693511834693 x[3,1]
   - 57.34693511834693 x[3,2] - 57.34693511834693 x[3,3]
   - 84.75755102827145 x[3,4] - 63.18711131651891 x[3,5]
   - 59.02459326558134 x[3,6] - 95.05283939070904 x[3,7]
   - 57.34693511834693 x[3,8] - 84.75755102827145 x[3,9]
   - 63.18711131651891 x[3,10] - 59.02459326558134 x[3,11]
   - 95.05283939070904 x[3,12] - 57.34693511834693 x[3,13]
   - 84.75755102827145 x[3,14] - 63.18711131651891 x[3,15]
   - 59.02459326558134 x[3,16] - 95.05283939070904 x[3,17]
   - 57.34693511834693 x[3,18] - 84.75755102827145 x[3,19]
   - 63.18711131651891 x[3,20] - 59.02459326558134 x[3,21]
   - 95.05283939070904 x[3,22] - 57.34693511834693 x[3,23]
   - 57.34693511834693 x[3,24] - 57.34693511834693 x[3,25]
   - start_service[3] + start_service[25] >= 0
 LowerBoundRoutingVehicle[4]: - 38.1205987906384 x[4,1]
   - 38.1205987906384 x[4,2] - 103.98388735598 x[4,3]
   - 38.1205987906384 x[4,4] - 53.98172644469906 x[4,5]
   - 47.8768068342243 x[4,6] - 56.75004824555334 x[4,7]
   - 103.98388735598 x[4,8] - 38.1205987906384 x[4,9]
   - 53.98172644469906 x[4,10] - 47.8768068342243 x[4,11]
   - 56.75004824555334 x[4,12] - 103.98388735598 x[4,13]
   - 38.1205987906384 x[4,14] - 53.98172644469906 x[4,15]
   - 47.8768068342243 x[4,16] - 56.75004824555334 x[4,17]
   - 103.98388735598 x[4,18] - 38.1205987906384 x[4,19]
   - 53.98172644469906 x[4,20] - 47.8768068342243 x[4,21]
   - 56.75004824555334 x[4,22] - 38.1205987906384 x[4,23]
   - 38.1205987906384 x[4,24] - 38.1205987906384 x[4,25] - start_service[4]
   + start_service[25] >= 0
 LowerBoundRoutingVehicle[5]: - 34.14174665045831 x[5,1]
   - 34.14174665045831 x[5,2] - 86.39229978440753 x[5,3]
   - 57.96057858487916 x[5,4] - 34.14174665045831 x[5,5]
   - 34.56896120543755 x[5,6] - 66.39242739679824 x[5,7]
   - 86.39229978440753 x[5,8] - 57.96057858487916 x[5,9]
   - 34.14174665045831 x[5,10] - 34.56896120543755 x[5,11]
   - 66.39242739679824 x[5,12] - 86.39229978440753 x[5,13]
   - 57.96057858487916 x[5,14] - 34.14174665045831 x[5,15]
   - 34.56896120543755 x[5,16] - 66.39242739679824 x[5,17]
   - 86.39229978440753 x[5,18] - 57.96057858487916 x[5,19]
   - 34.14174665045831 x[5,20] - 34.56896120543755 x[5,21]
   - 66.39242739679824 x[5,22] - 34.14174665045831 x[5,23]
   - 34.14174665045831 x[5,24] - 34.14174665045831 x[5,25]
   - start_service[5] + start_service[25] >= 0
 LowerBoundRoutingVehicle[6]: - 23.88912203495715 x[6,1]
   - 23.88912203495715 x[6,2] - 92.48240634897112 x[6,3]
   - 62.10828358990555 x[6,4] - 44.82158582093871 x[6,5]
   - 23.88912203495715 x[6,6] - 60.68586291448496 x[6,7]
   - 92.48240634897112 x[6,8] - 62.10828358990555 x[6,9]
   - 44.82158582093871 x[6,10] - 23.88912203495715 x[6,11]
   - 60.68586291448496 x[6,12] - 92.48240634897112 x[6,13]
   - 62.10828358990555 x[6,14] - 44.82158582093871 x[6,15]
   - 23.88912203495715 x[6,16] - 60.68586291448496 x[6,17]
   - 92.48240634897112 x[6,18] - 62.10828358990555 x[6,19]
   - 44.82158582093871 x[6,20] - 23.88912203495715 x[6,21]
   - 60.68586291448496 x[6,22] - 23.88912203495715 x[6,23]
   - 23.88912203495715 x[6,24] - 23.88912203495715 x[6,25]
   - start_service[6] + start_service[25] >= 0
 LowerBoundRoutingVehicle[7]: - 25.16924652494773 x[7,1]
   - 25.16924652494773 x[7,2] - 127.2305279841082 x[7,3]
   - 69.70140051124403 x[7,4] - 75.36492752230883 x[7,5]
   - 59.40573842449439 x[7,6] - 25.16924652494773 x[7,7]
   - 127.2305279841082 x[7,8] - 69.70140051124403 x[7,9]
   - 75.36492752230883 x[7,10] - 59.40573842449439 x[7,11]
   - 25.16924652494773 x[7,12] - 127.2305279841082 x[7,13]
   - 69.70140051124403 x[7,14] - 75.36492752230883 x[7,15]
   - 59.40573842449439 x[7,16] - 25.16924652494773 x[7,17]
   - 127.2305279841082 x[7,18] - 69.70140051124403 x[7,19]
   - 75.36492752230883 x[7,20] - 59.40573842449439 x[7,21]
   - 25.16924652494773 x[7,22] - 25.16924652494773 x[7,23]
   - 25.16924652494773 x[7,24] - 25.16924652494773 x[7,25]
   - start_service[7] + start_service[25] >= 0
 LowerBoundRoutingVehicle[8]: - 57.34693511834693 x[8,1]
   - 57.34693511834693 x[8,2] - 57.34693511834693 x[8,3]
   - 84.75755102827145 x[8,4] - 63.18711131651891 x[8,5]
   - 59.02459326558134 x[8,6] - 95.05283939070904 x[8,7]
   - 57.34693511834693 x[8,8] - 84.75755102827145 x[8,9]
   - 63.18711131651891 x[8,10] - 59.02459326558134 x[8,11]
   - 95.05283939070904 x[8,12] - 57.34693511834693 x[8,13]
   - 84.75755102827145 x[8,14] - 63.18711131651891 x[8,15]
   - 59.02459326558134 x[8,16] - 95.05283939070904 x[8,17]
   - 57.34693511834693 x[8,18] - 84.75755102827145 x[8,19]
   - 63.18711131651891 x[8,20] - 59.02459326558134 x[8,21]
   - 95.05283939070904 x[8,22] - 57.34693511834693 x[8,23]
   - 57.34693511834693 x[8,24] - 57.34693511834693 x[8,25]
   - start_service[8] + start_service[25] >= 0
 LowerBoundRoutingVehicle[9]: - 38.1205987906384 x[9,1]
   - 38.1205987906384 x[9,2] - 103.98388735598 x[9,3]
   - 38.1205987906384 x[9,4] - 53.98172644469906 x[9,5]
   - 47.8768068342243 x[9,6] - 56.75004824555334 x[9,7]
   - 103.98388735598 x[9,8] - 38.1205987906384 x[9,9]
   - 53.98172644469906 x[9,10] - 47.8768068342243 x[9,11]
   - 56.75004824555334 x[9,12] - 103.98388735598 x[9,13]
   - 38.1205987906384 x[9,14] - 53.98172644469906 x[9,15]
   - 47.8768068342243 x[9,16] - 56.75004824555334 x[9,17]
   - 103.98388735598 x[9,18] - 38.1205987906384 x[9,19]
   - 53.98172644469906 x[9,20] - 47.8768068342243 x[9,21]
   - 56.75004824555334 x[9,22] - 38.1205987906384 x[9,23]
   - 38.1205987906384 x[9,24] - 38.1205987906384 x[9,25] - start_service[9]
   + start_service[25] >= 0
 LowerBoundRoutingVehicle[10]: - 34.14174665045831 x[10,1]
   - 34.14174665045831 x[10,2] - 86.39229978440753 x[10,3]
   - 57.96057858487916 x[10,4] - 34.14174665045831 x[10,5]
   - 34.56896120543755 x[10,6] - 66.39242739679824 x[10,7]
   - 86.39229978440753 x[10,8] - 57.96057858487916 x[10,9]
   - 34.14174665045831 x[10,10] - 34.56896120543755 x[10,11]
   - 66.39242739679824 x[10,12] - 86.39229978440753 x[10,13]
   - 57.96057858487916 x[10,14] - 34.14174665045831 x[10,15]
   - 34.56896120543755 x[10,16] - 66.39242739679824 x[10,17]
   - 86.39229978440753 x[10,18] - 57.96057858487916 x[10,19]
   - 34.14174665045831 x[10,20] - 34.56896120543755 x[10,21]
   - 66.39242739679824 x[10,22] - 34.14174665045831 x[10,23]
   - 34.14174665045831 x[10,24] - 34.14174665045831 x[10,25]
   - start_service[10] + start_service[25] >= 0
 LowerBoundRoutingVehicle[11]: - 23.88912203495715 x[11,1]
   - 23.88912203495715 x[11,2] - 92.48240634897112 x[11,3]
   - 62.10828358990555 x[11,4] - 44.82158582093871 x[11,5]
   - 23.88912203495715 x[11,6] - 60.68586291448496 x[11,7]
   - 92.48240634897112 x[11,8] - 62.10828358990555 x[11,9]
   - 44.82158582093871 x[11,10] - 23.88912203495715 x[11,11]
   - 60.68586291448496 x[11,12] - 92.48240634897112 x[11,13]
   - 62.10828358990555 x[11,14] - 44.82158582093871 x[11,15]
   - 23.88912203495715 x[11,16] - 60.68586291448496 x[11,17]
   - 92.48240634897112 x[11,18] - 62.10828358990555 x[11,19]
   - 44.82158582093871 x[11,20] - 23.88912203495715 x[11,21]
   - 60.68586291448496 x[11,22] - 23.88912203495715 x[11,23]
   - 23.88912203495715 x[11,24] - 23.88912203495715 x[11,25]
   - start_service[11] + start_service[25] >= 0
 LowerBoundRoutingVehicle[12]: - 25.16924652494773 x[12,1]
   - 25.16924652494773 x[12,2] - 127.2305279841082 x[12,3]
   - 69.70140051124403 x[12,4] - 75.36492752230883 x[12,5]
   - 59.40573842449439 x[12,6] - 25.16924652494773 x[12,7]
   - 127.2305279841082 x[12,8] - 69.70140051124403 x[12,9]
   - 75.36492752230883 x[12,10] - 59.40573842449439 x[12,11]
   - 25.16924652494773 x[12,12] - 127.2305279841082 x[12,13]
   - 69.70140051124403 x[12,14] - 75.36492752230883 x[12,15]
   - 59.40573842449439 x[12,16] - 25.16924652494773 x[12,17]
   - 127.2305279841082 x[12,18] - 69.70140051124403 x[12,19]
   - 75.36492752230883 x[12,20] - 59.40573842449439 x[12,21]
   - 25.16924652494773 x[12,22] - 25.16924652494773 x[12,23]
   - 25.16924652494773 x[12,24] - 25.16924652494773 x[12,25]
   - start_service[12] + start_service[25] >= 0
 LowerBoundRoutingVehicle[13]: - 57.34693511834693 x[13,1]
   - 57.34693511834693 x[13,2] - 57.34693511834693 x[13,3]
   - 84.75755102827145 x[13,4] - 63.18711131651891 x[13,5]
   - 59.02459326558134 x[13,6] - 95.05283939070904 x[13,7]
   - 57.34693511834693 x[13,8] - 84.75755102827145 x[13,9]
   - 63.18711131651891 x[13,10] - 59.02459326558134 x[13,11]
   - 95.05283939070904 x[13,12] - 57.34693511834693 x[13,13]
   - 84.75755102827145 x[13,14] - 63.18711131651891 x[13,15]
   - 59.02459326558134 x[13,16] - 95.05283939070904 x[13,17]
   - 57.34693511834693 x[13,18] - 84.75755102827145 x[13,19]
   - 63.18711131651891 x[13,20] - 59.02459326558134 x[13,21]
   - 95.05283939070904 x[13,22] - 57.34693511834693 x[13,23]
   - 57.34693511834693 x[13,24] - 57.34693511834693 x[13,25]
   - start_service[13] + start_service[25] >= 0
 LowerBoundRoutingVehicle[14]: - 38.1205987906384 x[14,1]
   - 38.1205987906384 x[14,2] - 103.98388735598 x[14,3]
   - 38.1205987906384 x[14,4] - 53.98172644469906 x[14,5]
   - 47.8768068342243 x[14,6] - 56.75004824555334 x[14,7]
   - 103.98388735598 x[14,8] - 38.1205987906384 x[14,9]
   - 53.98172644469906 x[14,10] - 47.8768068342243 x[14,11]
   - 56.75004824555334 x[14,12] - 103.98388735598 x[14,13]
   - 38.1205987906384 x[14,14] - 53.98172644469906 x[14,15]
   - 47.8768068342243 x[14,16] - 56.75004824555334 x[14,17]
   - 103.98388735598 x[14,18] - 38.1205987906384 x[14,19]
   - 53.98172644469906 x[14,20] - 47.8768068342243 x[14,21]
   - 56.75004824555334 x[14,22] - 38.1205987906384 x[14,23]
   - 38.1205987906384 x[14,24] - 38.1205987906384 x[14,25]
   - start_service[14] + start_service[25] >= 0
 LowerBoundRoutingVehicle[15]: - 34.14174665045831 x[15,1]
   - 34.14174665045831 x[15,2] - 86.39229978440753 x[15,3]
   - 57.96057858487916 x[15,4] - 34.14174665045831 x[15,5]
   - 34.56896120543755 x[15,6] - 66.39242739679824 x[15,7]
   - 86.39229978440753 x[15,8] - 57.96057858487916 x[15,9]
   - 34.14174665045831 x[15,10] - 34.56896120543755 x[15,11]
   - 66.39242739679824 x[15,12] - 86.39229978440753 x[15,13]
   - 57.96057858487916 x[15,14] - 34.14174665045831 x[15,15]
   - 34.56896120543755 x[15,16] - 66.39242739679824 x[15,17]
   - 86.39229978440753 x[15,18] - 57.96057858487916 x[15,19]
   - 34.14174665045831 x[15,20] - 34.56896120543755 x[15,21]
   - 66.39242739679824 x[15,22] - 34.14174665045831 x[15,23]
   - 34.14174665045831 x[15,24] - 34.14174665045831 x[15,25]
   - start_service[15] + start_service[25] >= 0
 LowerBoundRoutingVehicle[16]: - 23.88912203495715 x[16,1]
   - 23.88912203495715 x[16,2] - 92.48240634897112 x[16,3]
   - 62.10828358990555 x[16,4] - 44.82158582093871 x[16,5]
   - 23.88912203495715 x[16,6] - 60.68586291448496 x[16,7]
   - 92.48240634897112 x[16,8] - 62.10828358990555 x[16,9]
   - 44.82158582093871 x[16,10] - 23.88912203495715 x[16,11]
   - 60.68586291448496 x[16,12] - 92.48240634897112 x[16,13]
   - 62.10828358990555 x[16,14] - 44.82158582093871 x[16,15]
   - 23.88912203495715 x[16,16] - 60.68586291448496 x[16,17]
   - 92.48240634897112 x[16,18] - 62.10828358990555 x[16,19]
   - 44.82158582093871 x[16,20] - 23.88912203495715 x[16,21]
   - 60.68586291448496 x[16,22] - 23.88912203495715 x[16,23]
   - 23.88912203495715 x[16,24] - 23.88912203495715 x[16,25]
   - start_service[16] + start_service[25] >= 0
 LowerBoundRoutingVehicle[17]: - 25.16924652494773 x[17,1]
   - 25.16924652494773 x[17,2] - 127.2305279841082 x[17,3]
   - 69.70140051124403 x[17,4] - 75.36492752230883 x[17,5]
   - 59.40573842449439 x[17,6] - 25.16924652494773 x[17,7]
   - 127.2305279841082 x[17,8] - 69.70140051124403 x[17,9]
   - 75.36492752230883 x[17,10] - 59.40573842449439 x[17,11]
   - 25.16924652494773 x[17,12] - 127.2305279841082 x[17,13]
   - 69.70140051124403 x[17,14] - 75.36492752230883 x[17,15]
   - 59.40573842449439 x[17,16] - 25.16924652494773 x[17,17]
   - 127.2305279841082 x[17,18] - 69.70140051124403 x[17,19]
   - 75.36492752230883 x[17,20] - 59.40573842449439 x[17,21]
   - 25.16924652494773 x[17,22] - 25.16924652494773 x[17,23]
   - 25.16924652494773 x[17,24] - 25.16924652494773 x[17,25]
   - start_service[17] + start_service[25] >= 0
 LowerBoundRoutingVehicle[18]: - 57.34693511834693 x[18,1]
   - 57.34693511834693 x[18,2] - 57.34693511834693 x[18,3]
   - 84.75755102827145 x[18,4] - 63.18711131651891 x[18,5]
   - 59.02459326558134 x[18,6] - 95.05283939070904 x[18,7]
   - 57.34693511834693 x[18,8] - 84.75755102827145 x[18,9]
   - 63.18711131651891 x[18,10] - 59.02459326558134 x[18,11]
   - 95.05283939070904 x[18,12] - 57.34693511834693 x[18,13]
   - 84.75755102827145 x[18,14] - 63.18711131651891 x[18,15]
   - 59.02459326558134 x[18,16] - 95.05283939070904 x[18,17]
   - 57.34693511834693 x[18,18] - 84.75755102827145 x[18,19]
   - 63.18711131651891 x[18,20] - 59.02459326558134 x[18,21]
   - 95.05283939070904 x[18,22] - 57.34693511834693 x[18,23]
   - 57.34693511834693 x[18,24] - 57.34693511834693 x[18,25]
   - start_service[18] + start_service[25] >= 0
 LowerBoundRoutingVehicle[19]: - 38.1205987906384 x[19,1]
   - 38.1205987906384 x[19,2] - 103.98388735598 x[19,3]
   - 38.1205987906384 x[19,4] - 53.98172644469906 x[19,5]
   - 47.8768068342243 x[19,6] - 56.75004824555334 x[19,7]
   - 103.98388735598 x[19,8] - 38.1205987906384 x[19,9]
   - 53.98172644469906 x[19,10] - 47.8768068342243 x[19,11]
   - 56.75004824555334 x[19,12] - 103.98388735598 x[19,13]
   - 38.1205987906384 x[19,14] - 53.98172644469906 x[19,15]
   - 47.8768068342243 x[19,16] - 56.75004824555334 x[19,17]
   - 103.98388735598 x[19,18] - 38.1205987906384 x[19,19]
   - 53.98172644469906 x[19,20] - 47.8768068342243 x[19,21]
   - 56.75004824555334 x[19,22] - 38.1205987906384 x[19,23]
   - 38.1205987906384 x[19,24] - 38.1205987906384 x[19,25]
   - start_service[19] + start_service[25] >= 0
 LowerBoundRoutingVehicle[20]: - 34.14174665045831 x[20,1]
   - 34.14174665045831 x[20,2] - 86.39229978440753 x[20,3]
   - 57.96057858487916 x[20,4] - 34.14174665045831 x[20,5]
   - 34.56896120543755 x[20,6] - 66.39242739679824 x[20,7]
   - 86.39229978440753 x[20,8] - 57.96057858487916 x[20,9]
   - 34.14174665045831 x[20,10] - 34.56896120543755 x[20,11]
   - 66.39242739679824 x[20,12] - 86.39229978440753 x[20,13]
   - 57.96057858487916 x[20,14] - 34.14174665045831 x[20,15]
   - 34.56896120543755 x[20,16] - 66.39242739679824 x[20,17]
   - 86.39229978440753 x[20,18] - 57.96057858487916 x[20,19]
   - 34.14174665045831 x[20,20] - 34.56896120543755 x[20,21]
   - 66.39242739679824 x[20,22] - 34.14174665045831 x[20,23]
   - 34.14174665045831 x[20,24] - 34.14174665045831 x[20,25]
   - start_service[20] + start_service[25] >= 0
 LowerBoundRoutingVehicle[21]: - 23.88912203495715 x[21,1]
   - 23.88912203495715 x[21,2] - 92.48240634897112 x[21,3]
   - 62.10828358990555 x[21,4] - 44.82158582093871 x[21,5]
   - 23.88912203495715 x[21,6] - 60.68586291448496 x[21,7]
   - 92.48240634897112 x[21,8] - 62.10828358990555 x[21,9]
   - 44.82158582093871 x[21,10] - 23.88912203495715 x[21,11]
   - 60.68586291448496 x[21,12] - 92.48240634897112 x[21,13]
   - 62.10828358990555 x[21,14] - 44.82158582093871 x[21,15]
   - 23.88912203495715 x[21,16] - 60.68586291448496 x[21,17]
   - 92.48240634897112 x[21,18] - 62.10828358990555 x[21,19]
   - 44.82158582093871 x[21,20] - 23.88912203495715 x[21,21]
   - 60.68586291448496 x[21,22] - 23.88912203495715 x[21,23]
   - 23.88912203495715 x[21,24] - 23.88912203495715 x[21,25]
   - start_service[21] + start_service[25] >= 0
 LowerBoundRoutingVehicle[22]: - 25.16924652494773 x[22,1]
   - 25.16924652494773 x[22,2] - 127.2305279841082 x[22,3]
   - 69.70140051124403 x[22,4] - 75.36492752230883 x[22,5]
   - 59.40573842449439 x[22,6] - 25.16924652494773 x[22,7]
   - 127.2305279841082 x[22,8] - 69.70140051124403 x[22,9]
   - 75.36492752230883 x[22,10] - 59.40573842449439 x[22,11]
   - 25.16924652494773 x[22,12] - 127.2305279841082 x[22,13]
   - 69.70140051124403 x[22,14] - 75.36492752230883 x[22,15]
   - 59.40573842449439 x[22,16] - 25.16924652494773 x[22,17]
   - 127.2305279841082 x[22,18] - 69.70140051124403 x[22,19]
   - 75.36492752230883 x[22,20] - 59.40573842449439 x[22,21]
   - 25.16924652494773 x[22,22] - 25.16924652494773 x[22,23]
   - 25.16924652494773 x[22,24] - 25.16924652494773 x[22,25]
   - start_service[22] + start_service[25] >= 0
 LowerBoundRoutingVehicle[23]: - 0.02 x[23,1] - 0.02 x[23,2]
   - 114.6738702366939 x[23,3] - 76.22119758127681 x[23,4]
   - 68.26349330091662 x[23,5] - 47.7582440699143 x[23,6]
   - 50.31849304989545 x[23,7] - 114.6738702366939 x[23,8]
   - 76.22119758127681 x[23,9] - 68.26349330091662 x[23,10]
   - 47.7582440699143 x[23,11] - 50.31849304989545 x[23,12]
   - 114.6738702366939 x[23,13] - 76.22119758127681 x[23,14]
   - 68.26349330091662 x[23,15] - 47.7582440699143 x[23,16]
   - 50.31849304989545 x[23,17] - 114.6738702366939 x[23,18]
   - 76.22119758127681 x[23,19] - 68.26349330091662 x[23,20]
   - 47.7582440699143 x[23,21] - 50.31849304989545 x[23,22] - 0.02 x[23,23]
   - 0.02 x[23,24] - 0.02 x[23,25] - start_service[23] + start_service[25]
   >= 0
 LowerBoundRoutingVehicle[24]: - 0.02 x[24,1] - 0.02 x[24,2]
   - 114.6738702366939 x[24,3] - 76.22119758127681 x[24,4]
   - 68.26349330091662 x[24,5] - 47.7582440699143 x[24,6]
   - 50.31849304989545 x[24,7] - 114.6738702366939 x[24,8]
   - 76.22119758127681 x[24,9] - 68.26349330091662 x[24,10]
   - 47.7582440699143 x[24,11] - 50.31849304989545 x[24,12]
   - 114.6738702366939 x[24,13] - 76.22119758127681 x[24,14]
   - 68.26349330091662 x[24,15] - 47.7582440699143 x[24,16]
   - 50.31849304989545 x[24,17] - 114.6738702366939 x[24,18]
   - 76.22119758127681 x[24,19] - 68.26349330091662 x[24,20]
   - 47.7582440699143 x[24,21] - 50.31849304989545 x[24,22] - 0.02 x[24,23]
   - 0.02 x[24,24] - 0.02 x[24,25] - start_service[24] + start_service[25]
   >= 0
 LowerBoundArrivalVehicle[1]: - 0.02 x[0,1] - 0.02 x[1,1] - 0.02 x[2,1]
   - 114.6738702366939 x[3,1] - 76.22119758127681 x[4,1]
   - 68.26349330091662 x[5,1] - 47.7582440699143 x[6,1]
   - 50.31849304989545 x[7,1] - 114.6738702366939 x[8,1]
   - 76.22119758127681 x[9,1] - 68.26349330091662 x[10,1]
   - 47.7582440699143 x[11,1] - 50.31849304989545 x[12,1]
   - 114.6738702366939 x[13,1] - 76.22119758127681 x[14,1]
   - 68.26349330091662 x[15,1] - 47.7582440699143 x[16,1]
   - 50.31849304989545 x[17,1] - 114.6738702366939 x[18,1]
   - 76.22119758127681 x[19,1] - 68.26349330091662 x[20,1]
   - 47.7582440699143 x[21,1] - 50.31849304989545 x[22,1] - 0.02 x[23,1]
   - 0.02 x[24,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 0.02 x[0,2] - 0.02 x[1,2] - 0.02 x[2,2]
   - 114.6738702366939 x[3,2] - 76.22119758127681 x[4,2]
   - 68.26349330091662 x[5,2] - 47.7582440699143 x[6,2]
   - 50.31849304989545 x[7,2] - 114.6738702366939 x[8,2]
   - 76.22119758127681 x[9,2] - 68.26349330091662 x[10,2]
   - 47.7582440699143 x[11,2] - 50.31849304989545 x[12,2]
   - 114.6738702366939 x[13,2] - 76.22119758127681 x[14,2]
   - 68.26349330091662 x[15,2] - 47.7582440699143 x[16,2]
   - 50.31849304989545 x[17,2] - 114.6738702366939 x[18,2]
   - 76.22119758127681 x[19,2] - 68.26349330091662 x[20,2]
   - 47.7582440699143 x[21,2] - 50.31849304989545 x[22,2] - 0.02 x[23,2]
   - 0.02 x[24,2] - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 57.34693511834693 x[0,3]
   - 57.34693511834693 x[1,3] - 57.34693511834693 x[2,3]
   - 57.34693511834693 x[3,3] - 84.75755102827145 x[4,3]
   - 63.18711131651891 x[5,3] - 59.02459326558134 x[6,3]
   - 95.05283939070904 x[7,3] - 57.34693511834693 x[8,3]
   - 84.75755102827145 x[9,3] - 63.18711131651891 x[10,3]
   - 59.02459326558134 x[11,3] - 95.05283939070904 x[12,3]
   - 57.34693511834693 x[13,3] - 84.75755102827145 x[14,3]
   - 63.18711131651891 x[15,3] - 59.02459326558134 x[16,3]
   - 95.05283939070904 x[17,3] - 57.34693511834693 x[18,3]
   - 84.75755102827145 x[19,3] - 63.18711131651891 x[20,3]
   - 59.02459326558134 x[21,3] - 95.05283939070904 x[22,3]
   - 57.34693511834693 x[23,3] - 57.34693511834693 x[24,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 38.1205987906384 x[0,4]
   - 38.1205987906384 x[1,4] - 38.1205987906384 x[2,4]
   - 103.98388735598 x[3,4] - 38.1205987906384 x[4,4]
   - 53.98172644469906 x[5,4] - 47.8768068342243 x[6,4]
   - 56.75004824555334 x[7,4] - 103.98388735598 x[8,4]
   - 38.1205987906384 x[9,4] - 53.98172644469906 x[10,4]
   - 47.8768068342243 x[11,4] - 56.75004824555334 x[12,4]
   - 103.98388735598 x[13,4] - 38.1205987906384 x[14,4]
   - 53.98172644469906 x[15,4] - 47.8768068342243 x[16,4]
   - 56.75004824555334 x[17,4] - 103.98388735598 x[18,4]
   - 38.1205987906384 x[19,4] - 53.98172644469906 x[20,4]
   - 47.8768068342243 x[21,4] - 56.75004824555334 x[22,4]
   - 38.1205987906384 x[23,4] - 38.1205987906384 x[24,4] - start_service[0]
   + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 34.14174665045831 x[0,5]
   - 34.14174665045831 x[1,5] - 34.14174665045831 x[2,5]
   - 86.39229978440753 x[3,5] - 57.96057858487916 x[4,5]
   - 34.14174665045831 x[5,5] - 34.56896120543755 x[6,5]
   - 66.39242739679824 x[7,5] - 86.39229978440753 x[8,5]
   - 57.96057858487916 x[9,5] - 34.14174665045831 x[10,5]
   - 34.56896120543755 x[11,5] - 66.39242739679824 x[12,5]
   - 86.39229978440753 x[13,5] - 57.96057858487916 x[14,5]
   - 34.14174665045831 x[15,5] - 34.56896120543755 x[16,5]
   - 66.39242739679824 x[17,5] - 86.39229978440753 x[18,5]
   - 57.96057858487916 x[19,5] - 34.14174665045831 x[20,5]
   - 34.56896120543755 x[21,5] - 66.39242739679824 x[22,5]
   - 34.14174665045831 x[23,5] - 34.14174665045831 x[24,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 23.88912203495715 x[0,6]
   - 23.88912203495715 x[1,6] - 23.88912203495715 x[2,6]
   - 92.48240634897112 x[3,6] - 62.10828358990555 x[4,6]
   - 44.82158582093871 x[5,6] - 23.88912203495715 x[6,6]
   - 60.68586291448496 x[7,6] - 92.48240634897112 x[8,6]
   - 62.10828358990555 x[9,6] - 44.82158582093871 x[10,6]
   - 23.88912203495715 x[11,6] - 60.68586291448496 x[12,6]
   - 92.48240634897112 x[13,6] - 62.10828358990555 x[14,6]
   - 44.82158582093871 x[15,6] - 23.88912203495715 x[16,6]
   - 60.68586291448496 x[17,6] - 92.48240634897112 x[18,6]
   - 62.10828358990555 x[19,6] - 44.82158582093871 x[20,6]
   - 23.88912203495715 x[21,6] - 60.68586291448496 x[22,6]
   - 23.88912203495715 x[23,6] - 23.88912203495715 x[24,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 25.16924652494773 x[0,7]
   - 25.16924652494773 x[1,7] - 25.16924652494773 x[2,7]
   - 127.2305279841082 x[3,7] - 69.70140051124403 x[4,7]
   - 75.36492752230883 x[5,7] - 59.40573842449439 x[6,7]
   - 25.16924652494773 x[7,7] - 127.2305279841082 x[8,7]
   - 69.70140051124403 x[9,7] - 75.36492752230883 x[10,7]
   - 59.40573842449439 x[11,7] - 25.16924652494773 x[12,7]
   - 127.2305279841082 x[13,7] - 69.70140051124403 x[14,7]
   - 75.36492752230883 x[15,7] - 59.40573842449439 x[16,7]
   - 25.16924652494773 x[17,7] - 127.2305279841082 x[18,7]
   - 69.70140051124403 x[19,7] - 75.36492752230883 x[20,7]
   - 59.40573842449439 x[21,7] - 25.16924652494773 x[22,7]
   - 25.16924652494773 x[23,7] - 25.16924652494773 x[24,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 57.34693511834693 x[0,8]
   - 57.34693511834693 x[1,8] - 57.34693511834693 x[2,8]
   - 57.34693511834693 x[3,8] - 84.75755102827145 x[4,8]
   - 63.18711131651891 x[5,8] - 59.02459326558134 x[6,8]
   - 95.05283939070904 x[7,8] - 57.34693511834693 x[8,8]
   - 84.75755102827145 x[9,8] - 63.18711131651891 x[10,8]
   - 59.02459326558134 x[11,8] - 95.05283939070904 x[12,8]
   - 57.34693511834693 x[13,8] - 84.75755102827145 x[14,8]
   - 63.18711131651891 x[15,8] - 59.02459326558134 x[16,8]
   - 95.05283939070904 x[17,8] - 57.34693511834693 x[18,8]
   - 84.75755102827145 x[19,8] - 63.18711131651891 x[20,8]
   - 59.02459326558134 x[21,8] - 95.05283939070904 x[22,8]
   - 57.34693511834693 x[23,8] - 57.34693511834693 x[24,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 38.1205987906384 x[0,9]
   - 38.1205987906384 x[1,9] - 38.1205987906384 x[2,9]
   - 103.98388735598 x[3,9] - 38.1205987906384 x[4,9]
   - 53.98172644469906 x[5,9] - 47.8768068342243 x[6,9]
   - 56.75004824555334 x[7,9] - 103.98388735598 x[8,9]
   - 38.1205987906384 x[9,9] - 53.98172644469906 x[10,9]
   - 47.8768068342243 x[11,9] - 56.75004824555334 x[12,9]
   - 103.98388735598 x[13,9] - 38.1205987906384 x[14,9]
   - 53.98172644469906 x[15,9] - 47.8768068342243 x[16,9]
   - 56.75004824555334 x[17,9] - 103.98388735598 x[18,9]
   - 38.1205987906384 x[19,9] - 53.98172644469906 x[20,9]
   - 47.8768068342243 x[21,9] - 56.75004824555334 x[22,9]
   - 38.1205987906384 x[23,9] - 38.1205987906384 x[24,9] - start_service[0]
   + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 34.14174665045831 x[0,10]
   - 34.14174665045831 x[1,10] - 34.14174665045831 x[2,10]
   - 86.39229978440753 x[3,10] - 57.96057858487916 x[4,10]
   - 34.14174665045831 x[5,10] - 34.56896120543755 x[6,10]
   - 66.39242739679824 x[7,10] - 86.39229978440753 x[8,10]
   - 57.96057858487916 x[9,10] - 34.14174665045831 x[10,10]
   - 34.56896120543755 x[11,10] - 66.39242739679824 x[12,10]
   - 86.39229978440753 x[13,10] - 57.96057858487916 x[14,10]
   - 34.14174665045831 x[15,10] - 34.56896120543755 x[16,10]
   - 66.39242739679824 x[17,10] - 86.39229978440753 x[18,10]
   - 57.96057858487916 x[19,10] - 34.14174665045831 x[20,10]
   - 34.56896120543755 x[21,10] - 66.39242739679824 x[22,10]
   - 34.14174665045831 x[23,10] - 34.14174665045831 x[24,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 23.88912203495715 x[0,11]
   - 23.88912203495715 x[1,11] - 23.88912203495715 x[2,11]
   - 92.48240634897112 x[3,11] - 62.10828358990555 x[4,11]
   - 44.82158582093871 x[5,11] - 23.88912203495715 x[6,11]
   - 60.68586291448496 x[7,11] - 92.48240634897112 x[8,11]
   - 62.10828358990555 x[9,11] - 44.82158582093871 x[10,11]
   - 23.88912203495715 x[11,11] - 60.68586291448496 x[12,11]
   - 92.48240634897112 x[13,11] - 62.10828358990555 x[14,11]
   - 44.82158582093871 x[15,11] - 23.88912203495715 x[16,11]
   - 60.68586291448496 x[17,11] - 92.48240634897112 x[18,11]
   - 62.10828358990555 x[19,11] - 44.82158582093871 x[20,11]
   - 23.88912203495715 x[21,11] - 60.68586291448496 x[22,11]
   - 23.88912203495715 x[23,11] - 23.88912203495715 x[24,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 25.16924652494773 x[0,12]
   - 25.16924652494773 x[1,12] - 25.16924652494773 x[2,12]
   - 127.2305279841082 x[3,12] - 69.70140051124403 x[4,12]
   - 75.36492752230883 x[5,12] - 59.40573842449439 x[6,12]
   - 25.16924652494773 x[7,12] - 127.2305279841082 x[8,12]
   - 69.70140051124403 x[9,12] - 75.36492752230883 x[10,12]
   - 59.40573842449439 x[11,12] - 25.16924652494773 x[12,12]
   - 127.2305279841082 x[13,12] - 69.70140051124403 x[14,12]
   - 75.36492752230883 x[15,12] - 59.40573842449439 x[16,12]
   - 25.16924652494773 x[17,12] - 127.2305279841082 x[18,12]
   - 69.70140051124403 x[19,12] - 75.36492752230883 x[20,12]
   - 59.40573842449439 x[21,12] - 25.16924652494773 x[22,12]
   - 25.16924652494773 x[23,12] - 25.16924652494773 x[24,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 57.34693511834693 x[0,13]
   - 57.34693511834693 x[1,13] - 57.34693511834693 x[2,13]
   - 57.34693511834693 x[3,13] - 84.75755102827145 x[4,13]
   - 63.18711131651891 x[5,13] - 59.02459326558134 x[6,13]
   - 95.05283939070904 x[7,13] - 57.34693511834693 x[8,13]
   - 84.75755102827145 x[9,13] - 63.18711131651891 x[10,13]
   - 59.02459326558134 x[11,13] - 95.05283939070904 x[12,13]
   - 57.34693511834693 x[13,13] - 84.75755102827145 x[14,13]
   - 63.18711131651891 x[15,13] - 59.02459326558134 x[16,13]
   - 95.05283939070904 x[17,13] - 57.34693511834693 x[18,13]
   - 84.75755102827145 x[19,13] - 63.18711131651891 x[20,13]
   - 59.02459326558134 x[21,13] - 95.05283939070904 x[22,13]
   - 57.34693511834693 x[23,13] - 57.34693511834693 x[24,13]
   - start_service[0] + start_service[13] >= 0
 LowerBoundArrivalVehicle[14]: - 38.1205987906384 x[0,14]
   - 38.1205987906384 x[1,14] - 38.1205987906384 x[2,14]
   - 103.98388735598 x[3,14] - 38.1205987906384 x[4,14]
   - 53.98172644469906 x[5,14] - 47.8768068342243 x[6,14]
   - 56.75004824555334 x[7,14] - 103.98388735598 x[8,14]
   - 38.1205987906384 x[9,14] - 53.98172644469906 x[10,14]
   - 47.8768068342243 x[11,14] - 56.75004824555334 x[12,14]
   - 103.98388735598 x[13,14] - 38.1205987906384 x[14,14]
   - 53.98172644469906 x[15,14] - 47.8768068342243 x[16,14]
   - 56.75004824555334 x[17,14] - 103.98388735598 x[18,14]
   - 38.1205987906384 x[19,14] - 53.98172644469906 x[20,14]
   - 47.8768068342243 x[21,14] - 56.75004824555334 x[22,14]
   - 38.1205987906384 x[23,14] - 38.1205987906384 x[24,14]
   - start_service[0] + start_service[14] >= 0
 LowerBoundArrivalVehicle[15]: - 34.14174665045831 x[0,15]
   - 34.14174665045831 x[1,15] - 34.14174665045831 x[2,15]
   - 86.39229978440753 x[3,15] - 57.96057858487916 x[4,15]
   - 34.14174665045831 x[5,15] - 34.56896120543755 x[6,15]
   - 66.39242739679824 x[7,15] - 86.39229978440753 x[8,15]
   - 57.96057858487916 x[9,15] - 34.14174665045831 x[10,15]
   - 34.56896120543755 x[11,15] - 66.39242739679824 x[12,15]
   - 86.39229978440753 x[13,15] - 57.96057858487916 x[14,15]
   - 34.14174665045831 x[15,15] - 34.56896120543755 x[16,15]
   - 66.39242739679824 x[17,15] - 86.39229978440753 x[18,15]
   - 57.96057858487916 x[19,15] - 34.14174665045831 x[20,15]
   - 34.56896120543755 x[21,15] - 66.39242739679824 x[22,15]
   - 34.14174665045831 x[23,15] - 34.14174665045831 x[24,15]
   - start_service[0] + start_service[15] >= 0
 LowerBoundArrivalVehicle[16]: - 23.88912203495715 x[0,16]
   - 23.88912203495715 x[1,16] - 23.88912203495715 x[2,16]
   - 92.48240634897112 x[3,16] - 62.10828358990555 x[4,16]
   - 44.82158582093871 x[5,16] - 23.88912203495715 x[6,16]
   - 60.68586291448496 x[7,16] - 92.48240634897112 x[8,16]
   - 62.10828358990555 x[9,16] - 44.82158582093871 x[10,16]
   - 23.88912203495715 x[11,16] - 60.68586291448496 x[12,16]
   - 92.48240634897112 x[13,16] - 62.10828358990555 x[14,16]
   - 44.82158582093871 x[15,16] - 23.88912203495715 x[16,16]
   - 60.68586291448496 x[17,16] - 92.48240634897112 x[18,16]
   - 62.10828358990555 x[19,16] - 44.82158582093871 x[20,16]
   - 23.88912203495715 x[21,16] - 60.68586291448496 x[22,16]
   - 23.88912203495715 x[23,16] - 23.88912203495715 x[24,16]
   - start_service[0] + start_service[16] >= 0
 LowerBoundArrivalVehicle[17]: - 25.16924652494773 x[0,17]
   - 25.16924652494773 x[1,17] - 25.16924652494773 x[2,17]
   - 127.2305279841082 x[3,17] - 69.70140051124403 x[4,17]
   - 75.36492752230883 x[5,17] - 59.40573842449439 x[6,17]
   - 25.16924652494773 x[7,17] - 127.2305279841082 x[8,17]
   - 69.70140051124403 x[9,17] - 75.36492752230883 x[10,17]
   - 59.40573842449439 x[11,17] - 25.16924652494773 x[12,17]
   - 127.2305279841082 x[13,17] - 69.70140051124403 x[14,17]
   - 75.36492752230883 x[15,17] - 59.40573842449439 x[16,17]
   - 25.16924652494773 x[17,17] - 127.2305279841082 x[18,17]
   - 69.70140051124403 x[19,17] - 75.36492752230883 x[20,17]
   - 59.40573842449439 x[21,17] - 25.16924652494773 x[22,17]
   - 25.16924652494773 x[23,17] - 25.16924652494773 x[24,17]
   - start_service[0] + start_service[17] >= 0
 LowerBoundArrivalVehicle[18]: - 57.34693511834693 x[0,18]
   - 57.34693511834693 x[1,18] - 57.34693511834693 x[2,18]
   - 57.34693511834693 x[3,18] - 84.75755102827145 x[4,18]
   - 63.18711131651891 x[5,18] - 59.02459326558134 x[6,18]
   - 95.05283939070904 x[7,18] - 57.34693511834693 x[8,18]
   - 84.75755102827145 x[9,18] - 63.18711131651891 x[10,18]
   - 59.02459326558134 x[11,18] - 95.05283939070904 x[12,18]
   - 57.34693511834693 x[13,18] - 84.75755102827145 x[14,18]
   - 63.18711131651891 x[15,18] - 59.02459326558134 x[16,18]
   - 95.05283939070904 x[17,18] - 57.34693511834693 x[18,18]
   - 84.75755102827145 x[19,18] - 63.18711131651891 x[20,18]
   - 59.02459326558134 x[21,18] - 95.05283939070904 x[22,18]
   - 57.34693511834693 x[23,18] - 57.34693511834693 x[24,18]
   - start_service[0] + start_service[18] >= 0
 LowerBoundArrivalVehicle[19]: - 38.1205987906384 x[0,19]
   - 38.1205987906384 x[1,19] - 38.1205987906384 x[2,19]
   - 103.98388735598 x[3,19] - 38.1205987906384 x[4,19]
   - 53.98172644469906 x[5,19] - 47.8768068342243 x[6,19]
   - 56.75004824555334 x[7,19] - 103.98388735598 x[8,19]
   - 38.1205987906384 x[9,19] - 53.98172644469906 x[10,19]
   - 47.8768068342243 x[11,19] - 56.75004824555334 x[12,19]
   - 103.98388735598 x[13,19] - 38.1205987906384 x[14,19]
   - 53.98172644469906 x[15,19] - 47.8768068342243 x[16,19]
   - 56.75004824555334 x[17,19] - 103.98388735598 x[18,19]
   - 38.1205987906384 x[19,19] - 53.98172644469906 x[20,19]
   - 47.8768068342243 x[21,19] - 56.75004824555334 x[22,19]
   - 38.1205987906384 x[23,19] - 38.1205987906384 x[24,19]
   - start_service[0] + start_service[19] >= 0
 LowerBoundArrivalVehicle[20]: - 34.14174665045831 x[0,20]
   - 34.14174665045831 x[1,20] - 34.14174665045831 x[2,20]
   - 86.39229978440753 x[3,20] - 57.96057858487916 x[4,20]
   - 34.14174665045831 x[5,20] - 34.56896120543755 x[6,20]
   - 66.39242739679824 x[7,20] - 86.39229978440753 x[8,20]
   - 57.96057858487916 x[9,20] - 34.14174665045831 x[10,20]
   - 34.56896120543755 x[11,20] - 66.39242739679824 x[12,20]
   - 86.39229978440753 x[13,20] - 57.96057858487916 x[14,20]
   - 34.14174665045831 x[15,20] - 34.56896120543755 x[16,20]
   - 66.39242739679824 x[17,20] - 86.39229978440753 x[18,20]
   - 57.96057858487916 x[19,20] - 34.14174665045831 x[20,20]
   - 34.56896120543755 x[21,20] - 66.39242739679824 x[22,20]
   - 34.14174665045831 x[23,20] - 34.14174665045831 x[24,20]
   - start_service[0] + start_service[20] >= 0
 LowerBoundArrivalVehicle[21]: - 23.88912203495715 x[0,21]
   - 23.88912203495715 x[1,21] - 23.88912203495715 x[2,21]
   - 92.48240634897112 x[3,21] - 62.10828358990555 x[4,21]
   - 44.82158582093871 x[5,21] - 23.88912203495715 x[6,21]
   - 60.68586291448496 x[7,21] - 92.48240634897112 x[8,21]
   - 62.10828358990555 x[9,21] - 44.82158582093871 x[10,21]
   - 23.88912203495715 x[11,21] - 60.68586291448496 x[12,21]
   - 92.48240634897112 x[13,21] - 62.10828358990555 x[14,21]
   - 44.82158582093871 x[15,21] - 23.88912203495715 x[16,21]
   - 60.68586291448496 x[17,21] - 92.48240634897112 x[18,21]
   - 62.10828358990555 x[19,21] - 44.82158582093871 x[20,21]
   - 23.88912203495715 x[21,21] - 60.68586291448496 x[22,21]
   - 23.88912203495715 x[23,21] - 23.88912203495715 x[24,21]
   - start_service[0] + start_service[21] >= 0
 LowerBoundArrivalVehicle[22]: - 25.16924652494773 x[0,22]
   - 25.16924652494773 x[1,22] - 25.16924652494773 x[2,22]
   - 127.2305279841082 x[3,22] - 69.70140051124403 x[4,22]
   - 75.36492752230883 x[5,22] - 59.40573842449439 x[6,22]
   - 25.16924652494773 x[7,22] - 127.2305279841082 x[8,22]
   - 69.70140051124403 x[9,22] - 75.36492752230883 x[10,22]
   - 59.40573842449439 x[11,22] - 25.16924652494773 x[12,22]
   - 127.2305279841082 x[13,22] - 69.70140051124403 x[14,22]
   - 75.36492752230883 x[15,22] - 59.40573842449439 x[16,22]
   - 25.16924652494773 x[17,22] - 127.2305279841082 x[18,22]
   - 69.70140051124403 x[19,22] - 75.36492752230883 x[20,22]
   - 59.40573842449439 x[21,22] - 25.16924652494773 x[22,22]
   - 25.16924652494773 x[23,22] - 25.16924652494773 x[24,22]
   - start_service[0] + start_service[22] >= 0
 LowerBoundArrivalVehicle[23]: - 0.02 x[0,23] - 0.02 x[1,23] - 0.02 x[2,23]
   - 114.6738702366939 x[3,23] - 76.22119758127681 x[4,23]
   - 68.26349330091662 x[5,23] - 47.7582440699143 x[6,23]
   - 50.31849304989545 x[7,23] - 114.6738702366939 x[8,23]
   - 76.22119758127681 x[9,23] - 68.26349330091662 x[10,23]
   - 47.7582440699143 x[11,23] - 50.31849304989545 x[12,23]
   - 114.6738702366939 x[13,23] - 76.22119758127681 x[14,23]
   - 68.26349330091662 x[15,23] - 47.7582440699143 x[16,23]
   - 50.31849304989545 x[17,23] - 114.6738702366939 x[18,23]
   - 76.22119758127681 x[19,23] - 68.26349330091662 x[20,23]
   - 47.7582440699143 x[21,23] - 50.31849304989545 x[22,23] - 0.02 x[23,23]
   - 0.02 x[24,23] - start_service[0] + start_service[23] >= 0
 LowerBoundArrivalVehicle[24]: - 0.02 x[0,24] - 0.02 x[1,24] - 0.02 x[2,24]
   - 114.6738702366939 x[3,24] - 76.22119758127681 x[4,24]
   - 68.26349330091662 x[5,24] - 47.7582440699143 x[6,24]
   - 50.31849304989545 x[7,24] - 114.6738702366939 x[8,24]
   - 76.22119758127681 x[9,24] - 68.26349330091662 x[10,24]
   - 47.7582440699143 x[11,24] - 50.31849304989545 x[12,24]
   - 114.6738702366939 x[13,24] - 76.22119758127681 x[14,24]
   - 68.26349330091662 x[15,24] - 47.7582440699143 x[16,24]
   - 50.31849304989545 x[17,24] - 114.6738702366939 x[18,24]
   - 76.22119758127681 x[19,24] - 68.26349330091662 x[20,24]
   - 47.7582440699143 x[21,24] - 50.31849304989545 x[22,24] - 0.02 x[23,24]
   - 0.02 x[24,24] - start_service[0] + start_service[24] >= 0
 LowerBoundArrivalVehicle[25]: - 0.02 x[0,25] - 0.02 x[1,25] - 0.02 x[2,25]
   - 114.6738702366939 x[3,25] - 76.22119758127681 x[4,25]
   - 68.26349330091662 x[5,25] - 47.7582440699143 x[6,25]
   - 50.31849304989545 x[7,25] - 114.6738702366939 x[8,25]
   - 76.22119758127681 x[9,25] - 68.26349330091662 x[10,25]
   - 47.7582440699143 x[11,25] - 50.31849304989545 x[12,25]
   - 114.6738702366939 x[13,25] - 76.22119758127681 x[14,25]
   - 68.26349330091662 x[15,25] - 47.7582440699143 x[16,25]
   - 50.31849304989545 x[17,25] - 114.6738702366939 x[18,25]
   - 76.22119758127681 x[19,25] - 68.26349330091662 x[20,25]
   - 47.7582440699143 x[21,25] - 50.31849304989545 x[22,25] - 0.02 x[23,25]
   - 0.02 x[24,25] - start_service[0] + start_service[25] >= 0
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
 R3337: x[0,3] + x[1,3] + x[2,3] + x[3,3] + x[4,3] + x[5,3] + x[6,3]
   + x[7,3] + x[8,3] + x[9,3] + x[10,3] + x[11,3] + x[12,3] + x[13,3]
   + x[14,3] + x[15,3] + x[16,3] + x[17,3] + x[18,3] + x[19,3] + x[20,3]
   + x[21,3] + x[22,3] + x[23,3] + x[24,3] + x[25,3] = 1
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
