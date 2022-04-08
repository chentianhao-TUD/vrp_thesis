\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[15,0] + 0 x[15,1] + 0 x[15,2] + 0 x[15,3] + 0 x[15,4]
   + 0 x[15,5] + 0 x[15,6] + 0 x[15,7] + 0 x[15,8] + 0 x[15,9] + 0 x[15,10]
   + 0 x[15,11] + 0 x[15,12] + 0 x[15,13] + 0 x[15,14] + 0 x[15,15]
   + 0 w[15,0] + 0 w[15,1] + 0 w[15,2] + 0 w[15,3] + 0 w[15,4] + 0 w[15,5]
   + 0 w[15,6] + 0 w[15,7] + 0 w[15,13] + 0 w[15,14] + 0 w[15,15]
   + 0 v[0,0] + 0 v[1,0] + 0 v[2,0] + 0 v[3,0] + 0 v[4,0] + 0 v[5,0]
   + 0 v[6,0] + 0 v[7,0] + 0 v[8,0] + 0 v[9,0] + 0 v[10,0] + 0 v[11,0]
   + 0 v[12,0] + 0 v[13,0] + 0 v[14,0] + 0 v[15,0] + 0 v[15,1] + 0 v[15,2]
   + 0 v[15,3] + 0 v[15,4] + 0 v[15,5] + 0 v[15,6] + 0 v[15,7] + 0 v[15,8]
   + 0 v[15,9] + 0 v[15,10] + 0 v[15,11] + 0 v[15,12] + 0 v[15,13]
   + 0 v[15,14] + 0 v[15,15] + 0 service_time[0] + 0 service_time[1]
   + 0 service_time[2] + 0 service_time[8] + 0 service_time[9]
   + 0 service_time[10] + 0 service_time[11] + 0 service_time[12]
   + 0 service_time[13] + 0 service_time[14] + 0 service_time[15]
   + start_service[15]
Subject To
 MaxNumberVehiclesStart: x[0,1] + x[0,2] + x[0,3] + x[0,4] + x[0,5]
   + x[0,6] + x[0,7] + x[0,8] + x[0,9] + x[0,10] + x[0,11] + x[0,12]
   + x[0,13] + x[0,14] + x[0,15] <= 2
 OutFlowVehicles[1]: x[1,1] + x[1,2] + x[1,3] + x[1,4] + x[1,5] + x[1,6]
   + x[1,7] + x[1,8] + x[1,9] + x[1,10] + x[1,11] + x[1,12] + x[1,13]
   + x[1,14] + x[1,15] = 1
 OutFlowVehicles[2]: x[2,1] + x[2,2] + x[2,3] + x[2,4] + x[2,5] + x[2,6]
   + x[2,7] + x[2,8] + x[2,9] + x[2,10] + x[2,11] + x[2,12] + x[2,13]
   + x[2,14] + x[2,15] = 1
 OutFlowVehicles[3]: x[3,1] + x[3,2] + x[3,3] + x[3,4] + x[3,5] + x[3,6]
   + x[3,7] + x[3,8] + x[3,9] + x[3,10] + x[3,11] + x[3,12] + x[3,13]
   + x[3,14] + x[3,15] = 1
 OutFlowVehicles[4]: x[4,1] + x[4,2] + x[4,3] + x[4,4] + x[4,5] + x[4,6]
   + x[4,7] + x[4,8] + x[4,9] + x[4,10] + x[4,11] + x[4,12] + x[4,13]
   + x[4,14] + x[4,15] = 1
 OutFlowVehicles[5]: x[5,1] + x[5,2] + x[5,3] + x[5,4] + x[5,5] + x[5,6]
   + x[5,7] + x[5,8] + x[5,9] + x[5,10] + x[5,11] + x[5,12] + x[5,13]
   + x[5,14] + x[5,15] = 1
 OutFlowVehicles[6]: x[6,1] + x[6,2] + x[6,3] + x[6,4] + x[6,5] + x[6,6]
   + x[6,7] + x[6,8] + x[6,9] + x[6,10] + x[6,11] + x[6,12] + x[6,13]
   + x[6,14] + x[6,15] = 1
 OutFlowVehicles[7]: x[7,1] + x[7,2] + x[7,3] + x[7,4] + x[7,5] + x[7,6]
   + x[7,7] + x[7,8] + x[7,9] + x[7,10] + x[7,11] + x[7,12] + x[7,13]
   + x[7,14] + x[7,15] = 1
 OutFlowVehicles[8]: x[8,1] + x[8,2] + x[8,3] + x[8,4] + x[8,5] + x[8,6]
   + x[8,7] + x[8,8] + x[8,9] + x[8,10] + x[8,11] + x[8,12] + x[8,13]
   + x[8,14] + x[8,15] = 1
 OutFlowVehicles[9]: x[9,1] + x[9,2] + x[9,3] + x[9,4] + x[9,5] + x[9,6]
   + x[9,7] + x[9,8] + x[9,9] + x[9,10] + x[9,11] + x[9,12] + x[9,13]
   + x[9,14] + x[9,15] = 1
 OutFlowVehicles[10]: x[10,1] + x[10,2] + x[10,3] + x[10,4] + x[10,5]
   + x[10,6] + x[10,7] + x[10,8] + x[10,9] + x[10,10] + x[10,11] + x[10,12]
   + x[10,13] + x[10,14] + x[10,15] = 1
 OutFlowVehicles[11]: x[11,1] + x[11,2] + x[11,3] + x[11,4] + x[11,5]
   + x[11,6] + x[11,7] + x[11,8] + x[11,9] + x[11,10] + x[11,11] + x[11,12]
   + x[11,13] + x[11,14] + x[11,15] = 1
 OutFlowVehicles[12]: x[12,1] + x[12,2] + x[12,3] + x[12,4] + x[12,5]
   + x[12,6] + x[12,7] + x[12,8] + x[12,9] + x[12,10] + x[12,11] + x[12,12]
   + x[12,13] + x[12,14] + x[12,15] = 1
 OutFlowVehicles[13]: x[13,1] + x[13,2] + x[13,3] + x[13,4] + x[13,5]
   + x[13,6] + x[13,7] + x[13,8] + x[13,9] + x[13,10] + x[13,11] + x[13,12]
   + x[13,13] + x[13,14] + x[13,15] = 1
 OutFlowVehicles[14]: x[14,1] + x[14,2] + x[14,3] + x[14,4] + x[14,5]
   + x[14,6] + x[14,7] + x[14,8] + x[14,9] + x[14,10] + x[14,11] + x[14,12]
   + x[14,13] + x[14,14] + x[14,15] = 1
 InFlowVehicles[1]: x[0,1] + x[1,1] + x[2,1] + x[3,1] + x[4,1] + x[5,1]
   + x[6,1] + x[7,1] + x[8,1] + x[9,1] + x[10,1] + x[11,1] + x[12,1]
   + x[13,1] + x[14,1] = 1
 InFlowVehicles[2]: x[0,2] + x[1,2] + x[2,2] + x[3,2] + x[4,2] + x[5,2]
   + x[6,2] + x[7,2] + x[8,2] + x[9,2] + x[10,2] + x[11,2] + x[12,2]
   + x[13,2] + x[14,2] = 1
 InFlowVehicles[3]: x[0,3] + x[1,3] + x[2,3] + x[3,3] + x[4,3] + x[5,3]
   + x[6,3] + x[7,3] + x[8,3] + x[9,3] + x[10,3] + x[11,3] + x[12,3]
   + x[13,3] + x[14,3] = 1
 InFlowVehicles[4]: x[0,4] + x[1,4] + x[2,4] + x[3,4] + x[4,4] + x[5,4]
   + x[6,4] + x[7,4] + x[8,4] + x[9,4] + x[10,4] + x[11,4] + x[12,4]
   + x[13,4] + x[14,4] = 1
 InFlowVehicles[5]: x[0,5] + x[1,5] + x[2,5] + x[3,5] + x[4,5] + x[5,5]
   + x[6,5] + x[7,5] + x[8,5] + x[9,5] + x[10,5] + x[11,5] + x[12,5]
   + x[13,5] + x[14,5] = 1
 InFlowVehicles[6]: x[0,6] + x[1,6] + x[2,6] + x[3,6] + x[4,6] + x[5,6]
   + x[6,6] + x[7,6] + x[8,6] + x[9,6] + x[10,6] + x[11,6] + x[12,6]
   + x[13,6] + x[14,6] = 1
 InFlowVehicles[7]: x[0,7] + x[1,7] + x[2,7] + x[3,7] + x[4,7] + x[5,7]
   + x[6,7] + x[7,7] + x[8,7] + x[9,7] + x[10,7] + x[11,7] + x[12,7]
   + x[13,7] + x[14,7] = 1
 InFlowVehicles[8]: x[0,8] + x[1,8] + x[2,8] + x[3,8] + x[4,8] + x[5,8]
   + x[6,8] + x[7,8] + x[8,8] + x[9,8] + x[10,8] + x[11,8] + x[12,8]
   + x[13,8] + x[14,8] = 1
 InFlowVehicles[9]: x[0,9] + x[1,9] + x[2,9] + x[3,9] + x[4,9] + x[5,9]
   + x[6,9] + x[7,9] + x[8,9] + x[9,9] + x[10,9] + x[11,9] + x[12,9]
   + x[13,9] + x[14,9] = 1
 InFlowVehicles[10]: x[0,10] + x[1,10] + x[2,10] + x[3,10] + x[4,10]
   + x[5,10] + x[6,10] + x[7,10] + x[8,10] + x[9,10] + x[10,10] + x[11,10]
   + x[12,10] + x[13,10] + x[14,10] = 1
 InFlowVehicles[11]: x[0,11] + x[1,11] + x[2,11] + x[3,11] + x[4,11]
   + x[5,11] + x[6,11] + x[7,11] + x[8,11] + x[9,11] + x[10,11] + x[11,11]
   + x[12,11] + x[13,11] + x[14,11] = 1
 InFlowVehicles[12]: x[0,12] + x[1,12] + x[2,12] + x[3,12] + x[4,12]
   + x[5,12] + x[6,12] + x[7,12] + x[8,12] + x[9,12] + x[10,12] + x[11,12]
   + x[12,12] + x[13,12] + x[14,12] = 1
 InFlowVehicles[13]: x[0,13] + x[1,13] + x[2,13] + x[3,13] + x[4,13]
   + x[5,13] + x[6,13] + x[7,13] + x[8,13] + x[9,13] + x[10,13] + x[11,13]
   + x[12,13] + x[13,13] + x[14,13] = 1
 InFlowVehicles[14]: x[0,14] + x[1,14] + x[2,14] + x[3,14] + x[4,14]
   + x[5,14] + x[6,14] + x[7,14] + x[8,14] + x[9,14] + x[10,14] + x[11,14]
   + x[12,14] + x[13,14] + x[14,14] = 1
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - start_service[0]
   + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 999999 x[0,2] - start_service[0]
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
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999999
 NodeArrivalVehicles[0,14]: - 999999 x[0,14] - start_service[0]
   + start_service[14] >= -999999
 NodeArrivalVehicles[0,15]: - 999999 x[0,15] - start_service[0]
   + start_service[15] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
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
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999999
 NodeArrivalVehicles[1,14]: - 999999 x[1,14] - start_service[1]
   + start_service[14] >= -999999
 NodeArrivalVehicles[1,15]: - 999999 x[1,15] - start_service[1]
   + start_service[15] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999999
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
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
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999999
 NodeArrivalVehicles[2,14]: - 999999 x[2,14] - start_service[2]
   + start_service[14] >= -999999
 NodeArrivalVehicles[2,15]: - 999999 x[2,15] - start_service[2]
   + start_service[15] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000563369351183e+06 x[3,1]
   + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.0000563369351183e+06 x[3,2]
   + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0000456469522376e+06 x[3,4]
   - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.000028055364666e+06 x[3,5]
   - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.0000341454712306e+06 x[3,6]
   - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.0000688935928658e+06 x[3,7]
   - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0000456469522376e+06 x[3,9]
   - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.000028055364666e+06 x[3,10]
   - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.0000341454712306e+06 x[3,11]
   - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 1.0000688935928658e+06 x[3,12]
   - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 1.0000563369351183e+06 x[3,13]
   - start_service[3] + start_service[13] >= -999999
 NodeArrivalVehicles[3,14]: - 1.0000563369351183e+06 x[3,14]
   - start_service[3] + start_service[14] >= -999999
 NodeArrivalVehicles[3,15]: - 1.0000563369351183e+06 x[3,15]
   - start_service[3] + start_service[15] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000371105987907e+06 x[4,1]
   + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.0000371105987907e+06 x[4,2]
   + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0000456469522376e+06 x[4,3]
   + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.0000188499797942e+06 x[4,5]
   - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000229976847993e+06 x[4,6]
   - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000305908017206e+06 x[4,7]
   - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0000456469522376e+06 x[4,8]
   - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.0000188499797942e+06 x[4,10]
   - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000229976847993e+06 x[4,11]
   - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 1.0000305908017206e+06 x[4,12]
   - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 1.0000371105987907e+06 x[4,13]
   - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[4,14]: - 1.0000371105987907e+06 x[4,14]
   - start_service[4] + start_service[14] >= -999999
 NodeArrivalVehicles[4,15]: - 1.0000371105987907e+06 x[4,15]
   - start_service[4] + start_service[15] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000331317466504e+06 x[5,1]
   + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000331317466504e+06 x[5,2]
   + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.000028055364666e+06 x[5,3]
   + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.0000188499797942e+06 x[5,4]
   + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000096898391704e+06 x[5,6]
   - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000402331808718e+06 x[5,7]
   - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.000028055364666e+06 x[5,8]
   - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.0000188499797942e+06 x[5,9]
   - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000096898391704e+06 x[5,11]
   - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 1.0000402331808718e+06 x[5,12]
   - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 1.0000331317466504e+06 x[5,13]
   - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[5,14]: - 1.0000331317466504e+06 x[5,14]
   - start_service[5] + start_service[14] >= -999999
 NodeArrivalVehicles[5,15]: - 1.0000331317466504e+06 x[5,15]
   - start_service[5] + start_service[15] >= -999999
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
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000345266163895e+06 x[6,7]
   - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.0000341454712306e+06 x[6,8]
   - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000229976847993e+06 x[6,9]
   - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000096898391704e+06 x[6,10]
   - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 1.0000345266163895e+06 x[6,12]
   - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 1.000022879122035e+06 x[6,13]
   - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[6,14]: - 1.000022879122035e+06 x[6,14]
   - start_service[6] + start_service[14] >= -999999
 NodeArrivalVehicles[6,15]: - 1.000022879122035e+06 x[6,15]
   - start_service[6] + start_service[15] >= -999999
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
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.0000688935928658e+06 x[7,8]
   - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 1.0000305908017206e+06 x[7,9]
   - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.0000402331808718e+06 x[7,10]
   - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000345266163895e+06 x[7,11]
   - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 1.0000241592465249e+06 x[7,13]
   - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[7,14]: - 1.0000241592465249e+06 x[7,14]
   - start_service[7] + start_service[14] >= -999999
 NodeArrivalVehicles[7,15]: - 1.0000241592465249e+06 x[7,15]
   - start_service[7] + start_service[15] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000563369351183e+06 x[8,1]
   + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.0000563369351183e+06 x[8,2]
   + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0000456469522376e+06 x[8,4]
   + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.000028055364666e+06 x[8,5]
   + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.0000341454712306e+06 x[8,6]
   + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.0000688935928658e+06 x[8,7]
   + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0000456469522376e+06 x[8,9]
   - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.000028055364666e+06 x[8,10]
   - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.0000341454712306e+06 x[8,11]
   - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 1.0000688935928658e+06 x[8,12]
   - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 1.0000563369351183e+06 x[8,13]
   - start_service[8] + start_service[13] >= -999999
 NodeArrivalVehicles[8,14]: - 1.0000563369351183e+06 x[8,14]
   - start_service[8] + start_service[14] >= -999999
 NodeArrivalVehicles[8,15]: - 1.0000563369351183e+06 x[8,15]
   - start_service[8] + start_service[15] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0000371105987907e+06 x[9,1]
   + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000371105987907e+06 x[9,2]
   + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0000456469522376e+06 x[9,3]
   + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.0000188499797942e+06 x[9,5]
   + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000229976847993e+06 x[9,6]
   + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 1.0000305908017206e+06 x[9,7]
   + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0000456469522376e+06 x[9,8]
   + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.0000188499797942e+06 x[9,10]
   - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000229976847993e+06 x[9,11]
   - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 1.0000305908017206e+06 x[9,12]
   - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 1.0000371105987907e+06 x[9,13]
   - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[9,14]: - 1.0000371105987907e+06 x[9,14]
   - start_service[9] + start_service[14] >= -999999
 NodeArrivalVehicles[9,15]: - 1.0000371105987907e+06 x[9,15]
   - start_service[9] + start_service[15] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000331317466504e+06 x[10,1]
   + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000331317466504e+06 x[10,2]
   + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.000028055364666e+06 x[10,3]
   + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.0000188499797942e+06 x[10,4]
   + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000096898391704e+06 x[10,6]
   + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.0000402331808718e+06 x[10,7]
   + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.000028055364666e+06 x[10,8]
   + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.0000188499797942e+06 x[10,9]
   + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000096898391704e+06 x[10,11]
   - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 1.0000402331808718e+06 x[10,12]
   - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 1.0000331317466504e+06 x[10,13]
   - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[10,14]: - 1.0000331317466504e+06 x[10,14]
   - start_service[10] + start_service[14] >= -999999
 NodeArrivalVehicles[10,15]: - 1.0000331317466504e+06 x[10,15]
   - start_service[10] + start_service[15] >= -999999
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
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000345266163895e+06 x[11,7]
   + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.0000341454712306e+06 x[11,8]
   + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000229976847993e+06 x[11,9]
   + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000096898391704e+06 x[11,10]
   + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 1.0000345266163895e+06 x[11,12]
   - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 1.000022879122035e+06 x[11,13]
   - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[11,14]: - 1.000022879122035e+06 x[11,14]
   - start_service[11] + start_service[14] >= -999999
 NodeArrivalVehicles[11,15]: - 1.000022879122035e+06 x[11,15]
   - start_service[11] + start_service[15] >= -999999
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
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000688935928658e+06 x[12,8]
   + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000305908017206e+06 x[12,9]
   + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000402331808718e+06 x[12,10]
   + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.0000345266163895e+06 x[12,11]
   + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 1.0000241592465249e+06 x[12,13]
   - start_service[12] + start_service[13] >= -999999
 NodeArrivalVehicles[12,14]: - 1.0000241592465249e+06 x[12,14]
   - start_service[12] + start_service[14] >= -999999
 NodeArrivalVehicles[12,15]: - 1.0000241592465249e+06 x[12,15]
   - start_service[12] + start_service[15] >= -999999
 NodeArrivalVehicles[13,1]: - 999999 x[13,1] + start_service[1]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,2]: - 999999 x[13,2] + start_service[2]
   - start_service[13] >= -999999
 NodeArrivalVehicles[13,3]: - 1.0000563369351183e+06 x[13,3]
   + start_service[3] - start_service[13] >= -999999
 NodeArrivalVehicles[13,4]: - 1.0000371105987907e+06 x[13,4]
   + start_service[4] - start_service[13] >= -999999
 NodeArrivalVehicles[13,5]: - 1.0000331317466504e+06 x[13,5]
   + start_service[5] - start_service[13] >= -999999
 NodeArrivalVehicles[13,6]: - 1.000022879122035e+06 x[13,6]
   + start_service[6] - start_service[13] >= -999999
 NodeArrivalVehicles[13,7]: - 1.0000241592465249e+06 x[13,7]
   + start_service[7] - start_service[13] >= -999999
 NodeArrivalVehicles[13,8]: - 1.0000563369351183e+06 x[13,8]
   + start_service[8] - start_service[13] >= -999999
 NodeArrivalVehicles[13,9]: - 1.0000371105987907e+06 x[13,9]
   + start_service[9] - start_service[13] >= -999999
 NodeArrivalVehicles[13,10]: - 1.0000331317466504e+06 x[13,10]
   + start_service[10] - start_service[13] >= -999999
 NodeArrivalVehicles[13,11]: - 1.000022879122035e+06 x[13,11]
   + start_service[11] - start_service[13] >= -999999
 NodeArrivalVehicles[13,12]: - 1.0000241592465249e+06 x[13,12]
   + start_service[12] - start_service[13] >= -999999
 NodeArrivalVehicles[13,13]: - 999999 x[13,13] >= -999999
 NodeArrivalVehicles[13,14]: - 999999 x[13,14] - start_service[13]
   + start_service[14] >= -999999
 NodeArrivalVehicles[13,15]: - 999999 x[13,15] - start_service[13]
   + start_service[15] >= -999999
 NodeArrivalVehicles[14,1]: - 999999 x[14,1] + start_service[1]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,2]: - 999999 x[14,2] + start_service[2]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,3]: - 1.0000563369351183e+06 x[14,3]
   + start_service[3] - start_service[14] >= -999999
 NodeArrivalVehicles[14,4]: - 1.0000371105987907e+06 x[14,4]
   + start_service[4] - start_service[14] >= -999999
 NodeArrivalVehicles[14,5]: - 1.0000331317466504e+06 x[14,5]
   + start_service[5] - start_service[14] >= -999999
 NodeArrivalVehicles[14,6]: - 1.000022879122035e+06 x[14,6]
   + start_service[6] - start_service[14] >= -999999
 NodeArrivalVehicles[14,7]: - 1.0000241592465249e+06 x[14,7]
   + start_service[7] - start_service[14] >= -999999
 NodeArrivalVehicles[14,8]: - 1.0000563369351183e+06 x[14,8]
   + start_service[8] - start_service[14] >= -999999
 NodeArrivalVehicles[14,9]: - 1.0000371105987907e+06 x[14,9]
   + start_service[9] - start_service[14] >= -999999
 NodeArrivalVehicles[14,10]: - 1.0000331317466504e+06 x[14,10]
   + start_service[10] - start_service[14] >= -999999
 NodeArrivalVehicles[14,11]: - 1.000022879122035e+06 x[14,11]
   + start_service[11] - start_service[14] >= -999999
 NodeArrivalVehicles[14,12]: - 1.0000241592465249e+06 x[14,12]
   + start_service[12] - start_service[14] >= -999999
 NodeArrivalVehicles[14,13]: - 999999 x[14,13] + start_service[13]
   - start_service[14] >= -999999
 NodeArrivalVehicles[14,14]: - 999999 x[14,14] >= -999999
 NodeArrivalVehicles[14,15]: - 999999 x[14,15] - start_service[14]
   + start_service[15] >= -999999
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[0,14] + w[0,15] <= 4
 TotalRobotStarts: w[0,0] + w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[0,14] + w[0,15] + w[1,0] + w[1,1] + w[1,2] + w[1,3]
   + w[1,4] + w[1,5] + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10]
   + w[1,11] + w[1,12] + w[1,13] + w[1,14] + w[1,15] + w[2,0] + w[2,1]
   + w[2,2] + w[2,3] + w[2,4] + w[2,5] + w[2,6] + w[2,7] + w[2,8] + w[2,9]
   + w[2,10] + w[2,11] + w[2,12] + w[2,13] + w[2,14] + w[2,15] <= 7
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] + w[1,14] + w[1,15] + w[2,1] + w[2,2] + w[2,3] + w[2,4]
   + w[2,5] + w[2,6] + w[2,7] + w[2,8] + w[2,9] + w[2,10] + w[2,11]
   + w[2,12] + w[2,13] + w[2,14] + w[2,15] <= 3
 RobotsFinishAtVehicleDepot: w[0,15] + w[1,15] + w[2,15] + w[3,15]
   + w[4,15] + w[5,15] + w[6,15] + w[7,15] + w[8,15] + w[9,15] + w[10,15]
   + w[11,15] + w[12,15] + w[13,15] + w[14,15] <= 4
 TotalRobotFinishes: w[0,13] + w[0,14] + w[0,15] + w[1,13] + w[1,14]
   + w[1,15] + w[2,13] + w[2,14] + w[2,15] + w[3,13] + w[3,14] + w[3,15]
   + w[4,13] + w[4,14] + w[4,15] + w[5,13] + w[5,14] + w[5,15] + w[6,13]
   + w[6,14] + w[6,15] + w[7,13] + w[7,14] + w[7,15] + w[8,13] + w[8,14]
   + w[8,15] + w[9,13] + w[9,14] + w[9,15] + w[10,13] + w[10,14] + w[10,15]
   + w[11,13] + w[11,14] + w[11,15] + w[12,13] + w[12,14] + w[12,15] <= 7
 FlowConservationRobots[3]: w[0,3] + w[1,3] + w[2,3] - w[3,1] - w[3,2]
   - w[3,4] - w[3,5] - w[3,6] - w[3,7] - w[3,8] - w[3,9] - w[3,10]
   - w[3,11] - w[3,12] - w[3,13] - w[3,14] - w[3,15] + w[4,3] + w[5,3]
   + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3] + w[12,3]
   + w[13,3] + w[14,3] = 0
 FlowConservationRobots[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] - w[4,1]
   - w[4,2] - w[4,3] - w[4,5] - w[4,6] - w[4,7] - w[4,8] - w[4,9] - w[4,10]
   - w[4,11] - w[4,12] - w[4,13] - w[4,14] - w[4,15] + w[5,4] + w[6,4]
   + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4] + w[12,4] + w[13,4]
   + w[14,4] = 0
 FlowConservationRobots[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   - w[5,1] - w[5,2] - w[5,3] - w[5,4] - w[5,6] - w[5,7] - w[5,8] - w[5,9]
   - w[5,10] - w[5,11] - w[5,12] - w[5,13] - w[5,14] - w[5,15] + w[6,5]
   + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5] + w[12,5] + w[13,5]
   + w[14,5] = 0
 FlowConservationRobots[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] - w[6,1] - w[6,2] - w[6,3] - w[6,4] - w[6,5] - w[6,7] - w[6,8]
   - w[6,9] - w[6,10] - w[6,11] - w[6,12] - w[6,13] - w[6,14] - w[6,15]
   + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6] + w[12,6] + w[13,6]
   + w[14,6] = 0
 FlowConservationRobots[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] - w[7,1] - w[7,2] - w[7,3] - w[7,4] - w[7,5] - w[7,6]
   - w[7,8] - w[7,9] - w[7,10] - w[7,11] - w[7,12] - w[7,13] - w[7,14]
   - w[7,15] + w[8,7] + w[9,7] + w[10,7] + w[11,7] + w[12,7] + w[13,7]
   + w[14,7] = 0
 FlowConservationRobots[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] - w[8,1] - w[8,2] - w[8,3] - w[8,4] - w[8,5]
   - w[8,6] - w[8,7] - w[8,9] - w[8,10] - w[8,11] - w[8,12] - w[8,13]
   - w[8,14] - w[8,15] + w[9,8] + w[10,8] + w[11,8] + w[12,8] + w[13,8]
   + w[14,8] = 0
 FlowConservationRobots[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] - w[9,1] - w[9,2] - w[9,3] - w[9,4]
   - w[9,5] - w[9,6] - w[9,7] - w[9,8] - w[9,10] - w[9,11] - w[9,12]
   - w[9,13] - w[9,14] - w[9,15] + w[10,9] + w[11,9] + w[12,9] + w[13,9]
   + w[14,9] = 0
 FlowConservationRobots[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] - w[10,1]
   - w[10,2] - w[10,3] - w[10,4] - w[10,5] - w[10,6] - w[10,7] - w[10,8]
   - w[10,9] - w[10,11] - w[10,12] - w[10,13] - w[10,14] - w[10,15]
   + w[11,10] + w[12,10] + w[13,10] + w[14,10] = 0
 FlowConservationRobots[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   - w[11,1] - w[11,2] - w[11,3] - w[11,4] - w[11,5] - w[11,6] - w[11,7]
   - w[11,8] - w[11,9] - w[11,10] - w[11,12] - w[11,13] - w[11,14]
   - w[11,15] + w[12,11] + w[13,11] + w[14,11] = 0
 FlowConservationRobots[12]: w[0,12] + w[1,12] + w[2,12] + w[3,12]
   + w[4,12] + w[5,12] + w[6,12] + w[7,12] + w[8,12] + w[9,12] + w[10,12]
   + w[11,12] - w[12,1] - w[12,2] - w[12,3] - w[12,4] - w[12,5] - w[12,6]
   - w[12,7] - w[12,8] - w[12,9] - w[12,10] - w[12,11] - w[12,13]
   - w[12,14] - w[12,15] + w[13,12] + w[14,12] = 0
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
 RobotsEndService[3]: - service_time[3] - start_service[3]
   + start_service[8] >= 0
 RobotsEndService[4]: - service_time[4] - start_service[4]
   + start_service[9] >= 0
 RobotsEndService[5]: - service_time[5] - start_service[5]
   + start_service[10] >= 0
 RobotsEndService[6]: - service_time[6] - start_service[6]
   + start_service[11] >= 0
 RobotsEndService[7]: - service_time[7] - start_service[7]
   + start_service[12] >= 0
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
 VehicleRobotSynchronization1[3,13]: - 2 x[3,13] + w[3,13] <= 0
 VehicleRobotSynchronization1[3,14]: - 2 x[3,14] + w[3,14] <= 0
 VehicleRobotSynchronization1[3,15]: - 2 x[3,15] + w[3,15] <= 0
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
 VehicleRobotSynchronization1[4,14]: - 2 x[4,14] + w[4,14] <= 0
 VehicleRobotSynchronization1[4,15]: - 2 x[4,15] + w[4,15] <= 0
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
 VehicleRobotSynchronization1[5,15]: - 2 x[5,15] + w[5,15] <= 0
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
 VehicleRobotSynchronization1[8,13]: - 2 x[8,13] + w[8,13] <= 0
 VehicleRobotSynchronization1[8,14]: - 2 x[8,14] + w[8,14] <= 0
 VehicleRobotSynchronization1[8,15]: - 2 x[8,15] + w[8,15] <= 0
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
 VehicleRobotSynchronization1[9,14]: - 2 x[9,14] + w[9,14] <= 0
 VehicleRobotSynchronization1[9,15]: - 2 x[9,15] + w[9,15] <= 0
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
 VehicleRobotSynchronization1[10,15]: - 2 x[10,15] + w[10,15] <= 0
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
 VehicleRobotSynchronization1[13,0]: - 2 x[13,0] + w[13,0] <= 0
 VehicleRobotSynchronization1[13,1]: - 2 x[13,1] + w[13,1] <= 0
 VehicleRobotSynchronization1[13,2]: - 2 x[13,2] + w[13,2] <= 0
 VehicleRobotSynchronization1[13,3]: - 2 x[13,3] + w[13,3] <= 0
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
 VehicleRobotSynchronization1[14,0]: - 2 x[14,0] + w[14,0] <= 0
 VehicleRobotSynchronization1[14,1]: - 2 x[14,1] + w[14,1] <= 0
 VehicleRobotSynchronization1[14,2]: - 2 x[14,2] + w[14,2] <= 0
 VehicleRobotSynchronization1[14,3]: - 2 x[14,3] + w[14,3] <= 0
 VehicleRobotSynchronization1[14,4]: - 2 x[14,4] + w[14,4] <= 0
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
 SettingMode[3]: - w[3,8] + y[3,1] + 2 y[3,2] = 0
 SettingMode[4]: - w[4,9] + y[4,1] + 2 y[4,2] = 0
 SettingMode[5]: - w[5,10] + y[5,1] + 2 y[5,2] = 0
 SettingMode[6]: - w[6,11] + y[6,1] + 2 y[6,2] = 0
 SettingMode[7]: - w[7,12] + y[7,1] + 2 y[7,2] = 0
 SettingServiceTime[3]: - 290 y[3,1] - 145 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 380 y[4,1] - 190 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 470 y[5,1] - 235 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 250 y[6,1] - 125 y[6,2] + service_time[6] = 0
 SettingServiceTime[7]: - 450 y[7,1] - 225 y[7,2] + service_time[7] = 0
 AverageTimeRobots: - 8.190990731192418 w[0,3] - 5.444371255805486 w[0,4]
   - 4.87596380720833 w[0,5] - 3.411303147851021 w[0,6]
   - 3.594178074992532 w[0,7] - 8.190990731192418 w[0,8]
   - 5.444371255805486 w[0,9] - 4.87596380720833 w[0,10]
   - 3.411303147851021 w[0,11] - 3.594178074992532 w[0,12]
   - 8.190990731192418 w[1,3] - 5.444371255805486 w[1,4]
   - 4.87596380720833 w[1,5] - 3.411303147851021 w[1,6]
   - 3.594178074992532 w[1,7] - 8.190990731192418 w[1,8]
   - 5.444371255805486 w[1,9] - 4.87596380720833 w[1,10]
   - 3.411303147851021 w[1,11] - 3.594178074992532 w[1,12]
   - 8.190990731192418 w[2,3] - 5.444371255805486 w[2,4]
   - 4.87596380720833 w[2,5] - 3.411303147851021 w[2,6]
   - 3.594178074992532 w[2,7] - 8.190990731192418 w[2,8]
   - 5.444371255805486 w[2,9] - 4.87596380720833 w[2,10]
   - 3.411303147851021 w[2,11] - 3.594178074992532 w[2,12]
   - 8.190990731192418 w[3,1] - 8.190990731192418 w[3,2]
   - 6.663850319661865 w[3,4] - 4.1507663808658 w[3,5]
   - 5.020781604374885 w[3,6] - 9.984798980823046 w[3,7]
   - 6.663850319661865 w[3,9] - 4.1507663808658 w[3,10]
   - 5.020781604374885 w[3,11] - 9.984798980823046 w[3,12]
   - 8.190990731192418 w[3,13] - 8.190990731192418 w[3,14]
   - 8.190990731192418 w[3,15] - 5.444371255805486 w[4,1]
   - 5.444371255805486 w[4,2] - 6.663850319661865 w[4,3]
   - 2.83571139917725 w[4,5] - 3.428240685609592 w[4,6]
   - 4.512971674372231 w[4,7] - 6.663850319661865 w[4,8]
   - 2.83571139917725 w[4,10] - 3.428240685609592 w[4,11]
   - 4.512971674372231 w[4,12] - 5.444371255805486 w[4,13]
   - 5.444371255805486 w[4,14] - 5.444371255805486 w[4,15]
   - 4.87596380720833 w[5,1] - 4.87596380720833 w[5,2]
   - 4.1507663808658 w[5,3] - 2.83571139917725 w[5,4]
   - 1.5271198814972 w[5,6] - 5.890454410264359 w[5,7]
   - 4.1507663808658 w[5,8] - 2.83571139917725 w[5,9]
   - 1.5271198814972 w[5,11] - 5.890454410264359 w[5,12]
   - 4.87596380720833 w[5,13] - 4.87596380720833 w[5,14]
   - 4.87596380720833 w[5,15] - 3.411303147851021 w[6,1]
   - 3.411303147851021 w[6,2] - 5.020781604374885 w[6,3]
   - 3.428240685609592 w[6,4] - 1.5271198814972 w[6,5]
   - 5.075230912791034 w[6,7] - 5.020781604374885 w[6,8]
   - 3.428240685609592 w[6,9] - 1.5271198814972 w[6,10]
   - 5.075230912791034 w[6,12] - 3.411303147851021 w[6,13]
   - 3.411303147851021 w[6,14] - 3.411303147851021 w[6,15]
   - 3.594178074992532 w[7,1] - 3.594178074992532 w[7,2]
   - 9.984798980823046 w[7,3] - 4.512971674372231 w[7,4]
   - 5.890454410264359 w[7,5] - 5.075230912791034 w[7,6]
   - 9.984798980823046 w[7,8] - 4.512971674372231 w[7,9]
   - 5.890454410264359 w[7,10] - 5.075230912791034 w[7,11]
   - 3.594178074992532 w[7,13] - 3.594178074992532 w[7,14]
   - 3.594178074992532 w[7,15] - 8.190990731192418 w[8,1]
   - 8.190990731192418 w[8,2] - 6.663850319661865 w[8,4]
   - 4.1507663808658 w[8,5] - 5.020781604374885 w[8,6]
   - 9.984798980823046 w[8,7] - 6.663850319661865 w[8,9]
   - 4.1507663808658 w[8,10] - 5.020781604374885 w[8,11]
   - 9.984798980823046 w[8,12] - 8.190990731192418 w[8,13]
   - 8.190990731192418 w[8,14] - 8.190990731192418 w[8,15]
   - 5.444371255805486 w[9,1] - 5.444371255805486 w[9,2]
   - 6.663850319661865 w[9,3] - 2.83571139917725 w[9,5]
   - 3.428240685609592 w[9,6] - 4.512971674372231 w[9,7]
   - 6.663850319661865 w[9,8] - 2.83571139917725 w[9,10]
   - 3.428240685609592 w[9,11] - 4.512971674372231 w[9,12]
   - 5.444371255805486 w[9,13] - 5.444371255805486 w[9,14]
   - 5.444371255805486 w[9,15] - 4.87596380720833 w[10,1]
   - 4.87596380720833 w[10,2] - 4.1507663808658 w[10,3]
   - 2.83571139917725 w[10,4] - 1.5271198814972 w[10,6]
   - 5.890454410264359 w[10,7] - 4.1507663808658 w[10,8]
   - 2.83571139917725 w[10,9] - 1.5271198814972 w[10,11]
   - 5.890454410264359 w[10,12] - 4.87596380720833 w[10,13]
   - 4.87596380720833 w[10,14] - 4.87596380720833 w[10,15]
   - 3.411303147851021 w[11,1] - 3.411303147851021 w[11,2]
   - 5.020781604374885 w[11,3] - 3.428240685609592 w[11,4]
   - 1.5271198814972 w[11,5] - 5.075230912791034 w[11,7]
   - 5.020781604374885 w[11,8] - 3.428240685609592 w[11,9]
   - 1.5271198814972 w[11,10] - 5.075230912791034 w[11,12]
   - 3.411303147851021 w[11,13] - 3.411303147851021 w[11,14]
   - 3.411303147851021 w[11,15] - 3.594178074992532 w[12,1]
   - 3.594178074992532 w[12,2] - 9.984798980823046 w[12,3]
   - 4.512971674372231 w[12,4] - 5.890454410264359 w[12,5]
   - 5.075230912791034 w[12,6] - 9.984798980823046 w[12,8]
   - 4.512971674372231 w[12,9] - 5.890454410264359 w[12,10]
   - 5.075230912791034 w[12,11] - 3.594178074992532 w[12,13]
   - 3.594178074992532 w[12,14] - 3.594178074992532 w[12,15]
   - 8.190990731192418 w[13,3] - 5.444371255805486 w[13,4]
   - 4.87596380720833 w[13,5] - 3.411303147851021 w[13,6]
   - 3.594178074992532 w[13,7] - 8.190990731192418 w[13,8]
   - 5.444371255805486 w[13,9] - 4.87596380720833 w[13,10]
   - 3.411303147851021 w[13,11] - 3.594178074992532 w[13,12]
   - 8.190990731192418 w[14,3] - 5.444371255805486 w[14,4]
   - 4.87596380720833 w[14,5] - 3.411303147851021 w[14,6]
   - 3.594178074992532 w[14,7] - 8.190990731192418 w[14,8]
   - 5.444371255805486 w[14,9] - 4.87596380720833 w[14,10]
   - 3.411303147851021 w[14,11] - 3.594178074992532 w[14,12]
   - 41.42857142857142 y[3,1] - 20.71428571428571 y[3,2]
   - 54.28571428571428 y[4,1] - 27.14285714285714 y[4,2]
   - 67.14285714285714 y[5,1] - 33.57142857142857 y[5,2]
   - 35.71428571428572 y[6,1] - 17.85714285714286 y[6,2]
   - 64.28571428571428 y[7,1] - 32.14285714285714 y[7,2]
   + start_service[15] >= 0
 AverageTimeVehicles: - 28.66846755917346 x[0,3] - 19.0552993953192 x[0,4]
   - 17.06587332522916 x[0,5] - 11.93956101747858 x[0,6]
   - 12.57962326247386 x[0,7] - 28.66846755917346 x[0,8]
   - 19.0552993953192 x[0,9] - 17.06587332522916 x[0,10]
   - 11.93956101747858 x[0,11] - 12.57962326247386 x[0,12]
   - 28.66846755917346 x[1,3] - 19.0552993953192 x[1,4]
   - 17.06587332522916 x[1,5] - 11.93956101747858 x[1,6]
   - 12.57962326247386 x[1,7] - 28.66846755917346 x[1,8]
   - 19.0552993953192 x[1,9] - 17.06587332522916 x[1,10]
   - 11.93956101747858 x[1,11] - 12.57962326247386 x[1,12]
   - 28.66846755917346 x[2,3] - 19.0552993953192 x[2,4]
   - 17.06587332522916 x[2,5] - 11.93956101747858 x[2,6]
   - 12.57962326247386 x[2,7] - 28.66846755917346 x[2,8]
   - 19.0552993953192 x[2,9] - 17.06587332522916 x[2,10]
   - 11.93956101747858 x[2,11] - 12.57962326247386 x[2,12]
   - 28.66846755917346 x[3,1] - 28.66846755917346 x[3,2]
   - 23.32347611881653 x[3,4] - 14.5276823330303 x[3,5]
   - 17.5727356153121 x[3,6] - 34.94679643288066 x[3,7]
   - 23.32347611881653 x[3,9] - 14.5276823330303 x[3,10]
   - 17.5727356153121 x[3,11] - 34.94679643288066 x[3,12]
   - 28.66846755917346 x[3,13] - 28.66846755917346 x[3,14]
   - 28.66846755917346 x[3,15] - 19.0552993953192 x[4,1]
   - 19.0552993953192 x[4,2] - 23.32347611881653 x[4,3]
   - 9.924989897120374 x[4,5] - 11.99884239963357 x[4,6]
   - 15.79540086030281 x[4,7] - 23.32347611881653 x[4,8]
   - 9.924989897120374 x[4,10] - 11.99884239963357 x[4,11]
   - 15.79540086030281 x[4,12] - 19.0552993953192 x[4,13]
   - 19.0552993953192 x[4,14] - 19.0552993953192 x[4,15]
   - 17.06587332522916 x[5,1] - 17.06587332522916 x[5,2]
   - 14.5276823330303 x[5,3] - 9.924989897120374 x[5,4]
   - 5.344919585240199 x[5,6] - 20.61659043592526 x[5,7]
   - 14.5276823330303 x[5,8] - 9.924989897120374 x[5,9]
   - 5.344919585240199 x[5,11] - 20.61659043592526 x[5,12]
   - 17.06587332522916 x[5,13] - 17.06587332522916 x[5,14]
   - 17.06587332522916 x[5,15] - 11.93956101747858 x[6,1]
   - 11.93956101747858 x[6,2] - 17.5727356153121 x[6,3]
   - 11.99884239963357 x[6,4] - 5.344919585240199 x[6,5]
   - 17.76330819476862 x[6,7] - 17.5727356153121 x[6,8]
   - 11.99884239963357 x[6,9] - 5.344919585240199 x[6,10]
   - 17.76330819476862 x[6,12] - 11.93956101747858 x[6,13]
   - 11.93956101747858 x[6,14] - 11.93956101747858 x[6,15]
   - 12.57962326247386 x[7,1] - 12.57962326247386 x[7,2]
   - 34.94679643288066 x[7,3] - 15.79540086030281 x[7,4]
   - 20.61659043592526 x[7,5] - 17.76330819476862 x[7,6]
   - 34.94679643288066 x[7,8] - 15.79540086030281 x[7,9]
   - 20.61659043592526 x[7,10] - 17.76330819476862 x[7,11]
   - 12.57962326247386 x[7,13] - 12.57962326247386 x[7,14]
   - 12.57962326247386 x[7,15] - 28.66846755917346 x[8,1]
   - 28.66846755917346 x[8,2] - 23.32347611881653 x[8,4]
   - 14.5276823330303 x[8,5] - 17.5727356153121 x[8,6]
   - 34.94679643288066 x[8,7] - 23.32347611881653 x[8,9]
   - 14.5276823330303 x[8,10] - 17.5727356153121 x[8,11]
   - 34.94679643288066 x[8,12] - 28.66846755917346 x[8,13]
   - 28.66846755917346 x[8,14] - 28.66846755917346 x[8,15]
   - 19.0552993953192 x[9,1] - 19.0552993953192 x[9,2]
   - 23.32347611881653 x[9,3] - 9.924989897120374 x[9,5]
   - 11.99884239963357 x[9,6] - 15.79540086030281 x[9,7]
   - 23.32347611881653 x[9,8] - 9.924989897120374 x[9,10]
   - 11.99884239963357 x[9,11] - 15.79540086030281 x[9,12]
   - 19.0552993953192 x[9,13] - 19.0552993953192 x[9,14]
   - 19.0552993953192 x[9,15] - 17.06587332522916 x[10,1]
   - 17.06587332522916 x[10,2] - 14.5276823330303 x[10,3]
   - 9.924989897120374 x[10,4] - 5.344919585240199 x[10,6]
   - 20.61659043592526 x[10,7] - 14.5276823330303 x[10,8]
   - 9.924989897120374 x[10,9] - 5.344919585240199 x[10,11]
   - 20.61659043592526 x[10,12] - 17.06587332522916 x[10,13]
   - 17.06587332522916 x[10,14] - 17.06587332522916 x[10,15]
   - 11.93956101747858 x[11,1] - 11.93956101747858 x[11,2]
   - 17.5727356153121 x[11,3] - 11.99884239963357 x[11,4]
   - 5.344919585240199 x[11,5] - 17.76330819476862 x[11,7]
   - 17.5727356153121 x[11,8] - 11.99884239963357 x[11,9]
   - 5.344919585240199 x[11,10] - 17.76330819476862 x[11,12]
   - 11.93956101747858 x[11,13] - 11.93956101747858 x[11,14]
   - 11.93956101747858 x[11,15] - 12.57962326247386 x[12,1]
   - 12.57962326247386 x[12,2] - 34.94679643288066 x[12,3]
   - 15.79540086030281 x[12,4] - 20.61659043592526 x[12,5]
   - 17.76330819476862 x[12,6] - 34.94679643288066 x[12,8]
   - 15.79540086030281 x[12,9] - 20.61659043592526 x[12,10]
   - 17.76330819476862 x[12,11] - 12.57962326247386 x[12,13]
   - 12.57962326247386 x[12,14] - 12.57962326247386 x[12,15]
   - 28.66846755917346 x[13,3] - 19.0552993953192 x[13,4]
   - 17.06587332522916 x[13,5] - 11.93956101747858 x[13,6]
   - 12.57962326247386 x[13,7] - 28.66846755917346 x[13,8]
   - 19.0552993953192 x[13,9] - 17.06587332522916 x[13,10]
   - 11.93956101747858 x[13,11] - 12.57962326247386 x[13,12]
   - 28.66846755917346 x[14,3] - 19.0552993953192 x[14,4]
   - 17.06587332522916 x[14,5] - 11.93956101747858 x[14,6]
   - 12.57962326247386 x[14,7] - 28.66846755917346 x[14,8]
   - 19.0552993953192 x[14,9] - 17.06587332522916 x[14,10]
   - 11.93956101747858 x[14,11] - 12.57962326247386 x[14,12]
   + start_service[15] >= 0
 LowerBoundRoutingVehicle[0]: - 114.6738702366939 x[0,3]
   - 76.22119758127681 x[0,4] - 68.26349330091662 x[0,5]
   - 47.7582440699143 x[0,6] - 50.31849304989545 x[0,7]
   - 114.6738702366939 x[0,8] - 76.22119758127681 x[0,9]
   - 68.26349330091662 x[0,10] - 47.7582440699143 x[0,11]
   - 50.31849304989545 x[0,12] - start_service[0] + start_service[15] >= 0
 LowerBoundRoutingVehicle[1]: - 114.6738702366939 x[1,3]
   - 76.22119758127681 x[1,4] - 68.26349330091662 x[1,5]
   - 47.7582440699143 x[1,6] - 50.31849304989545 x[1,7]
   - 114.6738702366939 x[1,8] - 76.22119758127681 x[1,9]
   - 68.26349330091662 x[1,10] - 47.7582440699143 x[1,11]
   - 50.31849304989545 x[1,12] - start_service[1] + start_service[15] >= 0
 LowerBoundRoutingVehicle[2]: - 114.6738702366939 x[2,3]
   - 76.22119758127681 x[2,4] - 68.26349330091662 x[2,5]
   - 47.7582440699143 x[2,6] - 50.31849304989545 x[2,7]
   - 114.6738702366939 x[2,8] - 76.22119758127681 x[2,9]
   - 68.26349330091662 x[2,10] - 47.7582440699143 x[2,11]
   - 50.31849304989545 x[2,12] - start_service[2] + start_service[15] >= 0
 LowerBoundRoutingVehicle[3]: - 57.33693511834693 x[3,1]
   - 57.33693511834693 x[3,2] - 57.33693511834693 x[3,3]
   - 84.75755102827146 x[3,4] - 63.18711131651891 x[3,5]
   - 59.02459326558134 x[3,6] - 95.05283939070904 x[3,7]
   - 57.33693511834693 x[3,8] - 84.75755102827146 x[3,9]
   - 63.18711131651891 x[3,10] - 59.02459326558134 x[3,11]
   - 95.05283939070904 x[3,12] - 57.33693511834693 x[3,13]
   - 57.33693511834693 x[3,14] - 57.33693511834693 x[3,15]
   - start_service[3] + start_service[15] >= 0
 LowerBoundRoutingVehicle[4]: - 38.11059879063841 x[4,1]
   - 38.11059879063841 x[4,2] - 103.98388735598 x[4,3]
   - 38.11059879063841 x[4,4] - 53.98172644469906 x[4,5]
   - 47.8768068342243 x[4,6] - 56.75004824555334 x[4,7]
   - 103.98388735598 x[4,8] - 38.11059879063841 x[4,9]
   - 53.98172644469906 x[4,10] - 47.8768068342243 x[4,11]
   - 56.75004824555334 x[4,12] - 38.11059879063841 x[4,13]
   - 38.11059879063841 x[4,14] - 38.11059879063841 x[4,15]
   - start_service[4] + start_service[15] >= 0
 LowerBoundRoutingVehicle[5]: - 34.13174665045831 x[5,1]
   - 34.13174665045831 x[5,2] - 86.39229978440753 x[5,3]
   - 57.96057858487916 x[5,4] - 34.13174665045831 x[5,5]
   - 34.56896120543755 x[5,6] - 66.39242739679824 x[5,7]
   - 86.39229978440753 x[5,8] - 57.96057858487916 x[5,9]
   - 34.13174665045831 x[5,10] - 34.56896120543755 x[5,11]
   - 66.39242739679824 x[5,12] - 34.13174665045831 x[5,13]
   - 34.13174665045831 x[5,14] - 34.13174665045831 x[5,15]
   - start_service[5] + start_service[15] >= 0
 LowerBoundRoutingVehicle[6]: - 23.87912203495715 x[6,1]
   - 23.87912203495715 x[6,2] - 92.48240634897112 x[6,3]
   - 62.10828358990555 x[6,4] - 44.82158582093871 x[6,5]
   - 23.87912203495715 x[6,6] - 60.68586291448496 x[6,7]
   - 92.48240634897112 x[6,8] - 62.10828358990555 x[6,9]
   - 44.82158582093871 x[6,10] - 23.87912203495715 x[6,11]
   - 60.68586291448496 x[6,12] - 23.87912203495715 x[6,13]
   - 23.87912203495715 x[6,14] - 23.87912203495715 x[6,15]
   - start_service[6] + start_service[15] >= 0
 LowerBoundRoutingVehicle[7]: - 25.15924652494773 x[7,1]
   - 25.15924652494773 x[7,2] - 127.2305279841082 x[7,3]
   - 69.70140051124403 x[7,4] - 75.36492752230883 x[7,5]
   - 59.40573842449439 x[7,6] - 25.15924652494773 x[7,7]
   - 127.2305279841082 x[7,8] - 69.70140051124403 x[7,9]
   - 75.36492752230883 x[7,10] - 59.40573842449439 x[7,11]
   - 25.15924652494773 x[7,12] - 25.15924652494773 x[7,13]
   - 25.15924652494773 x[7,14] - 25.15924652494773 x[7,15]
   - start_service[7] + start_service[15] >= 0
 LowerBoundRoutingVehicle[8]: - 57.33693511834693 x[8,1]
   - 57.33693511834693 x[8,2] - 57.33693511834693 x[8,3]
   - 84.75755102827146 x[8,4] - 63.18711131651891 x[8,5]
   - 59.02459326558134 x[8,6] - 95.05283939070904 x[8,7]
   - 57.33693511834693 x[8,8] - 84.75755102827146 x[8,9]
   - 63.18711131651891 x[8,10] - 59.02459326558134 x[8,11]
   - 95.05283939070904 x[8,12] - 57.33693511834693 x[8,13]
   - 57.33693511834693 x[8,14] - 57.33693511834693 x[8,15]
   - start_service[8] + start_service[15] >= 0
 LowerBoundRoutingVehicle[9]: - 38.11059879063841 x[9,1]
   - 38.11059879063841 x[9,2] - 103.98388735598 x[9,3]
   - 38.11059879063841 x[9,4] - 53.98172644469906 x[9,5]
   - 47.8768068342243 x[9,6] - 56.75004824555334 x[9,7]
   - 103.98388735598 x[9,8] - 38.11059879063841 x[9,9]
   - 53.98172644469906 x[9,10] - 47.8768068342243 x[9,11]
   - 56.75004824555334 x[9,12] - 38.11059879063841 x[9,13]
   - 38.11059879063841 x[9,14] - 38.11059879063841 x[9,15]
   - start_service[9] + start_service[15] >= 0
 LowerBoundRoutingVehicle[10]: - 34.13174665045831 x[10,1]
   - 34.13174665045831 x[10,2] - 86.39229978440753 x[10,3]
   - 57.96057858487916 x[10,4] - 34.13174665045831 x[10,5]
   - 34.56896120543755 x[10,6] - 66.39242739679824 x[10,7]
   - 86.39229978440753 x[10,8] - 57.96057858487916 x[10,9]
   - 34.13174665045831 x[10,10] - 34.56896120543755 x[10,11]
   - 66.39242739679824 x[10,12] - 34.13174665045831 x[10,13]
   - 34.13174665045831 x[10,14] - 34.13174665045831 x[10,15]
   - start_service[10] + start_service[15] >= 0
 LowerBoundRoutingVehicle[11]: - 23.87912203495715 x[11,1]
   - 23.87912203495715 x[11,2] - 92.48240634897112 x[11,3]
   - 62.10828358990555 x[11,4] - 44.82158582093871 x[11,5]
   - 23.87912203495715 x[11,6] - 60.68586291448496 x[11,7]
   - 92.48240634897112 x[11,8] - 62.10828358990555 x[11,9]
   - 44.82158582093871 x[11,10] - 23.87912203495715 x[11,11]
   - 60.68586291448496 x[11,12] - 23.87912203495715 x[11,13]
   - 23.87912203495715 x[11,14] - 23.87912203495715 x[11,15]
   - start_service[11] + start_service[15] >= 0
 LowerBoundRoutingVehicle[12]: - 25.15924652494773 x[12,1]
   - 25.15924652494773 x[12,2] - 127.2305279841082 x[12,3]
   - 69.70140051124403 x[12,4] - 75.36492752230883 x[12,5]
   - 59.40573842449439 x[12,6] - 25.15924652494773 x[12,7]
   - 127.2305279841082 x[12,8] - 69.70140051124403 x[12,9]
   - 75.36492752230883 x[12,10] - 59.40573842449439 x[12,11]
   - 25.15924652494773 x[12,12] - 25.15924652494773 x[12,13]
   - 25.15924652494773 x[12,14] - 25.15924652494773 x[12,15]
   - start_service[12] + start_service[15] >= 0
 LowerBoundRoutingVehicle[13]: - 114.6738702366939 x[13,3]
   - 76.22119758127681 x[13,4] - 68.26349330091662 x[13,5]
   - 47.7582440699143 x[13,6] - 50.31849304989545 x[13,7]
   - 114.6738702366939 x[13,8] - 76.22119758127681 x[13,9]
   - 68.26349330091662 x[13,10] - 47.7582440699143 x[13,11]
   - 50.31849304989545 x[13,12] - start_service[13] + start_service[15]
   >= 0
 LowerBoundRoutingVehicle[14]: - 114.6738702366939 x[14,3]
   - 76.22119758127681 x[14,4] - 68.26349330091662 x[14,5]
   - 47.7582440699143 x[14,6] - 50.31849304989545 x[14,7]
   - 114.6738702366939 x[14,8] - 76.22119758127681 x[14,9]
   - 68.26349330091662 x[14,10] - 47.7582440699143 x[14,11]
   - 50.31849304989545 x[14,12] - start_service[14] + start_service[15]
   >= 0
 LowerBoundArrivalVehicle[1]: - 114.6738702366939 x[3,1]
   - 76.22119758127681 x[4,1] - 68.26349330091662 x[5,1]
   - 47.7582440699143 x[6,1] - 50.31849304989545 x[7,1]
   - 114.6738702366939 x[8,1] - 76.22119758127681 x[9,1]
   - 68.26349330091662 x[10,1] - 47.7582440699143 x[11,1]
   - 50.31849304989545 x[12,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 114.6738702366939 x[3,2]
   - 76.22119758127681 x[4,2] - 68.26349330091662 x[5,2]
   - 47.7582440699143 x[6,2] - 50.31849304989545 x[7,2]
   - 114.6738702366939 x[8,2] - 76.22119758127681 x[9,2]
   - 68.26349330091662 x[10,2] - 47.7582440699143 x[11,2]
   - 50.31849304989545 x[12,2] - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 57.33693511834693 x[0,3]
   - 57.33693511834693 x[1,3] - 57.33693511834693 x[2,3]
   - 57.33693511834693 x[3,3] - 84.75755102827146 x[4,3]
   - 63.18711131651891 x[5,3] - 59.02459326558134 x[6,3]
   - 95.05283939070904 x[7,3] - 57.33693511834693 x[8,3]
   - 84.75755102827146 x[9,3] - 63.18711131651891 x[10,3]
   - 59.02459326558134 x[11,3] - 95.05283939070904 x[12,3]
   - 57.33693511834693 x[13,3] - 57.33693511834693 x[14,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 38.11059879063841 x[0,4]
   - 38.11059879063841 x[1,4] - 38.11059879063841 x[2,4]
   - 103.98388735598 x[3,4] - 38.11059879063841 x[4,4]
   - 53.98172644469906 x[5,4] - 47.8768068342243 x[6,4]
   - 56.75004824555334 x[7,4] - 103.98388735598 x[8,4]
   - 38.11059879063841 x[9,4] - 53.98172644469906 x[10,4]
   - 47.8768068342243 x[11,4] - 56.75004824555334 x[12,4]
   - 38.11059879063841 x[13,4] - 38.11059879063841 x[14,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 34.13174665045831 x[0,5]
   - 34.13174665045831 x[1,5] - 34.13174665045831 x[2,5]
   - 86.39229978440753 x[3,5] - 57.96057858487916 x[4,5]
   - 34.13174665045831 x[5,5] - 34.56896120543755 x[6,5]
   - 66.39242739679824 x[7,5] - 86.39229978440753 x[8,5]
   - 57.96057858487916 x[9,5] - 34.13174665045831 x[10,5]
   - 34.56896120543755 x[11,5] - 66.39242739679824 x[12,5]
   - 34.13174665045831 x[13,5] - 34.13174665045831 x[14,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 23.87912203495715 x[0,6]
   - 23.87912203495715 x[1,6] - 23.87912203495715 x[2,6]
   - 92.48240634897112 x[3,6] - 62.10828358990555 x[4,6]
   - 44.82158582093871 x[5,6] - 23.87912203495715 x[6,6]
   - 60.68586291448496 x[7,6] - 92.48240634897112 x[8,6]
   - 62.10828358990555 x[9,6] - 44.82158582093871 x[10,6]
   - 23.87912203495715 x[11,6] - 60.68586291448496 x[12,6]
   - 23.87912203495715 x[13,6] - 23.87912203495715 x[14,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 25.15924652494773 x[0,7]
   - 25.15924652494773 x[1,7] - 25.15924652494773 x[2,7]
   - 127.2305279841082 x[3,7] - 69.70140051124403 x[4,7]
   - 75.36492752230883 x[5,7] - 59.40573842449439 x[6,7]
   - 25.15924652494773 x[7,7] - 127.2305279841082 x[8,7]
   - 69.70140051124403 x[9,7] - 75.36492752230883 x[10,7]
   - 59.40573842449439 x[11,7] - 25.15924652494773 x[12,7]
   - 25.15924652494773 x[13,7] - 25.15924652494773 x[14,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 57.33693511834693 x[0,8]
   - 57.33693511834693 x[1,8] - 57.33693511834693 x[2,8]
   - 57.33693511834693 x[3,8] - 84.75755102827146 x[4,8]
   - 63.18711131651891 x[5,8] - 59.02459326558134 x[6,8]
   - 95.05283939070904 x[7,8] - 57.33693511834693 x[8,8]
   - 84.75755102827146 x[9,8] - 63.18711131651891 x[10,8]
   - 59.02459326558134 x[11,8] - 95.05283939070904 x[12,8]
   - 57.33693511834693 x[13,8] - 57.33693511834693 x[14,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 38.11059879063841 x[0,9]
   - 38.11059879063841 x[1,9] - 38.11059879063841 x[2,9]
   - 103.98388735598 x[3,9] - 38.11059879063841 x[4,9]
   - 53.98172644469906 x[5,9] - 47.8768068342243 x[6,9]
   - 56.75004824555334 x[7,9] - 103.98388735598 x[8,9]
   - 38.11059879063841 x[9,9] - 53.98172644469906 x[10,9]
   - 47.8768068342243 x[11,9] - 56.75004824555334 x[12,9]
   - 38.11059879063841 x[13,9] - 38.11059879063841 x[14,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 34.13174665045831 x[0,10]
   - 34.13174665045831 x[1,10] - 34.13174665045831 x[2,10]
   - 86.39229978440753 x[3,10] - 57.96057858487916 x[4,10]
   - 34.13174665045831 x[5,10] - 34.56896120543755 x[6,10]
   - 66.39242739679824 x[7,10] - 86.39229978440753 x[8,10]
   - 57.96057858487916 x[9,10] - 34.13174665045831 x[10,10]
   - 34.56896120543755 x[11,10] - 66.39242739679824 x[12,10]
   - 34.13174665045831 x[13,10] - 34.13174665045831 x[14,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 23.87912203495715 x[0,11]
   - 23.87912203495715 x[1,11] - 23.87912203495715 x[2,11]
   - 92.48240634897112 x[3,11] - 62.10828358990555 x[4,11]
   - 44.82158582093871 x[5,11] - 23.87912203495715 x[6,11]
   - 60.68586291448496 x[7,11] - 92.48240634897112 x[8,11]
   - 62.10828358990555 x[9,11] - 44.82158582093871 x[10,11]
   - 23.87912203495715 x[11,11] - 60.68586291448496 x[12,11]
   - 23.87912203495715 x[13,11] - 23.87912203495715 x[14,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 25.15924652494773 x[0,12]
   - 25.15924652494773 x[1,12] - 25.15924652494773 x[2,12]
   - 127.2305279841082 x[3,12] - 69.70140051124403 x[4,12]
   - 75.36492752230883 x[5,12] - 59.40573842449439 x[6,12]
   - 25.15924652494773 x[7,12] - 127.2305279841082 x[8,12]
   - 69.70140051124403 x[9,12] - 75.36492752230883 x[10,12]
   - 59.40573842449439 x[11,12] - 25.15924652494773 x[12,12]
   - 25.15924652494773 x[13,12] - 25.15924652494773 x[14,12]
   - start_service[0] + start_service[12] >= 0
 LowerBoundArrivalVehicle[13]: - 114.6738702366939 x[3,13]
   - 76.22119758127681 x[4,13] - 68.26349330091662 x[5,13]
   - 47.7582440699143 x[6,13] - 50.31849304989545 x[7,13]
   - 114.6738702366939 x[8,13] - 76.22119758127681 x[9,13]
   - 68.26349330091662 x[10,13] - 47.7582440699143 x[11,13]
   - 50.31849304989545 x[12,13] - start_service[0] + start_service[13]
   >= 0
 LowerBoundArrivalVehicle[14]: - 114.6738702366939 x[3,14]
   - 76.22119758127681 x[4,14] - 68.26349330091662 x[5,14]
   - 47.7582440699143 x[6,14] - 50.31849304989545 x[7,14]
   - 114.6738702366939 x[8,14] - 76.22119758127681 x[9,14]
   - 68.26349330091662 x[10,14] - 47.7582440699143 x[11,14]
   - 50.31849304989545 x[12,14] - start_service[0] + start_service[14]
   >= 0
 LowerBoundArrivalVehicle[15]: - 114.6738702366939 x[3,15]
   - 76.22119758127681 x[4,15] - 68.26349330091662 x[5,15]
   - 47.7582440699143 x[6,15] - 50.31849304989545 x[7,15]
   - 114.6738702366939 x[8,15] - 76.22119758127681 x[9,15]
   - 68.26349330091662 x[10,15] - 47.7582440699143 x[11,15]
   - 50.31849304989545 x[12,15] - start_service[0] + start_service[15]
   >= 0
 VisitArrivalNodesNotEmpty[3]: w[0,3] + w[1,3] + w[2,3] + w[3,3] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] + w[13,3] + w[14,3] >= 1
 VisitArrivalNodesNotEmpty[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] + w[4,4]
   + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4]
   + w[12,4] + w[13,4] + w[14,4] >= 1
 VisitArrivalNodesNotEmpty[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   + w[5,5] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5]
   + w[12,5] + w[13,5] + w[14,5] >= 1
 VisitArrivalNodesNotEmpty[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] + w[6,6] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6]
   + w[12,6] + w[13,6] + w[14,6] >= 1
 VisitArrivalNodesNotEmpty[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] + w[7,7] + w[8,7] + w[9,7] + w[10,7] + w[11,7]
   + w[12,7] + w[13,7] + w[14,7] >= 1
 LeaveArrivalNodesNotFullyLoaded[3]: w[3,0] + w[3,1] + w[3,2] + w[3,4]
   + w[3,5] + w[3,6] + w[3,7] + w[3,9] + w[3,10] + w[3,11] + w[3,12]
   + w[3,13] + w[3,14] + w[3,15] <= 1
 LeaveArrivalNodesNotFullyLoaded[4]: w[4,0] + w[4,1] + w[4,2] + w[4,3]
   + w[4,5] + w[4,6] + w[4,7] + w[4,8] + w[4,10] + w[4,11] + w[4,12]
   + w[4,13] + w[4,14] + w[4,15] <= 1
 LeaveArrivalNodesNotFullyLoaded[5]: w[5,0] + w[5,1] + w[5,2] + w[5,3]
   + w[5,4] + w[5,6] + w[5,7] + w[5,8] + w[5,9] + w[5,11] + w[5,12]
   + w[5,13] + w[5,14] + w[5,15] <= 1
 LeaveArrivalNodesNotFullyLoaded[6]: w[6,0] + w[6,1] + w[6,2] + w[6,3]
   + w[6,4] + w[6,5] + w[6,7] + w[6,8] + w[6,9] + w[6,10] + w[6,12]
   + w[6,13] + w[6,14] + w[6,15] <= 1
 LeaveArrivalNodesNotFullyLoaded[7]: w[7,0] + w[7,1] + w[7,2] + w[7,3]
   + w[7,4] + w[7,5] + w[7,6] + w[7,8] + w[7,9] + w[7,10] + w[7,11]
   + w[7,13] + w[7,14] + w[7,15] <= 1
 VisitDepartureNodesNotFullyLoaded[8]: w[0,8] + w[1,8] + w[2,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] + w[9,8] + w[10,8] + w[11,8] + w[12,8]
   + w[13,8] + w[14,8] + w[15,8] <= 1
 VisitDepartureNodesNotFullyLoaded[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] + w[10,9] + w[11,9] + w[12,9]
   + w[13,9] + w[14,9] + w[15,9] <= 1
 VisitDepartureNodesNotFullyLoaded[10]: w[0,10] + w[1,10] + w[2,10]
   + w[3,10] + w[4,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] + w[11,10]
   + w[12,10] + w[13,10] + w[14,10] + w[15,10] <= 1
 VisitDepartureNodesNotFullyLoaded[11]: w[0,11] + w[1,11] + w[2,11]
   + w[3,11] + w[4,11] + w[5,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   + w[12,11] + w[13,11] + w[14,11] + w[15,11] <= 1
 VisitDepartureNodesNotFullyLoaded[12]: w[0,12] + w[1,12] + w[2,12]
   + w[3,12] + w[4,12] + w[5,12] + w[6,12] + w[8,12] + w[9,12] + w[10,12]
   + w[11,12] + w[13,12] + w[14,12] + w[15,12] <= 1
 LeaveDepartureNodesNotEmpty[8]: w[8,1] + w[8,2] + w[8,3] + w[8,4] + w[8,5]
   + w[8,6] + w[8,7] + w[8,8] + w[8,9] + w[8,10] + w[8,11] + w[8,12]
   + w[8,13] + w[8,14] + w[8,15] >= 1
 LeaveDepartureNodesNotEmpty[9]: w[9,1] + w[9,2] + w[9,3] + w[9,4] + w[9,5]
   + w[9,6] + w[9,7] + w[9,8] + w[9,9] + w[9,10] + w[9,11] + w[9,12]
   + w[9,13] + w[9,14] + w[9,15] >= 1
 LeaveDepartureNodesNotEmpty[10]: w[10,1] + w[10,2] + w[10,3] + w[10,4]
   + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,10] + w[10,11]
   + w[10,12] + w[10,13] + w[10,14] + w[10,15] >= 1
 LeaveDepartureNodesNotEmpty[11]: w[11,1] + w[11,2] + w[11,3] + w[11,4]
   + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10] + w[11,11]
   + w[11,12] + w[11,13] + w[11,14] + w[11,15] >= 1
 LeaveDepartureNodesNotEmpty[12]: w[12,1] + w[12,2] + w[12,3] + w[12,4]
   + w[12,5] + w[12,6] + w[12,7] + w[12,8] + w[12,9] + w[12,10] + w[12,11]
   + w[12,12] + w[12,13] + w[12,14] + w[12,15] >= 1
 R1001: - start_service[0] + start_service[1] >= 0
 R1002: - start_service[0] + start_service[2] >= 0
 R1003: - start_service[1] + start_service[2] >= 0
 R1004: - start_service[13] + start_service[14] >= 0
 R1005: - start_service[13] + start_service[15] >= 0
 R1006: - start_service[14] + start_service[15] >= 0
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
 x[4,0] = 0
 x[4,4] = 0
 x[5,0] = 0
 x[5,5] = 0
 x[6,0] = 0
 x[6,6] = 0
 x[7,0] = 0
 x[7,7] = 0
 x[8,0] = 0
 x[8,3] = 0
 x[8,8] = 0
 x[9,0] = 0
 x[9,4] = 0
 x[9,9] = 0
 x[10,0] = 0
 x[10,5] = 0
 x[10,10] = 0
 x[11,0] = 0
 x[11,6] = 0
 x[11,11] = 0
 x[12,0] = 0
 x[12,7] = 0
 x[12,12] = 0
 x[13,0] = 0
 x[13,13] = 0
 x[13,14] = 0
 x[13,15] = 0
 x[14,0] = 0
 x[14,13] = 0
 x[14,14] = 0
 x[14,15] = 0
 x[15,0] = 0
 x[15,1] = 0
 x[15,2] = 0
 x[15,3] = 0
 x[15,4] = 0
 x[15,5] = 0
 x[15,6] = 0
 x[15,7] = 0
 x[15,8] = 0
 x[15,9] = 0
 x[15,10] = 0
 x[15,11] = 0
 x[15,12] = 0
 x[15,13] = 0
 x[15,14] = 0
 x[15,15] = 0
 w[0,0] = 0
 w[0,1] = 0
 w[0,2] = 0
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
 w[1,0] = 0
 w[1,1] = 0
 w[1,2] = 0
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
 w[2,0] = 0
 w[2,1] = 0
 w[2,2] = 0
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
 w[3,0] = 0
 w[3,1] = 0
 w[3,2] = 0
 w[3,3] = 0
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
 w[4,0] = 0
 w[4,1] = 0
 w[4,2] = 0
 w[4,3] <= 2
 w[4,4] = 0
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
 w[5,0] = 0
 w[5,1] = 0
 w[5,2] = 0
 w[5,3] <= 2
 w[5,4] <= 2
 w[5,5] = 0
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
 w[6,0] = 0
 w[6,1] = 0
 w[6,2] = 0
 w[6,3] <= 2
 w[6,4] <= 2
 w[6,5] <= 2
 w[6,6] = 0
 w[6,7] <= 2
 w[6,8] <= 2
 w[6,9] <= 2
 w[6,10] <= 2
 w[6,11] <= 2
 w[6,12] <= 2
 w[6,13] <= 2
 w[6,14] <= 2
 w[6,15] <= 2
 w[7,0] = 0
 w[7,1] = 0
 w[7,2] = 0
 w[7,3] <= 2
 w[7,4] <= 2
 w[7,5] <= 2
 w[7,6] <= 2
 w[7,7] = 0
 w[7,8] <= 2
 w[7,9] <= 2
 w[7,10] <= 2
 w[7,11] <= 2
 w[7,12] <= 2
 w[7,13] <= 2
 w[7,14] <= 2
 w[7,15] <= 2
 w[8,0] = 0
 w[8,1] = 0
 w[8,2] = 0
 w[8,3] = 0
 w[8,4] <= 2
 w[8,5] <= 2
 w[8,6] <= 2
 w[8,7] <= 2
 w[8,8] = 0
 w[8,9] <= 2
 w[8,10] <= 2
 w[8,11] <= 2
 w[8,12] <= 2
 w[8,13] <= 2
 w[8,14] <= 2
 w[8,15] <= 2
 w[9,0] = 0
 w[9,1] = 0
 w[9,2] = 0
 w[9,3] <= 2
 w[9,4] = 0
 w[9,5] <= 2
 w[9,6] <= 2
 w[9,7] <= 2
 w[9,8] <= 2
 w[9,9] = 0
 w[9,10] <= 2
 w[9,11] <= 2
 w[9,12] <= 2
 w[9,13] <= 2
 w[9,14] <= 2
 w[9,15] <= 2
 w[10,0] = 0
 w[10,1] = 0
 w[10,2] = 0
 w[10,3] <= 2
 w[10,4] <= 2
 w[10,5] = 0
 w[10,6] <= 2
 w[10,7] <= 2
 w[10,8] <= 2
 w[10,9] <= 2
 w[10,10] = 0
 w[10,11] <= 2
 w[10,12] <= 2
 w[10,13] <= 2
 w[10,14] <= 2
 w[10,15] <= 2
 w[11,0] = 0
 w[11,1] = 0
 w[11,2] = 0
 w[11,3] <= 2
 w[11,4] <= 2
 w[11,5] <= 2
 w[11,6] = 0
 w[11,7] <= 2
 w[11,8] <= 2
 w[11,9] <= 2
 w[11,10] <= 2
 w[11,11] = 0
 w[11,12] <= 2
 w[11,13] <= 2
 w[11,14] <= 2
 w[11,15] <= 2
 w[12,0] = 0
 w[12,1] = 0
 w[12,2] = 0
 w[12,3] <= 2
 w[12,4] <= 2
 w[12,5] <= 2
 w[12,6] <= 2
 w[12,7] = 0
 w[12,8] <= 2
 w[12,9] <= 2
 w[12,10] <= 2
 w[12,11] <= 2
 w[12,12] = 0
 w[12,13] <= 2
 w[12,14] <= 2
 w[12,15] <= 2
 w[13,0] = 0
 w[13,1] = 0
 w[13,2] = 0
 w[13,3] = 0
 w[13,4] = 0
 w[13,5] = 0
 w[13,6] = 0
 w[13,7] = 0
 w[13,8] = 0
 w[13,9] = 0
 w[13,10] = 0
 w[13,11] = 0
 w[13,12] = 0
 w[13,13] = 0
 w[13,14] = 0
 w[13,15] = 0
 w[14,0] = 0
 w[14,1] = 0
 w[14,2] = 0
 w[14,3] = 0
 w[14,4] = 0
 w[14,5] = 0
 w[14,6] = 0
 w[14,7] = 0
 w[14,8] = 0
 w[14,9] = 0
 w[14,10] = 0
 w[14,11] = 0
 w[14,12] = 0
 w[14,13] = 0
 w[14,14] = 0
 w[14,15] = 0
 w[15,0] = 0
 w[15,1] = 0
 w[15,2] = 0
 w[15,3] = 0
 w[15,4] = 0
 w[15,5] = 0
 w[15,6] = 0
 w[15,7] = 0
 w[15,8] = 0
 w[15,9] = 0
 w[15,10] = 0
 w[15,11] = 0
 w[15,12] = 0
 w[15,13] = 0
 w[15,14] = 0
 w[15,15] = 0
 v[0,0] = 0
 v[0,1] = 0
 v[0,2] = 0
 v[1,0] = 0
 v[1,1] = 0
 v[1,2] = 0
 v[2,0] = 0
 v[2,1] = 0
 v[2,2] = 0
 v[3,0] = 0
 v[3,1] = 0
 v[3,2] = 0
 v[3,3] = 0
 v[3,8] = 1
 v[4,0] = 0
 v[4,1] = 0
 v[4,2] = 0
 v[4,4] = 0
 v[4,9] = 1
 v[5,0] = 0
 v[5,1] = 0
 v[5,2] = 0
 v[5,5] = 0
 v[5,10] = 1
 v[6,0] = 0
 v[6,1] = 0
 v[6,2] = 0
 v[6,6] = 0
 v[6,11] = 1
 v[7,0] = 0
 v[7,1] = 0
 v[7,2] = 0
 v[7,7] = 0
 v[7,12] = 1
 v[8,0] = 0
 v[8,1] = 0
 v[8,2] = 0
 v[8,3] = 0
 v[8,8] = 0
 v[9,0] = 0
 v[9,1] = 0
 v[9,2] = 0
 v[9,4] = 0
 v[9,9] = 0
 v[10,0] = 0
 v[10,1] = 0
 v[10,2] = 0
 v[10,5] = 0
 v[10,10] = 0
 v[11,0] = 0
 v[11,1] = 0
 v[11,2] = 0
 v[11,6] = 0
 v[11,11] = 0
 v[12,0] = 0
 v[12,1] = 0
 v[12,2] = 0
 v[12,7] = 0
 v[12,12] = 0
 v[13,0] = 0
 v[13,1] = 0
 v[13,2] = 0
 v[13,3] = 0
 v[13,4] = 0
 v[13,5] = 0
 v[13,6] = 0
 v[13,7] = 0
 v[13,8] = 0
 v[13,9] = 0
 v[13,10] = 0
 v[13,11] = 0
 v[13,12] = 0
 v[13,13] = 0
 v[13,14] = 0
 v[13,15] = 0
 v[14,0] = 0
 v[14,1] = 0
 v[14,2] = 0
 v[14,3] = 0
 v[14,4] = 0
 v[14,5] = 0
 v[14,6] = 0
 v[14,7] = 0
 v[14,8] = 0
 v[14,9] = 0
 v[14,10] = 0
 v[14,11] = 0
 v[14,12] = 0
 v[14,13] = 0
 v[14,14] = 0
 v[14,15] = 0
 v[15,0] = 0
 v[15,1] = 0
 v[15,2] = 0
 v[15,3] = 0
 v[15,4] = 0
 v[15,5] = 0
 v[15,6] = 0
 v[15,7] = 0
 v[15,8] = 0
 v[15,9] = 0
 v[15,10] = 0
 v[15,11] = 0
 v[15,12] = 0
 v[15,13] = 0
 v[15,14] = 0
 v[15,15] = 0
 service_time[0] = 0
 service_time[1] = 0
 service_time[2] = 0
 service_time[8] = 0
 service_time[9] = 0
 service_time[10] = 0
 service_time[11] = 0
 service_time[12] = 0
 service_time[13] = 0
 service_time[14] = 0
 service_time[15] = 0
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
Binaries
 x[0,0] x[0,1] x[0,2] x[0,3] x[0,4] x[0,5] x[0,6] x[0,7] x[0,8] x[0,9]
 x[0,10] x[0,11] x[0,12] x[0,13] x[0,14] x[0,15] x[1,0] x[1,1] x[1,2]
 x[1,3] x[1,4] x[1,5] x[1,6] x[1,7] x[1,8] x[1,9] x[1,10] x[1,11] x[1,12]
 x[1,13] x[1,14] x[1,15] x[2,0] x[2,1] x[2,2] x[2,3] x[2,4] x[2,5] x[2,6]
 x[2,7] x[2,8] x[2,9] x[2,10] x[2,11] x[2,12] x[2,13] x[2,14] x[2,15]
 x[3,0] x[3,1] x[3,2] x[3,3] x[3,4] x[3,5] x[3,6] x[3,7] x[3,8] x[3,9]
 x[3,10] x[3,11] x[3,12] x[3,13] x[3,14] x[3,15] x[4,0] x[4,1] x[4,2]
 x[4,3] x[4,4] x[4,5] x[4,6] x[4,7] x[4,8] x[4,9] x[4,10] x[4,11] x[4,12]
 x[4,13] x[4,14] x[4,15] x[5,0] x[5,1] x[5,2] x[5,3] x[5,4] x[5,5] x[5,6]
 x[5,7] x[5,8] x[5,9] x[5,10] x[5,11] x[5,12] x[5,13] x[5,14] x[5,15]
 x[6,0] x[6,1] x[6,2] x[6,3] x[6,4] x[6,5] x[6,6] x[6,7] x[6,8] x[6,9]
 x[6,10] x[6,11] x[6,12] x[6,13] x[6,14] x[6,15] x[7,0] x[7,1] x[7,2]
 x[7,3] x[7,4] x[7,5] x[7,6] x[7,7] x[7,8] x[7,9] x[7,10] x[7,11] x[7,12]
 x[7,13] x[7,14] x[7,15] x[8,0] x[8,1] x[8,2] x[8,3] x[8,4] x[8,5] x[8,6]
 x[8,7] x[8,8] x[8,9] x[8,10] x[8,11] x[8,12] x[8,13] x[8,14] x[8,15]
 x[9,0] x[9,1] x[9,2] x[9,3] x[9,4] x[9,5] x[9,6] x[9,7] x[9,8] x[9,9]
 x[9,10] x[9,11] x[9,12] x[9,13] x[9,14] x[9,15] x[10,0] x[10,1] x[10,2]
 x[10,3] x[10,4] x[10,5] x[10,6] x[10,7] x[10,8] x[10,9] x[10,10] x[10,11]
 x[10,12] x[10,13] x[10,14] x[10,15] x[11,0] x[11,1] x[11,2] x[11,3]
 x[11,4] x[11,5] x[11,6] x[11,7] x[11,8] x[11,9] x[11,10] x[11,11] x[11,12]
 x[11,13] x[11,14] x[11,15] x[12,0] x[12,1] x[12,2] x[12,3] x[12,4] x[12,5]
 x[12,6] x[12,7] x[12,8] x[12,9] x[12,10] x[12,11] x[12,12] x[12,13]
 x[12,14] x[12,15] x[13,0] x[13,1] x[13,2] x[13,3] x[13,4] x[13,5] x[13,6]
 x[13,7] x[13,8] x[13,9] x[13,10] x[13,11] x[13,12] x[13,13] x[13,14]
 x[13,15] x[14,0] x[14,1] x[14,2] x[14,3] x[14,4] x[14,5] x[14,6] x[14,7]
 x[14,8] x[14,9] x[14,10] x[14,11] x[14,12] x[14,13] x[14,14] x[14,15]
 x[15,0] x[15,1] x[15,2] x[15,3] x[15,4] x[15,5] x[15,6] x[15,7] x[15,8]
 x[15,9] x[15,10] x[15,11] x[15,12] x[15,13] x[15,14] x[15,15] v[0,0]
 v[0,1] v[0,2] v[0,3] v[0,4] v[0,5] v[0,6] v[0,7] v[0,8] v[0,9] v[0,10]
 v[0,11] v[0,12] v[0,13] v[0,14] v[0,15] v[1,0] v[1,1] v[1,2] v[1,3] v[1,4]
 v[1,5] v[1,6] v[1,7] v[1,8] v[1,9] v[1,10] v[1,11] v[1,12] v[1,13] v[1,14]
 v[1,15] v[2,0] v[2,1] v[2,2] v[2,3] v[2,4] v[2,5] v[2,6] v[2,7] v[2,8]
 v[2,9] v[2,10] v[2,11] v[2,12] v[2,13] v[2,14] v[2,15] v[3,0] v[3,1]
 v[3,2] v[3,3] v[3,4] v[3,5] v[3,6] v[3,7] v[3,8] v[3,9] v[3,10] v[3,11]
 v[3,12] v[3,13] v[3,14] v[3,15] v[4,0] v[4,1] v[4,2] v[4,3] v[4,4] v[4,5]
 v[4,6] v[4,7] v[4,8] v[4,9] v[4,10] v[4,11] v[4,12] v[4,13] v[4,14]
 v[4,15] v[5,0] v[5,1] v[5,2] v[5,3] v[5,4] v[5,5] v[5,6] v[5,7] v[5,8]
 v[5,9] v[5,10] v[5,11] v[5,12] v[5,13] v[5,14] v[5,15] v[6,0] v[6,1]
 v[6,2] v[6,3] v[6,4] v[6,5] v[6,6] v[6,7] v[6,8] v[6,9] v[6,10] v[6,11]
 v[6,12] v[6,13] v[6,14] v[6,15] v[7,0] v[7,1] v[7,2] v[7,3] v[7,4] v[7,5]
 v[7,6] v[7,7] v[7,8] v[7,9] v[7,10] v[7,11] v[7,12] v[7,13] v[7,14]
 v[7,15] v[8,0] v[8,1] v[8,2] v[8,3] v[8,4] v[8,5] v[8,6] v[8,7] v[8,8]
 v[8,9] v[8,10] v[8,11] v[8,12] v[8,13] v[8,14] v[8,15] v[9,0] v[9,1]
 v[9,2] v[9,3] v[9,4] v[9,5] v[9,6] v[9,7] v[9,8] v[9,9] v[9,10] v[9,11]
 v[9,12] v[9,13] v[9,14] v[9,15] v[10,0] v[10,1] v[10,2] v[10,3] v[10,4]
 v[10,5] v[10,6] v[10,7] v[10,8] v[10,9] v[10,10] v[10,11] v[10,12]
 v[10,13] v[10,14] v[10,15] v[11,0] v[11,1] v[11,2] v[11,3] v[11,4] v[11,5]
 v[11,6] v[11,7] v[11,8] v[11,9] v[11,10] v[11,11] v[11,12] v[11,13]
 v[11,14] v[11,15] v[12,0] v[12,1] v[12,2] v[12,3] v[12,4] v[12,5] v[12,6]
 v[12,7] v[12,8] v[12,9] v[12,10] v[12,11] v[12,12] v[12,13] v[12,14]
 v[12,15] v[13,0] v[13,1] v[13,2] v[13,3] v[13,4] v[13,5] v[13,6] v[13,7]
 v[13,8] v[13,9] v[13,10] v[13,11] v[13,12] v[13,13] v[13,14] v[13,15]
 v[14,0] v[14,1] v[14,2] v[14,3] v[14,4] v[14,5] v[14,6] v[14,7] v[14,8]
 v[14,9] v[14,10] v[14,11] v[14,12] v[14,13] v[14,14] v[14,15] v[15,0]
 v[15,1] v[15,2] v[15,3] v[15,4] v[15,5] v[15,6] v[15,7] v[15,8] v[15,9]
 v[15,10] v[15,11] v[15,12] v[15,13] v[15,14] v[15,15] y[3,1] y[3,2] y[4,1]
 y[4,2] y[5,1] y[5,2] y[6,1] y[6,2] y[7,1] y[7,2]
Generals
 w[0,0] w[0,1] w[0,2] w[0,3] w[0,4] w[0,5] w[0,6] w[0,7] w[0,8] w[0,9]
 w[0,10] w[0,11] w[0,12] w[0,13] w[0,14] w[0,15] w[1,0] w[1,1] w[1,2]
 w[1,3] w[1,4] w[1,5] w[1,6] w[1,7] w[1,8] w[1,9] w[1,10] w[1,11] w[1,12]
 w[1,13] w[1,14] w[1,15] w[2,0] w[2,1] w[2,2] w[2,3] w[2,4] w[2,5] w[2,6]
 w[2,7] w[2,8] w[2,9] w[2,10] w[2,11] w[2,12] w[2,13] w[2,14] w[2,15]
 w[3,0] w[3,1] w[3,2] w[3,3] w[3,4] w[3,5] w[3,6] w[3,7] w[3,8] w[3,9]
 w[3,10] w[3,11] w[3,12] w[3,13] w[3,14] w[3,15] w[4,0] w[4,1] w[4,2]
 w[4,3] w[4,4] w[4,5] w[4,6] w[4,7] w[4,8] w[4,9] w[4,10] w[4,11] w[4,12]
 w[4,13] w[4,14] w[4,15] w[5,0] w[5,1] w[5,2] w[5,3] w[5,4] w[5,5] w[5,6]
 w[5,7] w[5,8] w[5,9] w[5,10] w[5,11] w[5,12] w[5,13] w[5,14] w[5,15]
 w[6,0] w[6,1] w[6,2] w[6,3] w[6,4] w[6,5] w[6,6] w[6,7] w[6,8] w[6,9]
 w[6,10] w[6,11] w[6,12] w[6,13] w[6,14] w[6,15] w[7,0] w[7,1] w[7,2]
 w[7,3] w[7,4] w[7,5] w[7,6] w[7,7] w[7,8] w[7,9] w[7,10] w[7,11] w[7,12]
 w[7,13] w[7,14] w[7,15] w[8,0] w[8,1] w[8,2] w[8,3] w[8,4] w[8,5] w[8,6]
 w[8,7] w[8,8] w[8,9] w[8,10] w[8,11] w[8,12] w[8,13] w[8,14] w[8,15]
 w[9,0] w[9,1] w[9,2] w[9,3] w[9,4] w[9,5] w[9,6] w[9,7] w[9,8] w[9,9]
 w[9,10] w[9,11] w[9,12] w[9,13] w[9,14] w[9,15] w[10,0] w[10,1] w[10,2]
 w[10,3] w[10,4] w[10,5] w[10,6] w[10,7] w[10,8] w[10,9] w[10,10] w[10,11]
 w[10,12] w[10,13] w[10,14] w[10,15] w[11,0] w[11,1] w[11,2] w[11,3]
 w[11,4] w[11,5] w[11,6] w[11,7] w[11,8] w[11,9] w[11,10] w[11,11] w[11,12]
 w[11,13] w[11,14] w[11,15] w[12,0] w[12,1] w[12,2] w[12,3] w[12,4] w[12,5]
 w[12,6] w[12,7] w[12,8] w[12,9] w[12,10] w[12,11] w[12,12] w[12,13]
 w[12,14] w[12,15] w[13,0] w[13,1] w[13,2] w[13,3] w[13,4] w[13,5] w[13,6]
 w[13,7] w[13,8] w[13,9] w[13,10] w[13,11] w[13,12] w[13,13] w[13,14]
 w[13,15] w[14,0] w[14,1] w[14,2] w[14,3] w[14,4] w[14,5] w[14,6] w[14,7]
 w[14,8] w[14,9] w[14,10] w[14,11] w[14,12] w[14,13] w[14,14] w[14,15]
 w[15,0] w[15,1] w[15,2] w[15,3] w[15,4] w[15,5] w[15,6] w[15,7] w[15,8]
 w[15,9] w[15,10] w[15,11] w[15,12] w[15,13] w[15,14] w[15,15]
End
