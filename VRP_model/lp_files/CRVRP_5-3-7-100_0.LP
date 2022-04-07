\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[13,0] + 0 x[13,1] + 0 x[13,2] + 0 x[13,3] + 0 x[13,4]
   + 0 x[13,5] + 0 x[13,6] + 0 x[13,7] + 0 x[13,8] + 0 x[13,9] + 0 x[13,10]
   + 0 x[13,11] + 0 x[13,12] + 0 x[13,13] + 0 w[13,0] + 0 w[13,1]
   + 0 w[13,2] + 0 w[13,3] + 0 w[13,4] + 0 w[13,5] + 0 w[13,6] + 0 w[13,12]
   + 0 w[13,13] + 0 v[0,0] + 0 v[1,0] + 0 v[2,0] + 0 v[3,0] + 0 v[4,0]
   + 0 v[5,0] + 0 v[6,0] + 0 v[7,0] + 0 v[8,0] + 0 v[9,0] + 0 v[10,0]
   + 0 v[11,0] + 0 v[12,0] + 0 v[13,0] + 0 v[13,1] + 0 v[13,2] + 0 v[13,3]
   + 0 v[13,4] + 0 v[13,5] + 0 v[13,6] + 0 v[13,7] + 0 v[13,8] + 0 v[13,9]
   + 0 v[13,10] + 0 v[13,11] + 0 v[13,12] + 0 v[13,13] + 0 service_time[0]
   + 0 service_time[1] + 0 service_time[7] + 0 service_time[8]
   + 0 service_time[9] + 0 service_time[10] + 0 service_time[11]
   + 0 service_time[12] + 0 service_time[13] + start_service[13]
Subject To
 MaxNumberVehiclesStart: x[0,1] + x[0,2] + x[0,3] + x[0,4] + x[0,5]
   + x[0,6] + x[0,7] + x[0,8] + x[0,9] + x[0,10] + x[0,11] + x[0,12]
   + x[0,13] <= 3
 OutFlowVehicles[1]: x[1,1] + x[1,2] + x[1,3] + x[1,4] + x[1,5] + x[1,6]
   + x[1,7] + x[1,8] + x[1,9] + x[1,10] + x[1,11] + x[1,12] + x[1,13] = 1
 OutFlowVehicles[2]: x[2,1] + x[2,2] + x[2,3] + x[2,4] + x[2,5] + x[2,6]
   + x[2,7] + x[2,8] + x[2,9] + x[2,10] + x[2,11] + x[2,12] + x[2,13] = 1
 OutFlowVehicles[3]: x[3,1] + x[3,2] + x[3,3] + x[3,4] + x[3,5] + x[3,6]
   + x[3,7] + x[3,8] + x[3,9] + x[3,10] + x[3,11] + x[3,12] + x[3,13] = 1
 OutFlowVehicles[4]: x[4,1] + x[4,2] + x[4,3] + x[4,4] + x[4,5] + x[4,6]
   + x[4,7] + x[4,8] + x[4,9] + x[4,10] + x[4,11] + x[4,12] + x[4,13] = 1
 OutFlowVehicles[5]: x[5,1] + x[5,2] + x[5,3] + x[5,4] + x[5,5] + x[5,6]
   + x[5,7] + x[5,8] + x[5,9] + x[5,10] + x[5,11] + x[5,12] + x[5,13] = 1
 OutFlowVehicles[6]: x[6,1] + x[6,2] + x[6,3] + x[6,4] + x[6,5] + x[6,6]
   + x[6,7] + x[6,8] + x[6,9] + x[6,10] + x[6,11] + x[6,12] + x[6,13] = 1
 OutFlowVehicles[7]: x[7,1] + x[7,2] + x[7,3] + x[7,4] + x[7,5] + x[7,6]
   + x[7,7] + x[7,8] + x[7,9] + x[7,10] + x[7,11] + x[7,12] + x[7,13] = 1
 OutFlowVehicles[8]: x[8,1] + x[8,2] + x[8,3] + x[8,4] + x[8,5] + x[8,6]
   + x[8,7] + x[8,8] + x[8,9] + x[8,10] + x[8,11] + x[8,12] + x[8,13] = 1
 OutFlowVehicles[9]: x[9,1] + x[9,2] + x[9,3] + x[9,4] + x[9,5] + x[9,6]
   + x[9,7] + x[9,8] + x[9,9] + x[9,10] + x[9,11] + x[9,12] + x[9,13] = 1
 OutFlowVehicles[10]: x[10,1] + x[10,2] + x[10,3] + x[10,4] + x[10,5]
   + x[10,6] + x[10,7] + x[10,8] + x[10,9] + x[10,10] + x[10,11] + x[10,12]
   + x[10,13] = 1
 OutFlowVehicles[11]: x[11,1] + x[11,2] + x[11,3] + x[11,4] + x[11,5]
   + x[11,6] + x[11,7] + x[11,8] + x[11,9] + x[11,10] + x[11,11] + x[11,12]
   + x[11,13] = 1
 OutFlowVehicles[12]: x[12,1] + x[12,2] + x[12,3] + x[12,4] + x[12,5]
   + x[12,6] + x[12,7] + x[12,8] + x[12,9] + x[12,10] + x[12,11] + x[12,12]
   + x[12,13] = 1
 InFlowVehicles[1]: x[0,1] + x[1,1] + x[2,1] + x[3,1] + x[4,1] + x[5,1]
   + x[6,1] + x[7,1] + x[8,1] + x[9,1] + x[10,1] + x[11,1] + x[12,1] = 1
 InFlowVehicles[2]: x[0,2] + x[1,2] + x[2,2] + x[3,2] + x[4,2] + x[5,2]
   + x[6,2] + x[7,2] + x[8,2] + x[9,2] + x[10,2] + x[11,2] + x[12,2] = 1
 InFlowVehicles[3]: x[0,3] + x[1,3] + x[2,3] + x[3,3] + x[4,3] + x[5,3]
   + x[6,3] + x[7,3] + x[8,3] + x[9,3] + x[10,3] + x[11,3] + x[12,3] = 1
 InFlowVehicles[4]: x[0,4] + x[1,4] + x[2,4] + x[3,4] + x[4,4] + x[5,4]
   + x[6,4] + x[7,4] + x[8,4] + x[9,4] + x[10,4] + x[11,4] + x[12,4] = 1
 InFlowVehicles[5]: x[0,5] + x[1,5] + x[2,5] + x[3,5] + x[4,5] + x[5,5]
   + x[6,5] + x[7,5] + x[8,5] + x[9,5] + x[10,5] + x[11,5] + x[12,5] = 1
 InFlowVehicles[6]: x[0,6] + x[1,6] + x[2,6] + x[3,6] + x[4,6] + x[5,6]
   + x[6,6] + x[7,6] + x[8,6] + x[9,6] + x[10,6] + x[11,6] + x[12,6] = 1
 InFlowVehicles[7]: x[0,7] + x[1,7] + x[2,7] + x[3,7] + x[4,7] + x[5,7]
   + x[6,7] + x[7,7] + x[8,7] + x[9,7] + x[10,7] + x[11,7] + x[12,7] = 1
 InFlowVehicles[8]: x[0,8] + x[1,8] + x[2,8] + x[3,8] + x[4,8] + x[5,8]
   + x[6,8] + x[7,8] + x[8,8] + x[9,8] + x[10,8] + x[11,8] + x[12,8] = 1
 InFlowVehicles[9]: x[0,9] + x[1,9] + x[2,9] + x[3,9] + x[4,9] + x[5,9]
   + x[6,9] + x[7,9] + x[8,9] + x[9,9] + x[10,9] + x[11,9] + x[12,9] = 1
 InFlowVehicles[10]: x[0,10] + x[1,10] + x[2,10] + x[3,10] + x[4,10]
   + x[5,10] + x[6,10] + x[7,10] + x[8,10] + x[9,10] + x[10,10] + x[11,10]
   + x[12,10] = 1
 InFlowVehicles[11]: x[0,11] + x[1,11] + x[2,11] + x[3,11] + x[4,11]
   + x[5,11] + x[6,11] + x[7,11] + x[8,11] + x[9,11] + x[10,11] + x[11,11]
   + x[12,11] = 1
 InFlowVehicles[12]: x[0,12] + x[1,12] + x[2,12] + x[3,12] + x[4,12]
   + x[5,12] + x[6,12] + x[7,12] + x[8,12] + x[9,12] + x[10,12] + x[11,12]
   + x[12,12] = 1
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - start_service[0]
   + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 1.0000296123560457e+06 x[0,2]
   - start_service[0] + start_service[2] >= -999999
 NodeArrivalVehicles[0,3]: - 1.0000455895751599e+06 x[0,3]
   - start_service[0] + start_service[3] >= -999999
 NodeArrivalVehicles[0,4]: - 1.0000661657955264e+06 x[0,4]
   - start_service[0] + start_service[4] >= -999999
 NodeArrivalVehicles[0,5]: - 1.0000467938523132e+06 x[0,5]
   - start_service[0] + start_service[5] >= -999999
 NodeArrivalVehicles[0,6]: - 1.000058535470724e+06 x[0,6]
   - start_service[0] + start_service[6] >= -999999
 NodeArrivalVehicles[0,7]: - 1.0000296123560457e+06 x[0,7]
   - start_service[0] + start_service[7] >= -999999
 NodeArrivalVehicles[0,8]: - 1.0000455895751599e+06 x[0,8]
   - start_service[0] + start_service[8] >= -999999
 NodeArrivalVehicles[0,9]: - 1.0000661657955264e+06 x[0,9]
   - start_service[0] + start_service[9] >= -999999
 NodeArrivalVehicles[0,10]: - 1.0000467938523132e+06 x[0,10]
   - start_service[0] + start_service[10] >= -999999
 NodeArrivalVehicles[0,11]: - 1.000058535470724e+06 x[0,11]
   - start_service[0] + start_service[11] >= -999999
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 1.0000296123560457e+06 x[1,2]
   - start_service[1] + start_service[2] >= -999999
 NodeArrivalVehicles[1,3]: - 1.0000455895751599e+06 x[1,3]
   - start_service[1] + start_service[3] >= -999999
 NodeArrivalVehicles[1,4]: - 1.0000661657955264e+06 x[1,4]
   - start_service[1] + start_service[4] >= -999999
 NodeArrivalVehicles[1,5]: - 1.0000467938523132e+06 x[1,5]
   - start_service[1] + start_service[5] >= -999999
 NodeArrivalVehicles[1,6]: - 1.000058535470724e+06 x[1,6]
   - start_service[1] + start_service[6] >= -999999
 NodeArrivalVehicles[1,7]: - 1.0000296123560457e+06 x[1,7]
   - start_service[1] + start_service[7] >= -999999
 NodeArrivalVehicles[1,8]: - 1.0000455895751599e+06 x[1,8]
   - start_service[1] + start_service[8] >= -999999
 NodeArrivalVehicles[1,9]: - 1.0000661657955264e+06 x[1,9]
   - start_service[1] + start_service[9] >= -999999
 NodeArrivalVehicles[1,10]: - 1.0000467938523132e+06 x[1,10]
   - start_service[1] + start_service[10] >= -999999
 NodeArrivalVehicles[1,11]: - 1.000058535470724e+06 x[1,11]
   - start_service[1] + start_service[11] >= -999999
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999999
 NodeArrivalVehicles[2,1]: - 1.0000296123560457e+06 x[2,1]
   + start_service[1] - start_service[2] >= -999999
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 1.000076022979941e+06 x[2,3]
   - start_service[2] + start_service[3] >= -999999
 NodeArrivalVehicles[2,4]: - 1.0000468021655091e+06 x[2,4]
   - start_service[2] + start_service[4] >= -999999
 NodeArrivalVehicles[2,5]: - 1.0000422223602368e+06 x[2,5]
   - start_service[2] + start_service[5] >= -999999
 NodeArrivalVehicles[2,6]: - 1.0000766646394427e+06 x[2,6]
   - start_service[2] + start_service[6] >= -999999
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 1.000076022979941e+06 x[2,8]
   - start_service[2] + start_service[8] >= -999999
 NodeArrivalVehicles[2,9]: - 1.0000468021655091e+06 x[2,9]
   - start_service[2] + start_service[9] >= -999999
 NodeArrivalVehicles[2,10]: - 1.0000422223602368e+06 x[2,10]
   - start_service[2] + start_service[10] >= -999999
 NodeArrivalVehicles[2,11]: - 1.0000766646394427e+06 x[2,11]
   - start_service[2] + start_service[11] >= -999999
 NodeArrivalVehicles[2,12]: - 1.0000296123560457e+06 x[2,12]
   - start_service[2] + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 1.0000296123560457e+06 x[2,13]
   - start_service[2] + start_service[13] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000455895751599e+06 x[3,1]
   + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.000076022979941e+06 x[3,2]
   + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0001033774021529e+06 x[3,4]
   - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.0000761130502367e+06 x[3,5]
   - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.0000507866606601e+06 x[3,6]
   - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.000076022979941e+06 x[3,7]
   - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0001033774021529e+06 x[3,9]
   - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.0000761130502367e+06 x[3,10]
   - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.0000507866606601e+06 x[3,11]
   - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 1.0000455895751599e+06 x[3,12]
   - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 1.0000455895751599e+06 x[3,13]
   - start_service[3] + start_service[13] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000661657955264e+06 x[4,1]
   + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.0000468021655091e+06 x[4,2]
   + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0001033774021529e+06 x[4,3]
   + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.0000285700562182e+06 x[4,5]
   - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000767645100753e+06 x[4,6]
   - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000468021655091e+06 x[4,7]
   - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0001033774021529e+06 x[4,8]
   - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.0000285700562182e+06 x[4,10]
   - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000767645100753e+06 x[4,11]
   - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 1.0000661657955264e+06 x[4,12]
   - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 1.0000661657955264e+06 x[4,13]
   - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000467938523132e+06 x[5,1]
   + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000422223602368e+06 x[5,2]
   + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.0000761130502367e+06 x[5,3]
   + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.0000285700562182e+06 x[5,4]
   + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000477981369909e+06 x[5,6]
   - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000422223602368e+06 x[5,7]
   - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.0000761130502367e+06 x[5,8]
   - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.0000285700562182e+06 x[5,9]
   - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000477981369909e+06 x[5,11]
   - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 1.0000467938523132e+06 x[5,12]
   - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 1.0000467938523132e+06 x[5,13]
   - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[6,1]: - 1.000058535470724e+06 x[6,1]
   + start_service[1] - start_service[6] >= -999999
 NodeArrivalVehicles[6,2]: - 1.0000766646394427e+06 x[6,2]
   + start_service[2] - start_service[6] >= -999999
 NodeArrivalVehicles[6,3]: - 1.0000507866606601e+06 x[6,3]
   + start_service[3] - start_service[6] >= -999999
 NodeArrivalVehicles[6,4]: - 1.0000767645100753e+06 x[6,4]
   + start_service[4] - start_service[6] >= -999999
 NodeArrivalVehicles[6,5]: - 1.0000477981369909e+06 x[6,5]
   + start_service[5] - start_service[6] >= -999999
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000766646394427e+06 x[6,7]
   - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.0000507866606601e+06 x[6,8]
   - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000767645100753e+06 x[6,9]
   - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000477981369909e+06 x[6,10]
   - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 1.000058535470724e+06 x[6,12]
   - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 1.000058535470724e+06 x[6,13]
   - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[7,1]: - 1.0000296123560457e+06 x[7,1]
   + start_service[1] - start_service[7] >= -999999
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 1.000076022979941e+06 x[7,3]
   + start_service[3] - start_service[7] >= -999999
 NodeArrivalVehicles[7,4]: - 1.0000468021655091e+06 x[7,4]
   + start_service[4] - start_service[7] >= -999999
 NodeArrivalVehicles[7,5]: - 1.0000422223602368e+06 x[7,5]
   + start_service[5] - start_service[7] >= -999999
 NodeArrivalVehicles[7,6]: - 1.0000766646394427e+06 x[7,6]
   + start_service[6] - start_service[7] >= -999999
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.000076022979941e+06 x[7,8]
   - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 1.0000468021655091e+06 x[7,9]
   - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.0000422223602368e+06 x[7,10]
   - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000766646394427e+06 x[7,11]
   - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 1.0000296123560457e+06 x[7,12]
   - start_service[7] + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 1.0000296123560457e+06 x[7,13]
   - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000455895751599e+06 x[8,1]
   + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.000076022979941e+06 x[8,2]
   + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0001033774021529e+06 x[8,4]
   + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.0000761130502367e+06 x[8,5]
   + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.0000507866606601e+06 x[8,6]
   + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.000076022979941e+06 x[8,7]
   + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0001033774021529e+06 x[8,9]
   - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.0000761130502367e+06 x[8,10]
   - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.0000507866606601e+06 x[8,11]
   - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 1.0000455895751599e+06 x[8,12]
   - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 1.0000455895751599e+06 x[8,13]
   - start_service[8] + start_service[13] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0000661657955264e+06 x[9,1]
   + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000468021655091e+06 x[9,2]
   + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0001033774021529e+06 x[9,3]
   + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.0000285700562182e+06 x[9,5]
   + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000767645100753e+06 x[9,6]
   + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 1.0000468021655091e+06 x[9,7]
   + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0001033774021529e+06 x[9,8]
   + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.0000285700562182e+06 x[9,10]
   - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000767645100753e+06 x[9,11]
   - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 1.0000661657955264e+06 x[9,12]
   - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 1.0000661657955264e+06 x[9,13]
   - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000467938523132e+06 x[10,1]
   + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000422223602368e+06 x[10,2]
   + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.0000761130502367e+06 x[10,3]
   + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.0000285700562182e+06 x[10,4]
   + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000477981369909e+06 x[10,6]
   + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.0000422223602368e+06 x[10,7]
   + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.0000761130502367e+06 x[10,8]
   + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.0000285700562182e+06 x[10,9]
   + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000477981369909e+06 x[10,11]
   - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 1.0000467938523132e+06 x[10,12]
   - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 1.0000467938523132e+06 x[10,13]
   - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[11,1]: - 1.000058535470724e+06 x[11,1]
   + start_service[1] - start_service[11] >= -999999
 NodeArrivalVehicles[11,2]: - 1.0000766646394427e+06 x[11,2]
   + start_service[2] - start_service[11] >= -999999
 NodeArrivalVehicles[11,3]: - 1.0000507866606601e+06 x[11,3]
   + start_service[3] - start_service[11] >= -999999
 NodeArrivalVehicles[11,4]: - 1.0000767645100753e+06 x[11,4]
   + start_service[4] - start_service[11] >= -999999
 NodeArrivalVehicles[11,5]: - 1.0000477981369909e+06 x[11,5]
   + start_service[5] - start_service[11] >= -999999
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000766646394427e+06 x[11,7]
   + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.0000507866606601e+06 x[11,8]
   + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000767645100753e+06 x[11,9]
   + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000477981369909e+06 x[11,10]
   + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 1.000058535470724e+06 x[11,12]
   - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 1.000058535470724e+06 x[11,13]
   - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 1.0000296123560457e+06 x[12,2]
   + start_service[2] - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 1.0000455895751599e+06 x[12,3]
   + start_service[3] - start_service[12] >= -999999
 NodeArrivalVehicles[12,4]: - 1.0000661657955264e+06 x[12,4]
   + start_service[4] - start_service[12] >= -999999
 NodeArrivalVehicles[12,5]: - 1.0000467938523132e+06 x[12,5]
   + start_service[5] - start_service[12] >= -999999
 NodeArrivalVehicles[12,6]: - 1.000058535470724e+06 x[12,6]
   + start_service[6] - start_service[12] >= -999999
 NodeArrivalVehicles[12,7]: - 1.0000296123560457e+06 x[12,7]
   + start_service[7] - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000455895751599e+06 x[12,8]
   + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000661657955264e+06 x[12,9]
   + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000467938523132e+06 x[12,10]
   + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.000058535470724e+06 x[12,11]
   + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999999
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] <= 6
 TotalRobotStarts: w[0,0] + w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] + w[1,0] + w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5] + w[1,6]
   + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12] + w[1,13] <= 7
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] <= 1
 RobotsFinishAtVehicleDepot: w[0,13] + w[1,13] + w[2,13] + w[3,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] <= 6
 TotalRobotFinishes: w[0,12] + w[0,13] + w[1,12] + w[1,13] + w[2,12]
   + w[2,13] + w[3,12] + w[3,13] + w[4,12] + w[4,13] + w[5,12] + w[5,13]
   + w[6,12] + w[6,13] + w[7,12] + w[7,13] + w[8,12] + w[8,13] + w[9,12]
   + w[9,13] + w[10,12] + w[10,13] + w[11,12] + w[11,13] <= 7
 FlowConservationRobots[2]: w[0,2] + w[1,2] - w[2,1] - w[2,3] - w[2,4]
   - w[2,5] - w[2,6] - w[2,7] - w[2,8] - w[2,9] - w[2,10] - w[2,11]
   - w[2,12] - w[2,13] + w[3,2] + w[4,2] + w[5,2] + w[6,2] + w[7,2]
   + w[8,2] + w[9,2] + w[10,2] + w[11,2] + w[12,2] = 0
 FlowConservationRobots[3]: w[0,3] + w[1,3] + w[2,3] - w[3,1] - w[3,2]
   - w[3,4] - w[3,5] - w[3,6] - w[3,7] - w[3,8] - w[3,9] - w[3,10]
   - w[3,11] - w[3,12] - w[3,13] + w[4,3] + w[5,3] + w[6,3] + w[7,3]
   + w[8,3] + w[9,3] + w[10,3] + w[11,3] + w[12,3] = 0
 FlowConservationRobots[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] - w[4,1]
   - w[4,2] - w[4,3] - w[4,5] - w[4,6] - w[4,7] - w[4,8] - w[4,9] - w[4,10]
   - w[4,11] - w[4,12] - w[4,13] + w[5,4] + w[6,4] + w[7,4] + w[8,4]
   + w[9,4] + w[10,4] + w[11,4] + w[12,4] = 0
 FlowConservationRobots[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   - w[5,1] - w[5,2] - w[5,3] - w[5,4] - w[5,6] - w[5,7] - w[5,8] - w[5,9]
   - w[5,10] - w[5,11] - w[5,12] - w[5,13] + w[6,5] + w[7,5] + w[8,5]
   + w[9,5] + w[10,5] + w[11,5] + w[12,5] = 0
 FlowConservationRobots[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] - w[6,1] - w[6,2] - w[6,3] - w[6,4] - w[6,5] - w[6,7] - w[6,8]
   - w[6,9] - w[6,10] - w[6,11] - w[6,12] - w[6,13] + w[7,6] + w[8,6]
   + w[9,6] + w[10,6] + w[11,6] + w[12,6] = 0
 FlowConservationRobots[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] - w[7,1] - w[7,2] - w[7,3] - w[7,4] - w[7,5] - w[7,6]
   - w[7,8] - w[7,9] - w[7,10] - w[7,11] - w[7,12] - w[7,13] + w[8,7]
   + w[9,7] + w[10,7] + w[11,7] + w[12,7] = 0
 FlowConservationRobots[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] - w[8,1] - w[8,2] - w[8,3] - w[8,4] - w[8,5]
   - w[8,6] - w[8,7] - w[8,9] - w[8,10] - w[8,11] - w[8,12] - w[8,13]
   + w[9,8] + w[10,8] + w[11,8] + w[12,8] = 0
 FlowConservationRobots[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] - w[9,1] - w[9,2] - w[9,3] - w[9,4]
   - w[9,5] - w[9,6] - w[9,7] - w[9,8] - w[9,10] - w[9,11] - w[9,12]
   - w[9,13] + w[10,9] + w[11,9] + w[12,9] = 0
 FlowConservationRobots[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] - w[10,1]
   - w[10,2] - w[10,3] - w[10,4] - w[10,5] - w[10,6] - w[10,7] - w[10,8]
   - w[10,9] - w[10,11] - w[10,12] - w[10,13] + w[11,10] + w[12,10] = 0
 FlowConservationRobots[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   - w[11,1] - w[11,2] - w[11,3] - w[11,4] - w[11,5] - w[11,6] - w[11,7]
   - w[11,8] - w[11,9] - w[11,10] - w[11,12] - w[11,13] + w[12,11] = 0
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
 RobotsEndService[2]: - service_time[2] - start_service[2]
   + start_service[7] >= 0
 RobotsEndService[3]: - service_time[3] - start_service[3]
   + start_service[8] >= 0
 RobotsEndService[4]: - service_time[4] - start_service[4]
   + start_service[9] >= 0
 RobotsEndService[5]: - service_time[5] - start_service[5]
   + start_service[10] >= 0
 RobotsEndService[6]: - service_time[6] - start_service[6]
   + start_service[11] >= 0
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
 VehicleRobotSynchronization1[2,0]: - 2 x[2,0] + w[2,0] <= 0
 VehicleRobotSynchronization1[2,1]: - 2 x[2,1] + w[2,1] <= 0
 VehicleRobotSynchronization1[2,3]: - 2 x[2,3] + w[2,3] <= 0
 VehicleRobotSynchronization1[2,4]: - 2 x[2,4] + w[2,4] <= 0
 VehicleRobotSynchronization1[2,5]: - 2 x[2,5] + w[2,5] <= 0
 VehicleRobotSynchronization1[2,6]: - 2 x[2,6] + w[2,6] <= 0
 VehicleRobotSynchronization1[2,8]: - 2 x[2,8] + w[2,8] <= 0
 VehicleRobotSynchronization1[2,9]: - 2 x[2,9] + w[2,9] <= 0
 VehicleRobotSynchronization1[2,10]: - 2 x[2,10] + w[2,10] <= 0
 VehicleRobotSynchronization1[2,11]: - 2 x[2,11] + w[2,11] <= 0
 VehicleRobotSynchronization1[2,12]: - 2 x[2,12] + w[2,12] <= 0
 VehicleRobotSynchronization1[2,13]: - 2 x[2,13] + w[2,13] <= 0
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
 VehicleRobotSynchronization1[7,0]: - 2 x[7,0] + w[7,0] <= 0
 VehicleRobotSynchronization1[7,1]: - 2 x[7,1] + w[7,1] <= 0
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
 VehicleRobotSynchronization1[12,0]: - 2 x[12,0] + w[12,0] <= 0
 VehicleRobotSynchronization1[12,1]: - 2 x[12,1] + w[12,1] <= 0
 VehicleRobotSynchronization1[12,2]: - 2 x[12,2] + w[12,2] <= 0
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
 SettingMode[2]: - w[2,7] + y[2,1] + 2 y[2,2] = 0
 SettingMode[3]: - w[3,8] + y[3,1] + 2 y[3,2] = 0
 SettingMode[4]: - w[4,9] + y[4,1] + 2 y[4,2] = 0
 SettingMode[5]: - w[5,10] + y[5,1] + 2 y[5,2] = 0
 SettingMode[6]: - w[6,11] + y[6,1] + 2 y[6,2] = 0
 SettingServiceTime[2]: - 440 y[2,1] - 220 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 300 y[3,1] - 150 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 230 y[4,1] - 115 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 430 y[5,1] - 215 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 310 y[6,1] - 155 y[6,2] + service_time[6] = 0
 AverageTimeRobots: - 4.373193720816657 w[0,2] - 6.655653594272122 w[0,3]
   - 9.595113646627372 w[0,4] - 6.827693187606528 w[0,5]
   - 8.505067246276667 w[0,6] - 4.373193720816657 w[0,7]
   - 6.655653594272122 w[0,8] - 9.595113646627372 w[0,9]
   - 6.827693187606528 w[0,10] - 8.505067246276667 w[0,11]
   - 4.373193720816657 w[1,2] - 6.655653594272122 w[1,3]
   - 9.595113646627372 w[1,4] - 6.827693187606528 w[1,5]
   - 8.505067246276667 w[1,6] - 4.373193720816657 w[1,7]
   - 6.655653594272122 w[1,8] - 9.595113646627372 w[1,9]
   - 6.827693187606528 w[1,10] - 8.505067246276667 w[1,11]
   - 4.373193720816657 w[2,1] - 11.00328284871858 w[2,3]
   - 6.828880787016356 w[2,4] - 6.174622890973365 w[2,5]
   - 11.09494849181722 w[2,6] - 11.00328284871858 w[2,8]
   - 6.828880787016356 w[2,9] - 6.174622890973365 w[2,10]
   - 11.09494849181722 w[2,11] - 4.373193720816657 w[2,12]
   - 4.373193720816657 w[2,13] - 6.655653594272122 w[3,1]
   - 11.00328284871858 w[3,2] - 14.91105745041063 w[3,4]
   - 11.01615003381325 w[3,5] - 7.398094380000198 w[3,6]
   - 11.00328284871858 w[3,7] - 14.91105745041063 w[3,9]
   - 11.01615003381325 w[3,10] - 7.398094380000198 w[3,11]
   - 6.655653594272122 w[3,12] - 6.655653594272122 w[3,13]
   - 9.595113646627372 w[4,1] - 6.828880787016356 w[4,2]
   - 14.91105745041063 w[4,3] - 4.224293745460967 w[4,5]
   - 11.10921572504186 w[4,6] - 6.828880787016356 w[4,7]
   - 14.91105745041063 w[4,8] - 4.224293745460967 w[4,10]
   - 11.10921572504186 w[4,11] - 9.595113646627372 w[4,12]
   - 9.595113646627372 w[4,13] - 6.827693187606528 w[5,1]
   - 6.174622890973365 w[5,2] - 11.01615003381325 w[5,3]
   - 4.224293745460967 w[5,4] - 6.971162427265043 w[5,6]
   - 6.174622890973365 w[5,7] - 11.01615003381325 w[5,8]
   - 4.224293745460967 w[5,9] - 6.971162427265043 w[5,11]
   - 6.827693187606528 w[5,12] - 6.827693187606528 w[5,13]
   - 8.505067246276667 w[6,1] - 11.09494849181722 w[6,2]
   - 7.398094380000198 w[6,3] - 11.10921572504186 w[6,4]
   - 6.971162427265043 w[6,5] - 11.09494849181722 w[6,7]
   - 7.398094380000198 w[6,8] - 11.10921572504186 w[6,9]
   - 6.971162427265043 w[6,10] - 8.505067246276667 w[6,12]
   - 8.505067246276667 w[6,13] - 4.373193720816657 w[7,1]
   - 11.00328284871858 w[7,3] - 6.828880787016356 w[7,4]
   - 6.174622890973365 w[7,5] - 11.09494849181722 w[7,6]
   - 11.00328284871858 w[7,8] - 6.828880787016356 w[7,9]
   - 6.174622890973365 w[7,10] - 11.09494849181722 w[7,11]
   - 4.373193720816657 w[7,12] - 4.373193720816657 w[7,13]
   - 6.655653594272122 w[8,1] - 11.00328284871858 w[8,2]
   - 14.91105745041063 w[8,4] - 11.01615003381325 w[8,5]
   - 7.398094380000198 w[8,6] - 11.00328284871858 w[8,7]
   - 14.91105745041063 w[8,9] - 11.01615003381325 w[8,10]
   - 7.398094380000198 w[8,11] - 6.655653594272122 w[8,12]
   - 6.655653594272122 w[8,13] - 9.595113646627372 w[9,1]
   - 6.828880787016356 w[9,2] - 14.91105745041063 w[9,3]
   - 4.224293745460967 w[9,5] - 11.10921572504186 w[9,6]
   - 6.828880787016356 w[9,7] - 14.91105745041063 w[9,8]
   - 4.224293745460967 w[9,10] - 11.10921572504186 w[9,11]
   - 9.595113646627372 w[9,12] - 9.595113646627372 w[9,13]
   - 6.827693187606528 w[10,1] - 6.174622890973365 w[10,2]
   - 11.01615003381325 w[10,3] - 4.224293745460967 w[10,4]
   - 6.971162427265043 w[10,6] - 6.174622890973365 w[10,7]
   - 11.01615003381325 w[10,8] - 4.224293745460967 w[10,9]
   - 6.971162427265043 w[10,11] - 6.827693187606528 w[10,12]
   - 6.827693187606528 w[10,13] - 8.505067246276667 w[11,1]
   - 11.09494849181722 w[11,2] - 7.398094380000198 w[11,3]
   - 11.10921572504186 w[11,4] - 6.971162427265043 w[11,5]
   - 11.09494849181722 w[11,7] - 7.398094380000198 w[11,8]
   - 11.10921572504186 w[11,9] - 6.971162427265043 w[11,10]
   - 8.505067246276667 w[11,12] - 8.505067246276667 w[11,13]
   - 4.373193720816657 w[12,2] - 6.655653594272122 w[12,3]
   - 9.595113646627372 w[12,4] - 6.827693187606528 w[12,5]
   - 8.505067246276667 w[12,6] - 4.373193720816657 w[12,7]
   - 6.655653594272122 w[12,8] - 9.595113646627372 w[12,9]
   - 6.827693187606528 w[12,10] - 8.505067246276667 w[12,11]
   - 62.85714285714285 y[2,1] - 31.42857142857143 y[2,2]
   - 42.85714285714285 y[3,1] - 21.42857142857143 y[3,2]
   - 32.85714285714285 y[4,1] - 16.42857142857143 y[4,2]
   - 61.42857142857142 y[5,1] - 30.71428571428571 y[5,2]
   - 44.28571428571428 y[6,1] - 22.14285714285714 y[6,2]
   + start_service[13] >= 0
 AverageTimeVehicles: - 10.20411868190553 x[0,2] - 15.52985838663495 x[0,3]
   - 22.3885985087972 x[0,4] - 15.93128410441523 x[0,5]
   - 19.84515690797889 x[0,6] - 10.20411868190553 x[0,7]
   - 15.52985838663495 x[0,8] - 22.3885985087972 x[0,9]
   - 15.93128410441523 x[0,10] - 19.84515690797889 x[0,11]
   - 10.20411868190553 x[1,2] - 15.52985838663495 x[1,3]
   - 22.3885985087972 x[1,4] - 15.93128410441523 x[1,5]
   - 19.84515690797889 x[1,6] - 10.20411868190553 x[1,7]
   - 15.52985838663495 x[1,8] - 22.3885985087972 x[1,9]
   - 15.93128410441523 x[1,10] - 19.84515690797889 x[1,11]
   - 10.20411868190553 x[2,1] - 25.67432664701001 x[2,3]
   - 15.93405516970483 x[2,4] - 14.40745341227118 x[2,5]
   - 25.88821314757352 x[2,6] - 25.67432664701001 x[2,8]
   - 15.93405516970483 x[2,9] - 14.40745341227118 x[2,10]
   - 25.88821314757352 x[2,11] - 10.20411868190553 x[2,12]
   - 10.20411868190553 x[2,13] - 15.52985838663495 x[3,1]
   - 25.67432664701001 x[3,2] - 34.79246738429146 x[3,4]
   - 25.70435007889758 x[3,5] - 17.26222022000046 x[3,6]
   - 25.67432664701001 x[3,7] - 34.79246738429146 x[3,9]
   - 25.70435007889758 x[3,10] - 17.26222022000046 x[3,11]
   - 15.52985838663495 x[3,12] - 15.52985838663495 x[3,13]
   - 22.3885985087972 x[4,1] - 15.93405516970483 x[4,2]
   - 34.79246738429146 x[4,3] - 9.85668540607559 x[4,5]
   - 25.92150335843101 x[4,6] - 15.93405516970483 x[4,7]
   - 34.79246738429146 x[4,8] - 9.85668540607559 x[4,10]
   - 25.92150335843101 x[4,11] - 22.3885985087972 x[4,12]
   - 22.3885985087972 x[4,13] - 15.93128410441523 x[5,1]
   - 14.40745341227118 x[5,2] - 25.70435007889758 x[5,3]
   - 9.85668540607559 x[5,4] - 16.26604566361843 x[5,6]
   - 14.40745341227118 x[5,7] - 25.70435007889758 x[5,8]
   - 9.85668540607559 x[5,9] - 16.26604566361843 x[5,11]
   - 15.93128410441523 x[5,12] - 15.93128410441523 x[5,13]
   - 19.84515690797889 x[6,1] - 25.88821314757352 x[6,2]
   - 17.26222022000046 x[6,3] - 25.92150335843101 x[6,4]
   - 16.26604566361843 x[6,5] - 25.88821314757352 x[6,7]
   - 17.26222022000046 x[6,8] - 25.92150335843101 x[6,9]
   - 16.26604566361843 x[6,10] - 19.84515690797889 x[6,12]
   - 19.84515690797889 x[6,13] - 10.20411868190553 x[7,1]
   - 25.67432664701001 x[7,3] - 15.93405516970483 x[7,4]
   - 14.40745341227118 x[7,5] - 25.88821314757352 x[7,6]
   - 25.67432664701001 x[7,8] - 15.93405516970483 x[7,9]
   - 14.40745341227118 x[7,10] - 25.88821314757352 x[7,11]
   - 10.20411868190553 x[7,12] - 10.20411868190553 x[7,13]
   - 15.52985838663495 x[8,1] - 25.67432664701001 x[8,2]
   - 34.79246738429146 x[8,4] - 25.70435007889758 x[8,5]
   - 17.26222022000046 x[8,6] - 25.67432664701001 x[8,7]
   - 34.79246738429146 x[8,9] - 25.70435007889758 x[8,10]
   - 17.26222022000046 x[8,11] - 15.52985838663495 x[8,12]
   - 15.52985838663495 x[8,13] - 22.3885985087972 x[9,1]
   - 15.93405516970483 x[9,2] - 34.79246738429146 x[9,3]
   - 9.85668540607559 x[9,5] - 25.92150335843101 x[9,6]
   - 15.93405516970483 x[9,7] - 34.79246738429146 x[9,8]
   - 9.85668540607559 x[9,10] - 25.92150335843101 x[9,11]
   - 22.3885985087972 x[9,12] - 22.3885985087972 x[9,13]
   - 15.93128410441523 x[10,1] - 14.40745341227118 x[10,2]
   - 25.70435007889758 x[10,3] - 9.85668540607559 x[10,4]
   - 16.26604566361843 x[10,6] - 14.40745341227118 x[10,7]
   - 25.70435007889758 x[10,8] - 9.85668540607559 x[10,9]
   - 16.26604566361843 x[10,11] - 15.93128410441523 x[10,12]
   - 15.93128410441523 x[10,13] - 19.84515690797889 x[11,1]
   - 25.88821314757352 x[11,2] - 17.26222022000046 x[11,3]
   - 25.92150335843101 x[11,4] - 16.26604566361843 x[11,5]
   - 25.88821314757352 x[11,7] - 17.26222022000046 x[11,8]
   - 25.92150335843101 x[11,9] - 16.26604566361843 x[11,10]
   - 19.84515690797889 x[11,12] - 19.84515690797889 x[11,13]
   - 10.20411868190553 x[12,2] - 15.52985838663495 x[12,3]
   - 22.3885985087972 x[12,4] - 15.93128410441523 x[12,5]
   - 19.84515690797889 x[12,6] - 10.20411868190553 x[12,7]
   - 15.52985838663495 x[12,8] - 22.3885985087972 x[12,9]
   - 15.93128410441523 x[12,10] - 19.84515690797889 x[12,11]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[0]: - 61.2247120914332 x[0,2]
   - 93.17915031980972 x[0,3] - 134.3315910527832 x[0,4]
   - 95.58770462649139 x[0,5] - 119.0709414478734 x[0,6]
   - 61.2247120914332 x[0,7] - 93.17915031980972 x[0,8]
   - 134.3315910527832 x[0,9] - 95.58770462649139 x[0,10]
   - 119.0709414478734 x[0,11] - start_service[0] + start_service[13] >= 0
 LowerBoundRoutingVehicle[1]: - 61.2247120914332 x[1,2]
   - 93.17915031980972 x[1,3] - 134.3315910527832 x[1,4]
   - 95.58770462649139 x[1,5] - 119.0709414478734 x[1,6]
   - 61.2247120914332 x[1,7] - 93.17915031980972 x[1,8]
   - 134.3315910527832 x[1,9] - 95.58770462649139 x[1,10]
   - 119.0709414478734 x[1,11] - start_service[1] + start_service[13] >= 0
 LowerBoundRoutingVehicle[2]: - 30.6123560457166 x[2,1]
   - 30.6123560457166 x[2,2] - 123.6125551009349 x[2,3]
   - 114.9679610355061 x[2,4] - 91.01621255005925 x[2,5]
   - 137.2001101666573 x[2,6] - 30.6123560457166 x[2,7]
   - 123.6125551009349 x[2,8] - 114.9679610355061 x[2,9]
   - 91.01621255005925 x[2,10] - 137.2001101666573 x[2,11]
   - 30.6123560457166 x[2,12] - 30.6123560457166 x[2,13] - start_service[2]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[3]: - 46.58957515990486 x[3,1]
   - 107.6353359867466 x[3,2] - 46.58957515990486 x[3,3]
   - 171.543197679266 x[3,4] - 124.9069025499384 x[3,5]
   - 111.3221313839381 x[3,6] - 107.6353359867466 x[3,7]
   - 46.58957515990486 x[3,8] - 171.543197679266 x[3,9]
   - 124.9069025499384 x[3,10] - 111.3221313839381 x[3,11]
   - 46.58957515990486 x[3,12] - 46.58957515990486 x[3,13]
   - start_service[3] + start_service[13] >= 0
 LowerBoundRoutingVehicle[4]: - 67.16579552639161 x[4,1]
   - 78.4145215548311 x[4,2] - 150.9669773127792 x[4,3]
   - 67.16579552639161 x[4,4] - 77.36390853147246 x[4,5]
   - 137.2999807992297 x[4,6] - 78.4145215548311 x[4,7]
   - 150.9669773127792 x[4,8] - 67.16579552639161 x[4,9]
   - 77.36390853147246 x[4,10] - 137.2999807992297 x[4,11]
   - 67.16579552639161 x[4,12] - 67.16579552639161 x[4,13]
   - start_service[4] + start_service[13] >= 0
 LowerBoundRoutingVehicle[5]: - 47.79385231324569 x[5,1]
   - 73.83471628253015 x[5,2] - 123.7026253965976 x[5,3]
   - 96.73585174461837 x[5,4] - 47.79385231324569 x[5,5]
   - 108.333607714792 x[5,6] - 73.83471628253015 x[5,7]
   - 123.7026253965976 x[5,8] - 96.73585174461837 x[5,9]
   - 47.79385231324569 x[5,10] - 108.333607714792 x[5,11]
   - 47.79385231324569 x[5,12] - 47.79385231324569 x[5,13]
   - start_service[5] + start_service[13] >= 0
 LowerBoundRoutingVehicle[6]: - 59.53547072393668 x[6,1]
   - 108.2769954884372 x[6,2] - 98.37623581990624 x[6,3]
   - 144.9303056016846 x[6,4] - 96.59198930410099 x[6,5]
   - 59.53547072393668 x[6,6] - 108.2769954884372 x[6,7]
   - 98.37623581990624 x[6,8] - 144.9303056016846 x[6,9]
   - 96.59198930410099 x[6,10] - 59.53547072393668 x[6,11]
   - 59.53547072393668 x[6,12] - 59.53547072393668 x[6,13]
   - start_service[6] + start_service[13] >= 0
 LowerBoundRoutingVehicle[7]: - 30.6123560457166 x[7,1]
   - 30.6123560457166 x[7,2] - 123.6125551009349 x[7,3]
   - 114.9679610355061 x[7,4] - 91.01621255005925 x[7,5]
   - 137.2001101666573 x[7,6] - 30.6123560457166 x[7,7]
   - 123.6125551009349 x[7,8] - 114.9679610355061 x[7,9]
   - 91.01621255005925 x[7,10] - 137.2001101666573 x[7,11]
   - 30.6123560457166 x[7,12] - 30.6123560457166 x[7,13] - start_service[7]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[8]: - 46.58957515990486 x[8,1]
   - 107.6353359867466 x[8,2] - 46.58957515990486 x[8,3]
   - 171.543197679266 x[8,4] - 124.9069025499384 x[8,5]
   - 111.3221313839381 x[8,6] - 107.6353359867466 x[8,7]
   - 46.58957515990486 x[8,8] - 171.543197679266 x[8,9]
   - 124.9069025499384 x[8,10] - 111.3221313839381 x[8,11]
   - 46.58957515990486 x[8,12] - 46.58957515990486 x[8,13]
   - start_service[8] + start_service[13] >= 0
 LowerBoundRoutingVehicle[9]: - 67.16579552639161 x[9,1]
   - 78.4145215548311 x[9,2] - 150.9669773127792 x[9,3]
   - 67.16579552639161 x[9,4] - 77.36390853147246 x[9,5]
   - 137.2999807992297 x[9,6] - 78.4145215548311 x[9,7]
   - 150.9669773127792 x[9,8] - 67.16579552639161 x[9,9]
   - 77.36390853147246 x[9,10] - 137.2999807992297 x[9,11]
   - 67.16579552639161 x[9,12] - 67.16579552639161 x[9,13]
   - start_service[9] + start_service[13] >= 0
 LowerBoundRoutingVehicle[10]: - 47.79385231324569 x[10,1]
   - 73.83471628253015 x[10,2] - 123.7026253965976 x[10,3]
   - 96.73585174461837 x[10,4] - 47.79385231324569 x[10,5]
   - 108.333607714792 x[10,6] - 73.83471628253015 x[10,7]
   - 123.7026253965976 x[10,8] - 96.73585174461837 x[10,9]
   - 47.79385231324569 x[10,10] - 108.333607714792 x[10,11]
   - 47.79385231324569 x[10,12] - 47.79385231324569 x[10,13]
   - start_service[10] + start_service[13] >= 0
 LowerBoundRoutingVehicle[11]: - 59.53547072393668 x[11,1]
   - 108.2769954884372 x[11,2] - 98.37623581990624 x[11,3]
   - 144.9303056016846 x[11,4] - 96.59198930410099 x[11,5]
   - 59.53547072393668 x[11,6] - 108.2769954884372 x[11,7]
   - 98.37623581990624 x[11,8] - 144.9303056016846 x[11,9]
   - 96.59198930410099 x[11,10] - 59.53547072393668 x[11,11]
   - 59.53547072393668 x[11,12] - 59.53547072393668 x[11,13]
   - start_service[11] + start_service[13] >= 0
 LowerBoundRoutingVehicle[12]: - 61.2247120914332 x[12,2]
   - 93.17915031980972 x[12,3] - 134.3315910527832 x[12,4]
   - 95.58770462649139 x[12,5] - 119.0709414478734 x[12,6]
   - 61.2247120914332 x[12,7] - 93.17915031980972 x[12,8]
   - 134.3315910527832 x[12,9] - 95.58770462649139 x[12,10]
   - 119.0709414478734 x[12,11] - start_service[12] + start_service[13]
   >= 0
 LowerBoundArrivalVehicle[1]: - 61.2247120914332 x[2,1]
   - 93.17915031980972 x[3,1] - 134.3315910527832 x[4,1]
   - 95.58770462649139 x[5,1] - 119.0709414478734 x[6,1]
   - 61.2247120914332 x[7,1] - 93.17915031980972 x[8,1]
   - 134.3315910527832 x[9,1] - 95.58770462649139 x[10,1]
   - 119.0709414478734 x[11,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 30.6123560457166 x[0,2]
   - 30.6123560457166 x[1,2] - 30.6123560457166 x[2,2]
   - 123.6125551009349 x[3,2] - 114.9679610355061 x[4,2]
   - 91.01621255005925 x[5,2] - 137.2001101666573 x[6,2]
   - 30.6123560457166 x[7,2] - 123.6125551009349 x[8,2]
   - 114.9679610355061 x[9,2] - 91.01621255005925 x[10,2]
   - 137.2001101666573 x[11,2] - 30.6123560457166 x[12,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 46.58957515990486 x[0,3]
   - 46.58957515990486 x[1,3] - 107.6353359867466 x[2,3]
   - 46.58957515990486 x[3,3] - 171.543197679266 x[4,3]
   - 124.9069025499384 x[5,3] - 111.3221313839381 x[6,3]
   - 107.6353359867466 x[7,3] - 46.58957515990486 x[8,3]
   - 171.543197679266 x[9,3] - 124.9069025499384 x[10,3]
   - 111.3221313839381 x[11,3] - 46.58957515990486 x[12,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 67.16579552639161 x[0,4]
   - 67.16579552639161 x[1,4] - 78.4145215548311 x[2,4]
   - 150.9669773127792 x[3,4] - 67.16579552639161 x[4,4]
   - 77.36390853147246 x[5,4] - 137.2999807992297 x[6,4]
   - 78.4145215548311 x[7,4] - 150.9669773127792 x[8,4]
   - 67.16579552639161 x[9,4] - 77.36390853147246 x[10,4]
   - 137.2999807992297 x[11,4] - 67.16579552639161 x[12,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 47.79385231324569 x[0,5]
   - 47.79385231324569 x[1,5] - 73.83471628253015 x[2,5]
   - 123.7026253965976 x[3,5] - 96.73585174461837 x[4,5]
   - 47.79385231324569 x[5,5] - 108.333607714792 x[6,5]
   - 73.83471628253015 x[7,5] - 123.7026253965976 x[8,5]
   - 96.73585174461837 x[9,5] - 47.79385231324569 x[10,5]
   - 108.333607714792 x[11,5] - 47.79385231324569 x[12,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 59.53547072393668 x[0,6]
   - 59.53547072393668 x[1,6] - 108.2769954884372 x[2,6]
   - 98.37623581990624 x[3,6] - 144.9303056016846 x[4,6]
   - 96.59198930410099 x[5,6] - 59.53547072393668 x[6,6]
   - 108.2769954884372 x[7,6] - 98.37623581990624 x[8,6]
   - 144.9303056016846 x[9,6] - 96.59198930410099 x[10,6]
   - 59.53547072393668 x[11,6] - 59.53547072393668 x[12,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 30.6123560457166 x[0,7]
   - 30.6123560457166 x[1,7] - 30.6123560457166 x[2,7]
   - 123.6125551009349 x[3,7] - 114.9679610355061 x[4,7]
   - 91.01621255005925 x[5,7] - 137.2001101666573 x[6,7]
   - 30.6123560457166 x[7,7] - 123.6125551009349 x[8,7]
   - 114.9679610355061 x[9,7] - 91.01621255005925 x[10,7]
   - 137.2001101666573 x[11,7] - 30.6123560457166 x[12,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 46.58957515990486 x[0,8]
   - 46.58957515990486 x[1,8] - 107.6353359867466 x[2,8]
   - 46.58957515990486 x[3,8] - 171.543197679266 x[4,8]
   - 124.9069025499384 x[5,8] - 111.3221313839381 x[6,8]
   - 107.6353359867466 x[7,8] - 46.58957515990486 x[8,8]
   - 171.543197679266 x[9,8] - 124.9069025499384 x[10,8]
   - 111.3221313839381 x[11,8] - 46.58957515990486 x[12,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 67.16579552639161 x[0,9]
   - 67.16579552639161 x[1,9] - 78.4145215548311 x[2,9]
   - 150.9669773127792 x[3,9] - 67.16579552639161 x[4,9]
   - 77.36390853147246 x[5,9] - 137.2999807992297 x[6,9]
   - 78.4145215548311 x[7,9] - 150.9669773127792 x[8,9]
   - 67.16579552639161 x[9,9] - 77.36390853147246 x[10,9]
   - 137.2999807992297 x[11,9] - 67.16579552639161 x[12,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 47.79385231324569 x[0,10]
   - 47.79385231324569 x[1,10] - 73.83471628253015 x[2,10]
   - 123.7026253965976 x[3,10] - 96.73585174461837 x[4,10]
   - 47.79385231324569 x[5,10] - 108.333607714792 x[6,10]
   - 73.83471628253015 x[7,10] - 123.7026253965976 x[8,10]
   - 96.73585174461837 x[9,10] - 47.79385231324569 x[10,10]
   - 108.333607714792 x[11,10] - 47.79385231324569 x[12,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 59.53547072393668 x[0,11]
   - 59.53547072393668 x[1,11] - 108.2769954884372 x[2,11]
   - 98.37623581990624 x[3,11] - 144.9303056016846 x[4,11]
   - 96.59198930410099 x[5,11] - 59.53547072393668 x[6,11]
   - 108.2769954884372 x[7,11] - 98.37623581990624 x[8,11]
   - 144.9303056016846 x[9,11] - 96.59198930410099 x[10,11]
   - 59.53547072393668 x[11,11] - 59.53547072393668 x[12,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 61.2247120914332 x[2,12]
   - 93.17915031980972 x[3,12] - 134.3315910527832 x[4,12]
   - 95.58770462649139 x[5,12] - 119.0709414478734 x[6,12]
   - 61.2247120914332 x[7,12] - 93.17915031980972 x[8,12]
   - 134.3315910527832 x[9,12] - 95.58770462649139 x[10,12]
   - 119.0709414478734 x[11,12] - start_service[0] + start_service[12]
   >= 0
 LowerBoundArrivalVehicle[13]: - 61.2247120914332 x[2,13]
   - 93.17915031980972 x[3,13] - 134.3315910527832 x[4,13]
   - 95.58770462649139 x[5,13] - 119.0709414478734 x[6,13]
   - 61.2247120914332 x[7,13] - 93.17915031980972 x[8,13]
   - 134.3315910527832 x[9,13] - 95.58770462649139 x[10,13]
   - 119.0709414478734 x[11,13] - start_service[0] + start_service[13]
   >= 0
 VisitArrivalNodesNotEmpty[2]: w[0,2] + w[1,2] + w[2,2] + w[3,2] + w[4,2]
   + w[5,2] + w[6,2] + w[7,2] + w[8,2] + w[9,2] + w[10,2] + w[11,2]
   + w[12,2] >= 1
 VisitArrivalNodesNotEmpty[3]: w[0,3] + w[1,3] + w[2,3] + w[3,3] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] >= 1
 VisitArrivalNodesNotEmpty[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] + w[4,4]
   + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4]
   + w[12,4] >= 1
 VisitArrivalNodesNotEmpty[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   + w[5,5] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5]
   + w[12,5] >= 1
 VisitArrivalNodesNotEmpty[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] + w[6,6] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6]
   + w[12,6] >= 1
 LeaveArrivalNodesNotFullyLoaded[2]: w[2,0] + w[2,1] + w[2,3] + w[2,4]
   + w[2,5] + w[2,6] + w[2,8] + w[2,9] + w[2,10] + w[2,11] + w[2,12]
   + w[2,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[3]: w[3,0] + w[3,1] + w[3,2] + w[3,4]
   + w[3,5] + w[3,6] + w[3,7] + w[3,9] + w[3,10] + w[3,11] + w[3,12]
   + w[3,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[4]: w[4,0] + w[4,1] + w[4,2] + w[4,3]
   + w[4,5] + w[4,6] + w[4,7] + w[4,8] + w[4,10] + w[4,11] + w[4,12]
   + w[4,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[5]: w[5,0] + w[5,1] + w[5,2] + w[5,3]
   + w[5,4] + w[5,6] + w[5,7] + w[5,8] + w[5,9] + w[5,11] + w[5,12]
   + w[5,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[6]: w[6,0] + w[6,1] + w[6,2] + w[6,3]
   + w[6,4] + w[6,5] + w[6,7] + w[6,8] + w[6,9] + w[6,10] + w[6,12]
   + w[6,13] <= 1
 VisitDepartureNodesNotFullyLoaded[7]: w[0,7] + w[1,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] + w[8,7] + w[9,7] + w[10,7] + w[11,7] + w[12,7]
   + w[13,7] <= 1
 VisitDepartureNodesNotFullyLoaded[8]: w[0,8] + w[1,8] + w[2,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] + w[9,8] + w[10,8] + w[11,8] + w[12,8]
   + w[13,8] <= 1
 VisitDepartureNodesNotFullyLoaded[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] + w[10,9] + w[11,9] + w[12,9]
   + w[13,9] <= 1
 VisitDepartureNodesNotFullyLoaded[10]: w[0,10] + w[1,10] + w[2,10]
   + w[3,10] + w[4,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] + w[11,10]
   + w[12,10] + w[13,10] <= 1
 VisitDepartureNodesNotFullyLoaded[11]: w[0,11] + w[1,11] + w[2,11]
   + w[3,11] + w[4,11] + w[5,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   + w[12,11] + w[13,11] <= 1
 LeaveDepartureNodesNotEmpty[7]: w[7,1] + w[7,2] + w[7,3] + w[7,4] + w[7,5]
   + w[7,6] + w[7,7] + w[7,8] + w[7,9] + w[7,10] + w[7,11] + w[7,12]
   + w[7,13] >= 1
 LeaveDepartureNodesNotEmpty[8]: w[8,1] + w[8,2] + w[8,3] + w[8,4] + w[8,5]
   + w[8,6] + w[8,7] + w[8,8] + w[8,9] + w[8,10] + w[8,11] + w[8,12]
   + w[8,13] >= 1
 LeaveDepartureNodesNotEmpty[9]: w[9,1] + w[9,2] + w[9,3] + w[9,4] + w[9,5]
   + w[9,6] + w[9,7] + w[9,8] + w[9,9] + w[9,10] + w[9,11] + w[9,12]
   + w[9,13] >= 1
 LeaveDepartureNodesNotEmpty[10]: w[10,1] + w[10,2] + w[10,3] + w[10,4]
   + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,10] + w[10,11]
   + w[10,12] + w[10,13] >= 1
 LeaveDepartureNodesNotEmpty[11]: w[11,1] + w[11,2] + w[11,3] + w[11,4]
   + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10] + w[11,11]
   + w[11,12] + w[11,13] >= 1
Bounds
 x[0,0] = 0
 x[0,1] = 0
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
 x[7,2] = 0
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
 x[12,12] = 0
 x[12,13] = 0
 x[13,0] = 0
 x[13,1] = 0
 x[13,2] = 0
 x[13,3] = 0
 x[13,4] = 0
 x[13,5] = 0
 x[13,6] = 0
 x[13,7] = 0
 x[13,8] = 0
 x[13,9] = 0
 x[13,10] = 0
 x[13,11] = 0
 x[13,12] = 0
 x[13,13] = 0
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
 w[7,0] = -0
 w[7,1] = -0
 w[7,2] = -0
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
 w[8,0] = -0
 w[8,1] = -0
 w[8,2] <= 2
 w[8,3] = -0
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
 w[9,0] = -0
 w[9,1] = -0
 w[9,2] <= 2
 w[9,3] <= 2
 w[9,4] = -0
 w[9,5] <= 2
 w[9,6] <= 2
 w[9,7] <= 2
 w[9,8] <= 2
 w[9,9] = -0
 w[9,10] <= 2
 w[9,11] <= 2
 w[9,12] <= 2
 w[9,13] <= 2
 w[10,0] = -0
 w[10,1] = -0
 w[10,2] <= 2
 w[10,3] <= 2
 w[10,4] <= 2
 w[10,5] = -0
 w[10,6] <= 2
 w[10,7] <= 2
 w[10,8] <= 2
 w[10,9] <= 2
 w[10,10] = -0
 w[10,11] <= 2
 w[10,12] <= 2
 w[10,13] <= 2
 w[11,0] = -0
 w[11,1] = -0
 w[11,2] <= 2
 w[11,3] <= 2
 w[11,4] <= 2
 w[11,5] <= 2
 w[11,6] = -0
 w[11,7] <= 2
 w[11,8] <= 2
 w[11,9] <= 2
 w[11,10] <= 2
 w[11,11] = -0
 w[11,12] <= 2
 w[11,13] <= 2
 w[12,0] = -0
 w[12,1] = -0
 w[12,2] = -0
 w[12,3] = -0
 w[12,4] = -0
 w[12,5] = -0
 w[12,6] = -0
 w[12,7] = -0
 w[12,8] = -0
 w[12,9] = -0
 w[12,10] = -0
 w[12,11] = -0
 w[12,12] = -0
 w[12,13] = -0
 w[13,0] = -0
 w[13,1] = -0
 w[13,2] = -0
 w[13,3] = -0
 w[13,4] = -0
 w[13,5] = -0
 w[13,6] = -0
 w[13,7] = -0
 w[13,8] = -0
 w[13,9] = -0
 w[13,10] = -0
 w[13,11] = -0
 w[13,12] = -0
 w[13,13] = -0
 v[0,0] = 0
 v[0,1] = 0
 v[1,0] = 0
 v[1,1] = 0
 v[2,0] = 0
 v[2,1] = 0
 v[2,2] = 0
 v[2,7] = 1
 v[3,0] = 0
 v[3,1] = 0
 v[3,3] = 0
 v[3,8] = 1
 v[4,0] = 0
 v[4,1] = 0
 v[4,4] = 0
 v[4,9] = 1
 v[5,0] = 0
 v[5,1] = 0
 v[5,5] = 0
 v[5,10] = 1
 v[6,0] = 0
 v[6,1] = 0
 v[6,6] = 0
 v[6,11] = 1
 v[7,0] = 0
 v[7,1] = 0
 v[7,2] = 0
 v[7,7] = 0
 v[8,0] = 0
 v[8,1] = 0
 v[8,3] = 0
 v[8,8] = 0
 v[9,0] = 0
 v[9,1] = 0
 v[9,4] = 0
 v[9,9] = 0
 v[10,0] = 0
 v[10,1] = 0
 v[10,5] = 0
 v[10,10] = 0
 v[11,0] = 0
 v[11,1] = 0
 v[11,6] = 0
 v[11,11] = 0
 v[12,0] = 0
 v[12,1] = 0
 v[12,2] = 0
 v[12,3] = 0
 v[12,4] = 0
 v[12,5] = 0
 v[12,6] = 0
 v[12,7] = 0
 v[12,8] = 0
 v[12,9] = 0
 v[12,10] = 0
 v[12,11] = 0
 v[12,12] = 0
 v[12,13] = 0
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
 service_time[0] = 0
 service_time[1] = 0
 service_time[7] = 0
 service_time[8] = 0
 service_time[9] = 0
 service_time[10] = 0
 service_time[11] = 0
 service_time[12] = 0
 service_time[13] = 0
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
Binaries
 x[0,0] x[0,1] x[0,2] x[0,3] x[0,4] x[0,5] x[0,6] x[0,7] x[0,8] x[0,9]
 x[0,10] x[0,11] x[0,12] x[0,13] x[1,0] x[1,1] x[1,2] x[1,3] x[1,4] x[1,5]
 x[1,6] x[1,7] x[1,8] x[1,9] x[1,10] x[1,11] x[1,12] x[1,13] x[2,0] x[2,1]
 x[2,2] x[2,3] x[2,4] x[2,5] x[2,6] x[2,7] x[2,8] x[2,9] x[2,10] x[2,11]
 x[2,12] x[2,13] x[3,0] x[3,1] x[3,2] x[3,3] x[3,4] x[3,5] x[3,6] x[3,7]
 x[3,8] x[3,9] x[3,10] x[3,11] x[3,12] x[3,13] x[4,0] x[4,1] x[4,2] x[4,3]
 x[4,4] x[4,5] x[4,6] x[4,7] x[4,8] x[4,9] x[4,10] x[4,11] x[4,12] x[4,13]
 x[5,0] x[5,1] x[5,2] x[5,3] x[5,4] x[5,5] x[5,6] x[5,7] x[5,8] x[5,9]
 x[5,10] x[5,11] x[5,12] x[5,13] x[6,0] x[6,1] x[6,2] x[6,3] x[6,4] x[6,5]
 x[6,6] x[6,7] x[6,8] x[6,9] x[6,10] x[6,11] x[6,12] x[6,13] x[7,0] x[7,1]
 x[7,2] x[7,3] x[7,4] x[7,5] x[7,6] x[7,7] x[7,8] x[7,9] x[7,10] x[7,11]
 x[7,12] x[7,13] x[8,0] x[8,1] x[8,2] x[8,3] x[8,4] x[8,5] x[8,6] x[8,7]
 x[8,8] x[8,9] x[8,10] x[8,11] x[8,12] x[8,13] x[9,0] x[9,1] x[9,2] x[9,3]
 x[9,4] x[9,5] x[9,6] x[9,7] x[9,8] x[9,9] x[9,10] x[9,11] x[9,12] x[9,13]
 x[10,0] x[10,1] x[10,2] x[10,3] x[10,4] x[10,5] x[10,6] x[10,7] x[10,8]
 x[10,9] x[10,10] x[10,11] x[10,12] x[10,13] x[11,0] x[11,1] x[11,2]
 x[11,3] x[11,4] x[11,5] x[11,6] x[11,7] x[11,8] x[11,9] x[11,10] x[11,11]
 x[11,12] x[11,13] x[12,0] x[12,1] x[12,2] x[12,3] x[12,4] x[12,5] x[12,6]
 x[12,7] x[12,8] x[12,9] x[12,10] x[12,11] x[12,12] x[12,13] x[13,0]
 x[13,1] x[13,2] x[13,3] x[13,4] x[13,5] x[13,6] x[13,7] x[13,8] x[13,9]
 x[13,10] x[13,11] x[13,12] x[13,13] v[0,0] v[0,1] v[0,2] v[0,3] v[0,4]
 v[0,5] v[0,6] v[0,7] v[0,8] v[0,9] v[0,10] v[0,11] v[0,12] v[0,13] v[1,0]
 v[1,1] v[1,2] v[1,3] v[1,4] v[1,5] v[1,6] v[1,7] v[1,8] v[1,9] v[1,10]
 v[1,11] v[1,12] v[1,13] v[2,0] v[2,1] v[2,2] v[2,3] v[2,4] v[2,5] v[2,6]
 v[2,7] v[2,8] v[2,9] v[2,10] v[2,11] v[2,12] v[2,13] v[3,0] v[3,1] v[3,2]
 v[3,3] v[3,4] v[3,5] v[3,6] v[3,7] v[3,8] v[3,9] v[3,10] v[3,11] v[3,12]
 v[3,13] v[4,0] v[4,1] v[4,2] v[4,3] v[4,4] v[4,5] v[4,6] v[4,7] v[4,8]
 v[4,9] v[4,10] v[4,11] v[4,12] v[4,13] v[5,0] v[5,1] v[5,2] v[5,3] v[5,4]
 v[5,5] v[5,6] v[5,7] v[5,8] v[5,9] v[5,10] v[5,11] v[5,12] v[5,13] v[6,0]
 v[6,1] v[6,2] v[6,3] v[6,4] v[6,5] v[6,6] v[6,7] v[6,8] v[6,9] v[6,10]
 v[6,11] v[6,12] v[6,13] v[7,0] v[7,1] v[7,2] v[7,3] v[7,4] v[7,5] v[7,6]
 v[7,7] v[7,8] v[7,9] v[7,10] v[7,11] v[7,12] v[7,13] v[8,0] v[8,1] v[8,2]
 v[8,3] v[8,4] v[8,5] v[8,6] v[8,7] v[8,8] v[8,9] v[8,10] v[8,11] v[8,12]
 v[8,13] v[9,0] v[9,1] v[9,2] v[9,3] v[9,4] v[9,5] v[9,6] v[9,7] v[9,8]
 v[9,9] v[9,10] v[9,11] v[9,12] v[9,13] v[10,0] v[10,1] v[10,2] v[10,3]
 v[10,4] v[10,5] v[10,6] v[10,7] v[10,8] v[10,9] v[10,10] v[10,11] v[10,12]
 v[10,13] v[11,0] v[11,1] v[11,2] v[11,3] v[11,4] v[11,5] v[11,6] v[11,7]
 v[11,8] v[11,9] v[11,10] v[11,11] v[11,12] v[11,13] v[12,0] v[12,1]
 v[12,2] v[12,3] v[12,4] v[12,5] v[12,6] v[12,7] v[12,8] v[12,9] v[12,10]
 v[12,11] v[12,12] v[12,13] v[13,0] v[13,1] v[13,2] v[13,3] v[13,4] v[13,5]
 v[13,6] v[13,7] v[13,8] v[13,9] v[13,10] v[13,11] v[13,12] v[13,13] y[2,1]
 y[2,2] y[3,1] y[3,2] y[4,1] y[4,2] y[5,1] y[5,2] y[6,1] y[6,2]
Generals
 w[0,0] w[0,1] w[0,2] w[0,3] w[0,4] w[0,5] w[0,6] w[0,7] w[0,8] w[0,9]
 w[0,10] w[0,11] w[0,12] w[0,13] w[1,0] w[1,1] w[1,2] w[1,3] w[1,4] w[1,5]
 w[1,6] w[1,7] w[1,8] w[1,9] w[1,10] w[1,11] w[1,12] w[1,13] w[2,0] w[2,1]
 w[2,2] w[2,3] w[2,4] w[2,5] w[2,6] w[2,7] w[2,8] w[2,9] w[2,10] w[2,11]
 w[2,12] w[2,13] w[3,0] w[3,1] w[3,2] w[3,3] w[3,4] w[3,5] w[3,6] w[3,7]
 w[3,8] w[3,9] w[3,10] w[3,11] w[3,12] w[3,13] w[4,0] w[4,1] w[4,2] w[4,3]
 w[4,4] w[4,5] w[4,6] w[4,7] w[4,8] w[4,9] w[4,10] w[4,11] w[4,12] w[4,13]
 w[5,0] w[5,1] w[5,2] w[5,3] w[5,4] w[5,5] w[5,6] w[5,7] w[5,8] w[5,9]
 w[5,10] w[5,11] w[5,12] w[5,13] w[6,0] w[6,1] w[6,2] w[6,3] w[6,4] w[6,5]
 w[6,6] w[6,7] w[6,8] w[6,9] w[6,10] w[6,11] w[6,12] w[6,13] w[7,0] w[7,1]
 w[7,2] w[7,3] w[7,4] w[7,5] w[7,6] w[7,7] w[7,8] w[7,9] w[7,10] w[7,11]
 w[7,12] w[7,13] w[8,0] w[8,1] w[8,2] w[8,3] w[8,4] w[8,5] w[8,6] w[8,7]
 w[8,8] w[8,9] w[8,10] w[8,11] w[8,12] w[8,13] w[9,0] w[9,1] w[9,2] w[9,3]
 w[9,4] w[9,5] w[9,6] w[9,7] w[9,8] w[9,9] w[9,10] w[9,11] w[9,12] w[9,13]
 w[10,0] w[10,1] w[10,2] w[10,3] w[10,4] w[10,5] w[10,6] w[10,7] w[10,8]
 w[10,9] w[10,10] w[10,11] w[10,12] w[10,13] w[11,0] w[11,1] w[11,2]
 w[11,3] w[11,4] w[11,5] w[11,6] w[11,7] w[11,8] w[11,9] w[11,10] w[11,11]
 w[11,12] w[11,13] w[12,0] w[12,1] w[12,2] w[12,3] w[12,4] w[12,5] w[12,6]
 w[12,7] w[12,8] w[12,9] w[12,10] w[12,11] w[12,12] w[12,13] w[13,0]
 w[13,1] w[13,2] w[13,3] w[13,4] w[13,5] w[13,6] w[13,7] w[13,8] w[13,9]
 w[13,10] w[13,11] w[13,12] w[13,13]
End
