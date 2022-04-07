\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[13,0] + 0 x[13,1] + 0 x[13,2] + 0 x[13,3] + 0 x[13,4]
   + 0 x[13,5] + 0 x[13,6] + 0 x[13,7] + 0 x[13,8] + 0 x[13,9] + 0 x[13,10]
   + 0 x[13,11] + 0 x[13,12] + 0 x[13,13] + 0 w[0,0] + 0 w[13,0]
   + 0 w[13,1] + 0 w[13,12] + 0 w[13,13] + 0 v[0,0] + 0 v[1,0] + 0 v[2,0]
   + 0 v[3,0] + 0 v[4,0] + 0 v[5,0] + 0 v[6,0] + 0 v[7,0] + 0 v[8,0]
   + 0 v[9,0] + 0 v[10,0] + 0 v[11,0] + 0 v[12,0] + 0 v[13,0] + 0 v[13,1]
   + 0 v[13,2] + 0 v[13,3] + 0 v[13,4] + 0 v[13,5] + 0 v[13,6] + 0 v[13,7]
   + 0 v[13,8] + 0 v[13,9] + 0 v[13,10] + 0 v[13,11] + 0 v[13,12]
   + 0 v[13,13] + 0 service_time[13] + start_service[13]
Subject To
 MaxNumberVehiclesStart: x[0,1] + x[0,2] + x[0,3] + x[0,4] + x[0,5]
   + x[0,6] + x[0,7] + x[0,8] + x[0,9] + x[0,10] + x[0,11] + x[0,12]
   + x[0,13] <= 2
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
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - service_time[0]
   - start_service[0] + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 1.0000312259983328e+06 x[0,2]
   - service_time[0] - start_service[0] + start_service[2] >= -999999
 NodeArrivalVehicles[0,3]: - 1.0000272856838576e+06 x[0,3]
   - service_time[0] - start_service[0] + start_service[3] >= -999999
 NodeArrivalVehicles[0,4]: - 1.0000836457862235e+06 x[0,4]
   - service_time[0] - start_service[0] + start_service[4] >= -999999
 NodeArrivalVehicles[0,5]: - 1.0000549960468116e+06 x[0,5]
   - service_time[0] - start_service[0] + start_service[5] >= -999999
 NodeArrivalVehicles[0,6]: - 1.0000880772113261e+06 x[0,6]
   - service_time[0] - start_service[0] + start_service[6] >= -999999
 NodeArrivalVehicles[0,7]: - 1.0000312259983328e+06 x[0,7]
   - service_time[0] - start_service[0] + start_service[7] >= -999999
 NodeArrivalVehicles[0,8]: - 1.0000272856838576e+06 x[0,8]
   - service_time[0] - start_service[0] + start_service[8] >= -999999
 NodeArrivalVehicles[0,9]: - 1.0000836457862235e+06 x[0,9]
   - service_time[0] - start_service[0] + start_service[9] >= -999999
 NodeArrivalVehicles[0,10]: - 1.0000549960468116e+06 x[0,10]
   - service_time[0] - start_service[0] + start_service[10] >= -999999
 NodeArrivalVehicles[0,11]: - 1.0000880772113261e+06 x[0,11]
   - service_time[0] - start_service[0] + start_service[11] >= -999999
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] - service_time[1] >= -999999
 NodeArrivalVehicles[1,2]: - 1.0000312259983328e+06 x[1,2]
   - service_time[1] - start_service[1] + start_service[2] >= -999999
 NodeArrivalVehicles[1,3]: - 1.0000272856838576e+06 x[1,3]
   - service_time[1] - start_service[1] + start_service[3] >= -999999
 NodeArrivalVehicles[1,4]: - 1.0000836457862235e+06 x[1,4]
   - service_time[1] - start_service[1] + start_service[4] >= -999999
 NodeArrivalVehicles[1,5]: - 1.0000549960468116e+06 x[1,5]
   - service_time[1] - start_service[1] + start_service[5] >= -999999
 NodeArrivalVehicles[1,6]: - 1.0000880772113261e+06 x[1,6]
   - service_time[1] - start_service[1] + start_service[6] >= -999999
 NodeArrivalVehicles[1,7]: - 1.0000312259983328e+06 x[1,7]
   - service_time[1] - start_service[1] + start_service[7] >= -999999
 NodeArrivalVehicles[1,8]: - 1.0000272856838576e+06 x[1,8]
   - service_time[1] - start_service[1] + start_service[8] >= -999999
 NodeArrivalVehicles[1,9]: - 1.0000836457862235e+06 x[1,9]
   - service_time[1] - start_service[1] + start_service[9] >= -999999
 NodeArrivalVehicles[1,10]: - 1.0000549960468116e+06 x[1,10]
   - service_time[1] - start_service[1] + start_service[10] >= -999999
 NodeArrivalVehicles[1,11]: - 1.0000880772113261e+06 x[1,11]
   - service_time[1] - start_service[1] + start_service[11] >= -999999
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999999
 NodeArrivalVehicles[2,1]: - 1.0000312259983328e+06 x[2,1]
   - service_time[2] + start_service[1] - start_service[2] >= -999999
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] - service_time[2] >= -999999
 NodeArrivalVehicles[2,3]: - 1.0000250647332142e+06 x[2,3]
   - service_time[2] - start_service[2] + start_service[3] >= -999999
 NodeArrivalVehicles[2,4]: - 1.0000808217749445e+06 x[2,4]
   - service_time[2] - start_service[2] + start_service[4] >= -999999
 NodeArrivalVehicles[2,5]: - 1.0000268034405275e+06 x[2,5]
   - service_time[2] - start_service[2] + start_service[5] >= -999999
 NodeArrivalVehicles[2,6]: - 1.0000815147089314e+06 x[2,6]
   - service_time[2] - start_service[2] + start_service[6] >= -999999
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 1.0000250647332142e+06 x[2,8]
   - service_time[2] - start_service[2] + start_service[8] >= -999999
 NodeArrivalVehicles[2,9]: - 1.0000808217749445e+06 x[2,9]
   - service_time[2] - start_service[2] + start_service[9] >= -999999
 NodeArrivalVehicles[2,10]: - 1.0000268034405275e+06 x[2,10]
   - service_time[2] - start_service[2] + start_service[10] >= -999999
 NodeArrivalVehicles[2,11]: - 1.0000815147089314e+06 x[2,11]
   - service_time[2] - start_service[2] + start_service[11] >= -999999
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999999
 NodeArrivalVehicles[3,1]: - 1.0000272856838576e+06 x[3,1]
   - service_time[3] + start_service[1] - start_service[3] >= -999999
 NodeArrivalVehicles[3,2]: - 1.0000250647332142e+06 x[3,2]
   - service_time[3] + start_service[2] - start_service[3] >= -999999
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] - service_time[3] >= -999999
 NodeArrivalVehicles[3,4]: - 1.0000587605792192e+06 x[3,4]
   - service_time[3] - start_service[3] + start_service[4] >= -999999
 NodeArrivalVehicles[3,5]: - 1.0000339201586805e+06 x[3,5]
   - service_time[3] - start_service[3] + start_service[5] >= -999999
 NodeArrivalVehicles[3,6]: - 1.000061645695059e+06 x[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999999
 NodeArrivalVehicles[3,7]: - 1.0000250647332142e+06 x[3,7]
   - service_time[3] - start_service[3] + start_service[7] >= -999999
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 1.0000587605792192e+06 x[3,9]
   - service_time[3] - start_service[3] + start_service[9] >= -999999
 NodeArrivalVehicles[3,10]: - 1.0000339201586805e+06 x[3,10]
   - service_time[3] - start_service[3] + start_service[10] >= -999999
 NodeArrivalVehicles[3,11]: - 1.000061645695059e+06 x[3,11]
   - service_time[3] - start_service[3] + start_service[11] >= -999999
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999999
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 NodeArrivalVehicles[4,1]: - 1.0000836457862235e+06 x[4,1]
   - service_time[4] + start_service[1] - start_service[4] >= -999999
 NodeArrivalVehicles[4,2]: - 1.0000808217749445e+06 x[4,2]
   - service_time[4] + start_service[2] - start_service[4] >= -999999
 NodeArrivalVehicles[4,3]: - 1.0000587605792192e+06 x[4,3]
   - service_time[4] + start_service[3] - start_service[4] >= -999999
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] - service_time[4] >= -999999
 NodeArrivalVehicles[4,5]: - 1.000068152179442e+06 x[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999999
 NodeArrivalVehicles[4,6]: - 1.0000092433056755e+06 x[4,6]
   - service_time[4] - start_service[4] + start_service[6] >= -999999
 NodeArrivalVehicles[4,7]: - 1.0000808217749445e+06 x[4,7]
   - service_time[4] - start_service[4] + start_service[7] >= -999999
 NodeArrivalVehicles[4,8]: - 1.0000587605792192e+06 x[4,8]
   - service_time[4] - start_service[4] + start_service[8] >= -999999
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 1.000068152179442e+06 x[4,10]
   - service_time[4] - start_service[4] + start_service[10] >= -999999
 NodeArrivalVehicles[4,11]: - 1.0000092433056755e+06 x[4,11]
   - service_time[4] - start_service[4] + start_service[11] >= -999999
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999999
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999999
 NodeArrivalVehicles[5,1]: - 1.0000549960468116e+06 x[5,1]
   - service_time[5] + start_service[1] - start_service[5] >= -999999
 NodeArrivalVehicles[5,2]: - 1.0000268034405275e+06 x[5,2]
   - service_time[5] + start_service[2] - start_service[5] >= -999999
 NodeArrivalVehicles[5,3]: - 1.0000339201586805e+06 x[5,3]
   - service_time[5] + start_service[3] - start_service[5] >= -999999
 NodeArrivalVehicles[5,4]: - 1.000068152179442e+06 x[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999999
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] - service_time[5] >= -999999
 NodeArrivalVehicles[5,6]: - 1.0000655908555345e+06 x[5,6]
   - service_time[5] - start_service[5] + start_service[6] >= -999999
 NodeArrivalVehicles[5,7]: - 1.0000268034405275e+06 x[5,7]
   - service_time[5] - start_service[5] + start_service[7] >= -999999
 NodeArrivalVehicles[5,8]: - 1.0000339201586805e+06 x[5,8]
   - service_time[5] - start_service[5] + start_service[8] >= -999999
 NodeArrivalVehicles[5,9]: - 1.000068152179442e+06 x[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999999
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 1.0000655908555345e+06 x[5,11]
   - service_time[5] - start_service[5] + start_service[11] >= -999999
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999999
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999999
 NodeArrivalVehicles[6,1]: - 1.0000880772113261e+06 x[6,1]
   - service_time[6] + start_service[1] - start_service[6] >= -999999
 NodeArrivalVehicles[6,2]: - 1.0000815147089314e+06 x[6,2]
   - service_time[6] + start_service[2] - start_service[6] >= -999999
 NodeArrivalVehicles[6,3]: - 1.000061645695059e+06 x[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999999
 NodeArrivalVehicles[6,4]: - 1.0000092433056755e+06 x[6,4]
   - service_time[6] + start_service[4] - start_service[6] >= -999999
 NodeArrivalVehicles[6,5]: - 1.0000655908555345e+06 x[6,5]
   - service_time[6] + start_service[5] - start_service[6] >= -999999
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] - service_time[6] >= -999999
 NodeArrivalVehicles[6,7]: - 1.0000815147089314e+06 x[6,7]
   - service_time[6] - start_service[6] + start_service[7] >= -999999
 NodeArrivalVehicles[6,8]: - 1.000061645695059e+06 x[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999999
 NodeArrivalVehicles[6,9]: - 1.0000092433056755e+06 x[6,9]
   - service_time[6] - start_service[6] + start_service[9] >= -999999
 NodeArrivalVehicles[6,10]: - 1.0000655908555345e+06 x[6,10]
   - service_time[6] - start_service[6] + start_service[10] >= -999999
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999999
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999999
 NodeArrivalVehicles[7,1]: - 1.0000312259983328e+06 x[7,1]
   - service_time[7] + start_service[1] - start_service[7] >= -999999
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 1.0000250647332142e+06 x[7,3]
   - service_time[7] + start_service[3] - start_service[7] >= -999999
 NodeArrivalVehicles[7,4]: - 1.0000808217749445e+06 x[7,4]
   - service_time[7] + start_service[4] - start_service[7] >= -999999
 NodeArrivalVehicles[7,5]: - 1.0000268034405275e+06 x[7,5]
   - service_time[7] + start_service[5] - start_service[7] >= -999999
 NodeArrivalVehicles[7,6]: - 1.0000815147089314e+06 x[7,6]
   - service_time[7] + start_service[6] - start_service[7] >= -999999
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] - service_time[7] >= -999999
 NodeArrivalVehicles[7,8]: - 1.0000250647332142e+06 x[7,8]
   - service_time[7] - start_service[7] + start_service[8] >= -999999
 NodeArrivalVehicles[7,9]: - 1.0000808217749445e+06 x[7,9]
   - service_time[7] - start_service[7] + start_service[9] >= -999999
 NodeArrivalVehicles[7,10]: - 1.0000268034405275e+06 x[7,10]
   - service_time[7] - start_service[7] + start_service[10] >= -999999
 NodeArrivalVehicles[7,11]: - 1.0000815147089314e+06 x[7,11]
   - service_time[7] - start_service[7] + start_service[11] >= -999999
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999999
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999999
 NodeArrivalVehicles[8,1]: - 1.0000272856838576e+06 x[8,1]
   - service_time[8] + start_service[1] - start_service[8] >= -999999
 NodeArrivalVehicles[8,2]: - 1.0000250647332142e+06 x[8,2]
   - service_time[8] + start_service[2] - start_service[8] >= -999999
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 1.0000587605792192e+06 x[8,4]
   - service_time[8] + start_service[4] - start_service[8] >= -999999
 NodeArrivalVehicles[8,5]: - 1.0000339201586805e+06 x[8,5]
   - service_time[8] + start_service[5] - start_service[8] >= -999999
 NodeArrivalVehicles[8,6]: - 1.000061645695059e+06 x[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999999
 NodeArrivalVehicles[8,7]: - 1.0000250647332142e+06 x[8,7]
   - service_time[8] + start_service[7] - start_service[8] >= -999999
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] - service_time[8] >= -999999
 NodeArrivalVehicles[8,9]: - 1.0000587605792192e+06 x[8,9]
   - service_time[8] - start_service[8] + start_service[9] >= -999999
 NodeArrivalVehicles[8,10]: - 1.0000339201586805e+06 x[8,10]
   - service_time[8] - start_service[8] + start_service[10] >= -999999
 NodeArrivalVehicles[8,11]: - 1.000061645695059e+06 x[8,11]
   - service_time[8] - start_service[8] + start_service[11] >= -999999
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999999
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999999
 NodeArrivalVehicles[9,1]: - 1.0000836457862235e+06 x[9,1]
   - service_time[9] + start_service[1] - start_service[9] >= -999999
 NodeArrivalVehicles[9,2]: - 1.0000808217749445e+06 x[9,2]
   - service_time[9] + start_service[2] - start_service[9] >= -999999
 NodeArrivalVehicles[9,3]: - 1.0000587605792192e+06 x[9,3]
   - service_time[9] + start_service[3] - start_service[9] >= -999999
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 1.000068152179442e+06 x[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999999
 NodeArrivalVehicles[9,6]: - 1.0000092433056755e+06 x[9,6]
   - service_time[9] + start_service[6] - start_service[9] >= -999999
 NodeArrivalVehicles[9,7]: - 1.0000808217749445e+06 x[9,7]
   - service_time[9] + start_service[7] - start_service[9] >= -999999
 NodeArrivalVehicles[9,8]: - 1.0000587605792192e+06 x[9,8]
   - service_time[9] + start_service[8] - start_service[9] >= -999999
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] - service_time[9] >= -999999
 NodeArrivalVehicles[9,10]: - 1.000068152179442e+06 x[9,10]
   - service_time[9] - start_service[9] + start_service[10] >= -999999
 NodeArrivalVehicles[9,11]: - 1.0000092433056755e+06 x[9,11]
   - service_time[9] - start_service[9] + start_service[11] >= -999999
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999999
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999999
 NodeArrivalVehicles[10,1]: - 1.0000549960468116e+06 x[10,1]
   - service_time[10] + start_service[1] - start_service[10] >= -999999
 NodeArrivalVehicles[10,2]: - 1.0000268034405275e+06 x[10,2]
   - service_time[10] + start_service[2] - start_service[10] >= -999999
 NodeArrivalVehicles[10,3]: - 1.0000339201586805e+06 x[10,3]
   - service_time[10] + start_service[3] - start_service[10] >= -999999
 NodeArrivalVehicles[10,4]: - 1.000068152179442e+06 x[10,4]
   - service_time[10] + start_service[4] - start_service[10] >= -999999
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 1.0000655908555345e+06 x[10,6]
   - service_time[10] + start_service[6] - start_service[10] >= -999999
 NodeArrivalVehicles[10,7]: - 1.0000268034405275e+06 x[10,7]
   - service_time[10] + start_service[7] - start_service[10] >= -999999
 NodeArrivalVehicles[10,8]: - 1.0000339201586805e+06 x[10,8]
   - service_time[10] + start_service[8] - start_service[10] >= -999999
 NodeArrivalVehicles[10,9]: - 1.000068152179442e+06 x[10,9]
   - service_time[10] + start_service[9] - start_service[10] >= -999999
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] - service_time[10]
   >= -999999
 NodeArrivalVehicles[10,11]: - 1.0000655908555345e+06 x[10,11]
   - service_time[10] - start_service[10] + start_service[11] >= -999999
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999999
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999999
 NodeArrivalVehicles[11,1]: - 1.0000880772113261e+06 x[11,1]
   - service_time[11] + start_service[1] - start_service[11] >= -999999
 NodeArrivalVehicles[11,2]: - 1.0000815147089314e+06 x[11,2]
   - service_time[11] + start_service[2] - start_service[11] >= -999999
 NodeArrivalVehicles[11,3]: - 1.000061645695059e+06 x[11,3]
   - service_time[11] + start_service[3] - start_service[11] >= -999999
 NodeArrivalVehicles[11,4]: - 1.0000092433056755e+06 x[11,4]
   - service_time[11] + start_service[4] - start_service[11] >= -999999
 NodeArrivalVehicles[11,5]: - 1.0000655908555345e+06 x[11,5]
   - service_time[11] + start_service[5] - start_service[11] >= -999999
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 1.0000815147089314e+06 x[11,7]
   - service_time[11] + start_service[7] - start_service[11] >= -999999
 NodeArrivalVehicles[11,8]: - 1.000061645695059e+06 x[11,8]
   - service_time[11] + start_service[8] - start_service[11] >= -999999
 NodeArrivalVehicles[11,9]: - 1.0000092433056755e+06 x[11,9]
   - service_time[11] + start_service[9] - start_service[11] >= -999999
 NodeArrivalVehicles[11,10]: - 1.0000655908555345e+06 x[11,10]
   - service_time[11] + start_service[10] - start_service[11] >= -999999
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] - service_time[11]
   >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999999
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999999
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 1.0000312259983328e+06 x[12,2]
   - service_time[12] + start_service[2] - start_service[12] >= -999999
 NodeArrivalVehicles[12,3]: - 1.0000272856838576e+06 x[12,3]
   - service_time[12] + start_service[3] - start_service[12] >= -999999
 NodeArrivalVehicles[12,4]: - 1.0000836457862235e+06 x[12,4]
   - service_time[12] + start_service[4] - start_service[12] >= -999999
 NodeArrivalVehicles[12,5]: - 1.0000549960468116e+06 x[12,5]
   - service_time[12] + start_service[5] - start_service[12] >= -999999
 NodeArrivalVehicles[12,6]: - 1.0000880772113261e+06 x[12,6]
   - service_time[12] + start_service[6] - start_service[12] >= -999999
 NodeArrivalVehicles[12,7]: - 1.0000312259983328e+06 x[12,7]
   - service_time[12] + start_service[7] - start_service[12] >= -999999
 NodeArrivalVehicles[12,8]: - 1.0000272856838576e+06 x[12,8]
   - service_time[12] + start_service[8] - start_service[12] >= -999999
 NodeArrivalVehicles[12,9]: - 1.0000836457862235e+06 x[12,9]
   - service_time[12] + start_service[9] - start_service[12] >= -999999
 NodeArrivalVehicles[12,10]: - 1.0000549960468116e+06 x[12,10]
   - service_time[12] + start_service[10] - start_service[12] >= -999999
 NodeArrivalVehicles[12,11]: - 1.0000880772113261e+06 x[12,11]
   - service_time[12] + start_service[11] - start_service[12] >= -999999
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] - service_time[12]
   >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999999
 R194: w[0,5] = 2
 R195: w[0,2] = 2
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] <= 4
 TotalRobotStarts: w[0,2] + w[0,3] + w[0,4] + w[0,5] + w[0,6] + w[0,7]
   + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12] + w[0,13] + w[1,2]
   + w[1,3] + w[1,4] + w[1,5] + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10]
   + w[1,11] + w[1,12] + w[1,13] <= 5
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] <= 1
 RobotsFinishAtVehicleDepot: w[0,13] + w[1,13] + w[2,13] + w[3,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] <= 4
 TotalRobotFinishes: w[0,12] + w[0,13] + w[1,12] + w[1,13] + w[2,12]
   + w[2,13] + w[3,12] + w[3,13] + w[4,12] + w[4,13] + w[5,12] + w[5,13]
   + w[6,12] + w[6,13] + w[7,12] + w[7,13] + w[8,12] + w[8,13] + w[9,12]
   + w[9,13] + w[10,12] + w[10,13] + w[11,12] + w[11,13] <= 5
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
 StartServiceRobots[0,1]: - 999999 v[0,1] - service_time[0]
   - start_service[0] + start_service[1] >= -999999
 StartServiceRobots[0,2]: - 1.0000312259983328e+06 v[0,2] - service_time[0]
   - start_service[0] + start_service[2] >= -999999
 StartServiceRobots[0,3]: - 1.0000272856838576e+06 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999999
 StartServiceRobots[0,4]: - 1.0000836457862235e+06 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999999
 StartServiceRobots[0,5]: - 1.0000549960468116e+06 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999999
 StartServiceRobots[0,6]: - 1.0000880772113261e+06 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999999
 StartServiceRobots[0,7]: - 1.0000312259983328e+06 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999999
 StartServiceRobots[0,8]: - 1.0000272856838576e+06 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999999
 StartServiceRobots[0,9]: - 1.0000836457862235e+06 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999999
 StartServiceRobots[0,10]: - 1.0000549960468116e+06 v[0,10]
   - service_time[0] - start_service[0] + start_service[10] >= -999999
 StartServiceRobots[0,11]: - 1.0000880772113261e+06 v[0,11]
   - service_time[0] - start_service[0] + start_service[11] >= -999999
 StartServiceRobots[0,12]: - 999999 v[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999999
 StartServiceRobots[0,13]: - 999999 v[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 1.0000312259983328e+06 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999999
 StartServiceRobots[1,3]: - 1.0000272856838576e+06 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999999
 StartServiceRobots[1,4]: - 1.0000836457862235e+06 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999999
 StartServiceRobots[1,5]: - 1.0000549960468116e+06 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999999
 StartServiceRobots[1,6]: - 1.0000880772113261e+06 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999999
 StartServiceRobots[1,7]: - 1.0000312259983328e+06 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999999
 StartServiceRobots[1,8]: - 1.0000272856838576e+06 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999999
 StartServiceRobots[1,9]: - 1.0000836457862235e+06 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999999
 StartServiceRobots[1,10]: - 1.0000549960468116e+06 v[1,10]
   - service_time[1] - start_service[1] + start_service[10] >= -999999
 StartServiceRobots[1,11]: - 1.0000880772113261e+06 v[1,11]
   - service_time[1] - start_service[1] + start_service[11] >= -999999
 StartServiceRobots[1,12]: - 999999 v[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999999
 StartServiceRobots[1,13]: - 999999 v[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999999
 StartServiceRobots[2,1]: - 1.0000312259983328e+06 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999999
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 1.0000250647332142e+06 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999999
 StartServiceRobots[2,4]: - 1.0000808217749445e+06 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999999
 StartServiceRobots[2,5]: - 1.0000268034405275e+06 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999999
 StartServiceRobots[2,6]: - 1.0000815147089314e+06 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999999
 StartServiceRobots[2,7]: - 999999 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999999
 StartServiceRobots[2,8]: - 1.0000250647332142e+06 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999999
 StartServiceRobots[2,9]: - 1.0000808217749445e+06 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999999
 StartServiceRobots[2,10]: - 1.0000268034405275e+06 v[2,10]
   - service_time[2] - start_service[2] + start_service[10] >= -999999
 StartServiceRobots[2,11]: - 1.0000815147089314e+06 v[2,11]
   - service_time[2] - start_service[2] + start_service[11] >= -999999
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999999
 StartServiceRobots[2,13]: - 999999 v[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999999
 StartServiceRobots[3,1]: - 1.0000272856838576e+06 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999999
 StartServiceRobots[3,2]: - 1.0000250647332142e+06 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999999
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 1.0000587605792192e+06 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999999
 StartServiceRobots[3,5]: - 1.0000339201586805e+06 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999999
 StartServiceRobots[3,6]: - 1.000061645695059e+06 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999999
 StartServiceRobots[3,7]: - 1.0000250647332142e+06 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999999
 StartServiceRobots[3,8]: - 999999 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999999
 StartServiceRobots[3,9]: - 1.0000587605792192e+06 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999999
 StartServiceRobots[3,10]: - 1.0000339201586805e+06 v[3,10]
   - service_time[3] - start_service[3] + start_service[10] >= -999999
 StartServiceRobots[3,11]: - 1.000061645695059e+06 v[3,11]
   - service_time[3] - start_service[3] + start_service[11] >= -999999
 StartServiceRobots[3,12]: - 999999 v[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999999
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999999
 StartServiceRobots[4,1]: - 1.0000836457862235e+06 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999999
 StartServiceRobots[4,2]: - 1.0000808217749445e+06 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999999
 StartServiceRobots[4,3]: - 1.0000587605792192e+06 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999999
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 1.000068152179442e+06 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999999
 StartServiceRobots[4,6]: - 1.0000092433056755e+06 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999999
 StartServiceRobots[4,7]: - 1.0000808217749445e+06 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999999
 StartServiceRobots[4,8]: - 1.0000587605792192e+06 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999999
 StartServiceRobots[4,9]: - 999999 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999999
 StartServiceRobots[4,10]: - 1.000068152179442e+06 v[4,10]
   - service_time[4] - start_service[4] + start_service[10] >= -999999
 StartServiceRobots[4,11]: - 1.0000092433056755e+06 v[4,11]
   - service_time[4] - start_service[4] + start_service[11] >= -999999
 StartServiceRobots[4,12]: - 999999 v[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999999
 StartServiceRobots[4,13]: - 999999 v[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999999
 StartServiceRobots[5,1]: - 1.0000549960468116e+06 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999999
 StartServiceRobots[5,2]: - 1.0000268034405275e+06 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999999
 StartServiceRobots[5,3]: - 1.0000339201586805e+06 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999999
 StartServiceRobots[5,4]: - 1.000068152179442e+06 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999999
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 1.0000655908555345e+06 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999999
 StartServiceRobots[5,7]: - 1.0000268034405275e+06 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999999
 StartServiceRobots[5,8]: - 1.0000339201586805e+06 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999999
 StartServiceRobots[5,9]: - 1.000068152179442e+06 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999999
 StartServiceRobots[5,10]: - 999999 v[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999999
 StartServiceRobots[5,11]: - 1.0000655908555345e+06 v[5,11]
   - service_time[5] - start_service[5] + start_service[11] >= -999999
 StartServiceRobots[5,12]: - 999999 v[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999999
 StartServiceRobots[5,13]: - 999999 v[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999999
 StartServiceRobots[6,1]: - 1.0000880772113261e+06 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999999
 StartServiceRobots[6,2]: - 1.0000815147089314e+06 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999999
 StartServiceRobots[6,3]: - 1.000061645695059e+06 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999999
 StartServiceRobots[6,4]: - 1.0000092433056755e+06 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999999
 StartServiceRobots[6,5]: - 1.0000655908555345e+06 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999999
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 1.0000815147089314e+06 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999999
 StartServiceRobots[6,8]: - 1.000061645695059e+06 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999999
 StartServiceRobots[6,9]: - 1.0000092433056755e+06 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999999
 StartServiceRobots[6,10]: - 1.0000655908555345e+06 v[6,10]
   - service_time[6] - start_service[6] + start_service[10] >= -999999
 StartServiceRobots[6,11]: - 999999 v[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999999
 StartServiceRobots[6,12]: - 999999 v[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999999
 StartServiceRobots[6,13]: - 999999 v[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999999
 StartServiceRobots[7,1]: - 1.0000312259983328e+06 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999999
 StartServiceRobots[7,2]: - 999999 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999999
 StartServiceRobots[7,3]: - 1.0000250647332142e+06 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999999
 StartServiceRobots[7,4]: - 1.0000808217749445e+06 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999999
 StartServiceRobots[7,5]: - 1.0000268034405275e+06 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999999
 StartServiceRobots[7,6]: - 1.0000815147089314e+06 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999999
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 1.0000250647332142e+06 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999999
 StartServiceRobots[7,9]: - 1.0000808217749445e+06 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999999
 StartServiceRobots[7,10]: - 1.0000268034405275e+06 v[7,10]
   - service_time[7] - start_service[7] + start_service[10] >= -999999
 StartServiceRobots[7,11]: - 1.0000815147089314e+06 v[7,11]
   - service_time[7] - start_service[7] + start_service[11] >= -999999
 StartServiceRobots[7,12]: - 999999 v[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999999
 StartServiceRobots[7,13]: - 999999 v[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999999
 StartServiceRobots[8,1]: - 1.0000272856838576e+06 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999999
 StartServiceRobots[8,2]: - 1.0000250647332142e+06 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999999
 StartServiceRobots[8,3]: - 999999 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999999
 StartServiceRobots[8,4]: - 1.0000587605792192e+06 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999999
 StartServiceRobots[8,5]: - 1.0000339201586805e+06 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999999
 StartServiceRobots[8,6]: - 1.000061645695059e+06 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999999
 StartServiceRobots[8,7]: - 1.0000250647332142e+06 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999999
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 1.0000587605792192e+06 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999999
 StartServiceRobots[8,10]: - 1.0000339201586805e+06 v[8,10]
   - service_time[8] - start_service[8] + start_service[10] >= -999999
 StartServiceRobots[8,11]: - 1.000061645695059e+06 v[8,11]
   - service_time[8] - start_service[8] + start_service[11] >= -999999
 StartServiceRobots[8,12]: - 999999 v[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999999
 StartServiceRobots[8,13]: - 999999 v[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999999
 StartServiceRobots[9,1]: - 1.0000836457862235e+06 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999999
 StartServiceRobots[9,2]: - 1.0000808217749445e+06 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999999
 StartServiceRobots[9,3]: - 1.0000587605792192e+06 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999999
 StartServiceRobots[9,4]: - 999999 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999999
 StartServiceRobots[9,5]: - 1.000068152179442e+06 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999999
 StartServiceRobots[9,6]: - 1.0000092433056755e+06 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999999
 StartServiceRobots[9,7]: - 1.0000808217749445e+06 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999999
 StartServiceRobots[9,8]: - 1.0000587605792192e+06 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999999
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 1.000068152179442e+06 v[9,10]
   - service_time[9] - start_service[9] + start_service[10] >= -999999
 StartServiceRobots[9,11]: - 1.0000092433056755e+06 v[9,11]
   - service_time[9] - start_service[9] + start_service[11] >= -999999
 StartServiceRobots[9,12]: - 999999 v[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999999
 StartServiceRobots[9,13]: - 999999 v[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999999
 StartServiceRobots[10,1]: - 1.0000549960468116e+06 v[10,1]
   - service_time[10] + start_service[1] - start_service[10] >= -999999
 StartServiceRobots[10,2]: - 1.0000268034405275e+06 v[10,2]
   - service_time[10] + start_service[2] - start_service[10] >= -999999
 StartServiceRobots[10,3]: - 1.0000339201586805e+06 v[10,3]
   - service_time[10] + start_service[3] - start_service[10] >= -999999
 StartServiceRobots[10,4]: - 1.000068152179442e+06 v[10,4]
   - service_time[10] + start_service[4] - start_service[10] >= -999999
 StartServiceRobots[10,5]: - 999999 v[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999999
 StartServiceRobots[10,6]: - 1.0000655908555345e+06 v[10,6]
   - service_time[10] + start_service[6] - start_service[10] >= -999999
 StartServiceRobots[10,7]: - 1.0000268034405275e+06 v[10,7]
   - service_time[10] + start_service[7] - start_service[10] >= -999999
 StartServiceRobots[10,8]: - 1.0000339201586805e+06 v[10,8]
   - service_time[10] + start_service[8] - start_service[10] >= -999999
 StartServiceRobots[10,9]: - 1.000068152179442e+06 v[10,9]
   - service_time[10] + start_service[9] - start_service[10] >= -999999
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 1.0000655908555345e+06 v[10,11]
   - service_time[10] - start_service[10] + start_service[11] >= -999999
 StartServiceRobots[10,12]: - 999999 v[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999999
 StartServiceRobots[10,13]: - 999999 v[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999999
 StartServiceRobots[11,1]: - 1.0000880772113261e+06 v[11,1]
   - service_time[11] + start_service[1] - start_service[11] >= -999999
 StartServiceRobots[11,2]: - 1.0000815147089314e+06 v[11,2]
   - service_time[11] + start_service[2] - start_service[11] >= -999999
 StartServiceRobots[11,3]: - 1.000061645695059e+06 v[11,3]
   - service_time[11] + start_service[3] - start_service[11] >= -999999
 StartServiceRobots[11,4]: - 1.0000092433056755e+06 v[11,4]
   - service_time[11] + start_service[4] - start_service[11] >= -999999
 StartServiceRobots[11,5]: - 1.0000655908555345e+06 v[11,5]
   - service_time[11] + start_service[5] - start_service[11] >= -999999
 StartServiceRobots[11,6]: - 999999 v[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999999
 StartServiceRobots[11,7]: - 1.0000815147089314e+06 v[11,7]
   - service_time[11] + start_service[7] - start_service[11] >= -999999
 StartServiceRobots[11,8]: - 1.000061645695059e+06 v[11,8]
   - service_time[11] + start_service[8] - start_service[11] >= -999999
 StartServiceRobots[11,9]: - 1.0000092433056755e+06 v[11,9]
   - service_time[11] + start_service[9] - start_service[11] >= -999999
 StartServiceRobots[11,10]: - 1.0000655908555345e+06 v[11,10]
   - service_time[11] + start_service[10] - start_service[11] >= -999999
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 999999 v[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999999
 StartServiceRobots[11,13]: - 999999 v[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999999
 StartServiceRobots[12,1]: - 999999 v[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999999
 StartServiceRobots[12,2]: - 1.0000312259983328e+06 v[12,2]
   - service_time[12] + start_service[2] - start_service[12] >= -999999
 StartServiceRobots[12,3]: - 1.0000272856838576e+06 v[12,3]
   - service_time[12] + start_service[3] - start_service[12] >= -999999
 StartServiceRobots[12,4]: - 1.0000836457862235e+06 v[12,4]
   - service_time[12] + start_service[4] - start_service[12] >= -999999
 StartServiceRobots[12,5]: - 1.0000549960468116e+06 v[12,5]
   - service_time[12] + start_service[5] - start_service[12] >= -999999
 StartServiceRobots[12,6]: - 1.0000880772113261e+06 v[12,6]
   - service_time[12] + start_service[6] - start_service[12] >= -999999
 StartServiceRobots[12,7]: - 1.0000312259983328e+06 v[12,7]
   - service_time[12] + start_service[7] - start_service[12] >= -999999
 StartServiceRobots[12,8]: - 1.0000272856838576e+06 v[12,8]
   - service_time[12] + start_service[8] - start_service[12] >= -999999
 StartServiceRobots[12,9]: - 1.0000836457862235e+06 v[12,9]
   - service_time[12] + start_service[9] - start_service[12] >= -999999
 StartServiceRobots[12,10]: - 1.0000549960468116e+06 v[12,10]
   - service_time[12] + start_service[10] - start_service[12] >= -999999
 StartServiceRobots[12,11]: - 1.0000880772113261e+06 v[12,11]
   - service_time[12] + start_service[11] - start_service[12] >= -999999
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 999999 v[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999999
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
 SettingServiceTime[2]: - 200 y[2,1] - 100 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 440 y[3,1] - 220 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 280 y[4,1] - 140 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 410 y[5,1] - 205 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 300 y[6,1] - 150 y[6,2] + service_time[6] = 0
 AverageTimeRobots: - 6.445199666556815 w[0,2] - 5.657136771521762 w[0,3]
   - 16.92915724470268 w[0,4] - 11.19920936231749 w[0,5]
   - 17.81544226521789 w[0,6] - 6.445199666556815 w[0,7]
   - 5.657136771521762 w[0,8] - 16.92915724470268 w[0,9]
   - 11.19920936231749 w[0,10] - 17.81544226521789 w[0,11]
   - 6.445199666556815 w[1,2] - 5.657136771521762 w[1,3]
   - 16.92915724470268 w[1,4] - 11.19920936231749 w[1,5]
   - 17.81544226521789 w[1,6] - 6.445199666556815 w[1,7]
   - 5.657136771521762 w[1,8] - 16.92915724470268 w[1,9]
   - 11.19920936231749 w[1,10] - 17.81544226521789 w[1,11]
   - 6.445199666556815 w[2,1] - 5.212946642846055 w[2,3]
   - 16.36435498891883 w[2,4] - 5.560688105515116 w[2,5]
   - 16.50294178627976 w[2,6] - 5.212946642846055 w[2,8]
   - 16.36435498891883 w[2,9] - 5.560688105515116 w[2,10]
   - 16.50294178627976 w[2,11] - 5.657136771521762 w[3,1]
   - 5.212946642846055 w[3,2] - 11.95211584383644 w[3,4]
   - 6.984031736102465 w[3,5] - 12.52913901180067 w[3,6]
   - 5.212946642846055 w[3,7] - 11.95211584383644 w[3,9]
   - 6.984031736102465 w[3,10] - 12.52913901180067 w[3,11]
   - 16.92915724470268 w[4,1] - 16.36435498891883 w[4,2]
   - 11.95211584383644 w[4,3] - 13.83043588838924 w[4,5]
   - 2.048661135098436 w[4,6] - 16.36435498891883 w[4,7]
   - 11.95211584383644 w[4,8] - 13.83043588838924 w[4,10]
   - 2.048661135098436 w[4,11] - 11.19920936231749 w[5,1]
   - 5.560688105515116 w[5,2] - 6.984031736102465 w[5,3]
   - 13.83043588838924 w[5,4] - 13.31817110689782 w[5,6]
   - 5.560688105515116 w[5,7] - 6.984031736102465 w[5,8]
   - 13.83043588838924 w[5,9] - 13.31817110689782 w[5,11]
   - 17.81544226521789 w[6,1] - 16.50294178627976 w[6,2]
   - 12.52913901180067 w[6,3] - 2.048661135098436 w[6,4]
   - 13.31817110689782 w[6,5] - 16.50294178627976 w[6,7]
   - 12.52913901180067 w[6,8] - 2.048661135098436 w[6,9]
   - 13.31817110689782 w[6,10] - 6.445199666556815 w[7,1]
   - 5.212946642846055 w[7,3] - 16.36435498891883 w[7,4]
   - 5.560688105515116 w[7,5] - 16.50294178627976 w[7,6]
   - 5.212946642846055 w[7,8] - 16.36435498891883 w[7,9]
   - 5.560688105515116 w[7,10] - 16.50294178627976 w[7,11]
   - 5.657136771521762 w[8,1] - 5.212946642846055 w[8,2]
   - 11.95211584383644 w[8,4] - 6.984031736102465 w[8,5]
   - 12.52913901180067 w[8,6] - 5.212946642846055 w[8,7]
   - 11.95211584383644 w[8,9] - 6.984031736102465 w[8,10]
   - 12.52913901180067 w[8,11] - 16.92915724470268 w[9,1]
   - 16.36435498891883 w[9,2] - 11.95211584383644 w[9,3]
   - 13.83043588838924 w[9,5] - 2.048661135098436 w[9,6]
   - 16.36435498891883 w[9,7] - 11.95211584383644 w[9,8]
   - 13.83043588838924 w[9,10] - 2.048661135098436 w[9,11]
   - 11.19920936231749 w[10,1] - 5.560688105515116 w[10,2]
   - 6.984031736102465 w[10,3] - 13.83043588838924 w[10,4]
   - 13.31817110689782 w[10,6] - 5.560688105515116 w[10,7]
   - 6.984031736102465 w[10,8] - 13.83043588838924 w[10,9]
   - 13.31817110689782 w[10,11] - 17.81544226521789 w[11,1]
   - 16.50294178627976 w[11,2] - 12.52913901180067 w[11,3]
   - 2.048661135098436 w[11,4] - 13.31817110689782 w[11,5]
   - 16.50294178627976 w[11,7] - 12.52913901180067 w[11,8]
   - 2.048661135098436 w[11,9] - 13.31817110689782 w[11,10]
   - 6.445199666556815 w[12,2] - 5.657136771521762 w[12,3]
   - 16.92915724470268 w[12,4] - 11.19920936231749 w[12,5]
   - 17.81544226521789 w[12,6] - 6.445199666556815 w[12,7]
   - 5.657136771521762 w[12,8] - 16.92915724470268 w[12,9]
   - 11.19920936231749 w[12,10] - 17.81544226521789 w[12,11] - 40 y[2,1]
   - 80 y[2,2] - 88 y[3,1] - 176 y[3,2] - 56 y[4,1] - 112 y[4,2]
   - 82 y[5,1] - 164 y[5,2] - 60 y[6,1] - 120 y[6,2] + start_service[13]
   >= 0
 AverageTimeVehices: - 16.11299916639204 x[0,2] - 14.1428419288044 x[0,3]
   - 42.32289311175671 x[0,4] - 27.99802340579372 x[0,5]
   - 44.53860566304473 x[0,6] - 16.11299916639204 x[0,7]
   - 14.1428419288044 x[0,8] - 42.32289311175671 x[0,9]
   - 27.99802340579372 x[0,10] - 44.53860566304473 x[0,11]
   - 16.11299916639204 x[1,2] - 14.1428419288044 x[1,3]
   - 42.32289311175671 x[1,4] - 27.99802340579372 x[1,5]
   - 44.53860566304473 x[1,6] - 16.11299916639204 x[1,7]
   - 14.1428419288044 x[1,8] - 42.32289311175671 x[1,9]
   - 27.99802340579372 x[1,10] - 44.53860566304473 x[1,11]
   - 16.11299916639204 x[2,1] - 13.03236660711514 x[2,3]
   - 40.91088747229708 x[2,4] - 13.90172026378779 x[2,5]
   - 41.25735446569939 x[2,6] - 13.03236660711514 x[2,8]
   - 40.91088747229708 x[2,9] - 13.90172026378779 x[2,10]
   - 41.25735446569939 x[2,11] - 14.1428419288044 x[3,1]
   - 13.03236660711514 x[3,2] - 29.8802896095911 x[3,4]
   - 17.46007934025616 x[3,5] - 31.32284752950167 x[3,6]
   - 13.03236660711514 x[3,7] - 29.8802896095911 x[3,9]
   - 17.46007934025616 x[3,10] - 31.32284752950167 x[3,11]
   - 42.32289311175671 x[4,1] - 40.91088747229708 x[4,2]
   - 29.8802896095911 x[4,3] - 34.57608972097309 x[4,5]
   - 5.12165283774609 x[4,6] - 40.91088747229708 x[4,7]
   - 29.8802896095911 x[4,8] - 34.57608972097309 x[4,10]
   - 5.12165283774609 x[4,11] - 27.99802340579372 x[5,1]
   - 13.90172026378779 x[5,2] - 17.46007934025616 x[5,3]
   - 34.57608972097309 x[5,4] - 33.29542776724454 x[5,6]
   - 13.90172026378779 x[5,7] - 17.46007934025616 x[5,8]
   - 34.57608972097309 x[5,9] - 33.29542776724454 x[5,11]
   - 44.53860566304473 x[6,1] - 41.25735446569939 x[6,2]
   - 31.32284752950167 x[6,3] - 5.12165283774609 x[6,4]
   - 33.29542776724454 x[6,5] - 41.25735446569939 x[6,7]
   - 31.32284752950167 x[6,8] - 5.12165283774609 x[6,9]
   - 33.29542776724454 x[6,10] - 16.11299916639204 x[7,1]
   - 13.03236660711514 x[7,3] - 40.91088747229708 x[7,4]
   - 13.90172026378779 x[7,5] - 41.25735446569939 x[7,6]
   - 13.03236660711514 x[7,8] - 40.91088747229708 x[7,9]
   - 13.90172026378779 x[7,10] - 41.25735446569939 x[7,11]
   - 14.1428419288044 x[8,1] - 13.03236660711514 x[8,2]
   - 29.8802896095911 x[8,4] - 17.46007934025616 x[8,5]
   - 31.32284752950167 x[8,6] - 13.03236660711514 x[8,7]
   - 29.8802896095911 x[8,9] - 17.46007934025616 x[8,10]
   - 31.32284752950167 x[8,11] - 42.32289311175671 x[9,1]
   - 40.91088747229708 x[9,2] - 29.8802896095911 x[9,3]
   - 34.57608972097309 x[9,5] - 5.12165283774609 x[9,6]
   - 40.91088747229708 x[9,7] - 29.8802896095911 x[9,8]
   - 34.57608972097309 x[9,10] - 5.12165283774609 x[9,11]
   - 27.99802340579372 x[10,1] - 13.90172026378779 x[10,2]
   - 17.46007934025616 x[10,3] - 34.57608972097309 x[10,4]
   - 33.29542776724454 x[10,6] - 13.90172026378779 x[10,7]
   - 17.46007934025616 x[10,8] - 34.57608972097309 x[10,9]
   - 33.29542776724454 x[10,11] - 44.53860566304473 x[11,1]
   - 41.25735446569939 x[11,2] - 31.32284752950167 x[11,3]
   - 5.12165283774609 x[11,4] - 33.29542776724454 x[11,5]
   - 41.25735446569939 x[11,7] - 31.32284752950167 x[11,8]
   - 5.12165283774609 x[11,9] - 33.29542776724454 x[11,10]
   - 16.11299916639204 x[12,2] - 14.1428419288044 x[12,3]
   - 42.32289311175671 x[12,4] - 27.99802340579372 x[12,5]
   - 44.53860566304473 x[12,6] - 16.11299916639204 x[12,7]
   - 14.1428419288044 x[12,8] - 42.32289311175671 x[12,9]
   - 27.99802340579372 x[12,10] - 44.53860566304473 x[12,11]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[0]: - 32.22599833278407 x[0,2]
   - 28.28568385760881 x[0,3] - 84.64578622351341 x[0,4]
   - 55.99604681158743 x[0,5] - 89.07721132608947 x[0,6]
   - 32.22599833278407 x[0,7] - 28.28568385760881 x[0,8]
   - 84.64578622351341 x[0,9] - 55.99604681158743 x[0,10]
   - 89.07721132608947 x[0,11] - service_time[0] - start_service[0]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[1]: - 32.22599833278407 x[1,2]
   - 28.28568385760881 x[1,3] - 84.64578622351341 x[1,4]
   - 55.99604681158743 x[1,5] - 89.07721132608947 x[1,6]
   - 32.22599833278407 x[1,7] - 28.28568385760881 x[1,8]
   - 84.64578622351341 x[1,9] - 55.99604681158743 x[1,10]
   - 89.07721132608947 x[1,11] - service_time[1] - start_service[1]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[2]: - 32.22599833278407 x[2,1]
   - 26.06473321423027 x[2,3] - 81.82177494459415 x[2,4]
   - 27.80344052757557 x[2,5] - 82.51470893139879 x[2,6]
   - 26.06473321423027 x[2,8] - 81.82177494459415 x[2,9]
   - 27.80344052757557 x[2,10] - 82.51470893139879 x[2,11]
   - service_time[2] - start_service[2] + start_service[13] >= 0
 LowerBoundRoutingVehicle[3]: - 28.28568385760881 x[3,1]
   - 26.06473321423027 x[3,2] - 59.7605792191822 x[3,4]
   - 34.92015868051232 x[3,5] - 62.64569505900334 x[3,6]
   - 26.06473321423027 x[3,7] - 59.7605792191822 x[3,9]
   - 34.92015868051232 x[3,10] - 62.64569505900334 x[3,11]
   - service_time[3] - start_service[3] + start_service[13] >= 0
 LowerBoundRoutingVehicle[4]: - 84.64578622351341 x[4,1]
   - 81.82177494459415 x[4,2] - 59.7605792191822 x[4,3]
   - 69.15217944194617 x[4,5] - 10.24330567549218 x[4,6]
   - 81.82177494459415 x[4,7] - 59.7605792191822 x[4,8]
   - 69.15217944194617 x[4,10] - 10.24330567549218 x[4,11]
   - service_time[4] - start_service[4] + start_service[13] >= 0
 LowerBoundRoutingVehicle[5]: - 55.99604681158743 x[5,1]
   - 27.80344052757557 x[5,2] - 34.92015868051232 x[5,3]
   - 69.15217944194617 x[5,4] - 66.59085553448908 x[5,6]
   - 27.80344052757557 x[5,7] - 34.92015868051232 x[5,8]
   - 69.15217944194617 x[5,9] - 66.59085553448908 x[5,11] - service_time[5]
   - start_service[5] + start_service[13] >= 0
 LowerBoundRoutingVehicle[6]: - 89.07721132608947 x[6,1]
   - 82.51470893139879 x[6,2] - 62.64569505900334 x[6,3]
   - 10.24330567549218 x[6,4] - 66.59085553448908 x[6,5]
   - 82.51470893139879 x[6,7] - 62.64569505900334 x[6,8]
   - 10.24330567549218 x[6,9] - 66.59085553448908 x[6,10] - service_time[6]
   - start_service[6] + start_service[13] >= 0
 LowerBoundRoutingVehicle[7]: - 32.22599833278407 x[7,1]
   - 26.06473321423027 x[7,3] - 81.82177494459415 x[7,4]
   - 27.80344052757557 x[7,5] - 82.51470893139879 x[7,6]
   - 26.06473321423027 x[7,8] - 81.82177494459415 x[7,9]
   - 27.80344052757557 x[7,10] - 82.51470893139879 x[7,11]
   - service_time[7] - start_service[7] + start_service[13] >= 0
 LowerBoundRoutingVehicle[8]: - 28.28568385760881 x[8,1]
   - 26.06473321423027 x[8,2] - 59.7605792191822 x[8,4]
   - 34.92015868051232 x[8,5] - 62.64569505900334 x[8,6]
   - 26.06473321423027 x[8,7] - 59.7605792191822 x[8,9]
   - 34.92015868051232 x[8,10] - 62.64569505900334 x[8,11]
   - service_time[8] - start_service[8] + start_service[13] >= 0
 LowerBoundRoutingVehicle[9]: - 84.64578622351341 x[9,1]
   - 81.82177494459415 x[9,2] - 59.7605792191822 x[9,3]
   - 69.15217944194617 x[9,5] - 10.24330567549218 x[9,6]
   - 81.82177494459415 x[9,7] - 59.7605792191822 x[9,8]
   - 69.15217944194617 x[9,10] - 10.24330567549218 x[9,11]
   - service_time[9] - start_service[9] + start_service[13] >= 0
 LowerBoundRoutingVehicle[10]: - 55.99604681158743 x[10,1]
   - 27.80344052757557 x[10,2] - 34.92015868051232 x[10,3]
   - 69.15217944194617 x[10,4] - 66.59085553448908 x[10,6]
   - 27.80344052757557 x[10,7] - 34.92015868051232 x[10,8]
   - 69.15217944194617 x[10,9] - 66.59085553448908 x[10,11]
   - service_time[10] - start_service[10] + start_service[13] >= 0
 LowerBoundRoutingVehicle[11]: - 89.07721132608947 x[11,1]
   - 82.51470893139879 x[11,2] - 62.64569505900334 x[11,3]
   - 10.24330567549218 x[11,4] - 66.59085553448908 x[11,5]
   - 82.51470893139879 x[11,7] - 62.64569505900334 x[11,8]
   - 10.24330567549218 x[11,9] - 66.59085553448908 x[11,10]
   - service_time[11] - start_service[11] + start_service[13] >= 0
 LowerBoundRoutingVehicle[12]: - 32.22599833278407 x[12,2]
   - 28.28568385760881 x[12,3] - 84.64578622351341 x[12,4]
   - 55.99604681158743 x[12,5] - 89.07721132608947 x[12,6]
   - 32.22599833278407 x[12,7] - 28.28568385760881 x[12,8]
   - 84.64578622351341 x[12,9] - 55.99604681158743 x[12,10]
   - 89.07721132608947 x[12,11] - service_time[12] - start_service[12]
   + start_service[13] >= 0
 LowerBoundArrivalVehicle[1]: - 64.45199666556815 x[2,1]
   - 56.57136771521762 x[3,1] - 169.2915724470268 x[4,1]
   - 111.9920936231749 x[5,1] - 178.1544226521789 x[6,1]
   - 64.45199666556815 x[7,1] - 56.57136771521762 x[8,1]
   - 169.2915724470268 x[9,1] - 111.9920936231749 x[10,1]
   - 178.1544226521789 x[11,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 32.22599833278407 x[0,2]
   - 32.22599833278407 x[1,2] - 32.22599833278407 x[2,2]
   - 54.35041707183908 x[3,2] - 166.4675611681076 x[4,2]
   - 83.79948733916301 x[5,2] - 171.5919202574883 x[6,2]
   - 32.22599833278407 x[7,2] - 54.35041707183908 x[8,2]
   - 166.4675611681076 x[9,2] - 83.79948733916301 x[10,2]
   - 171.5919202574883 x[11,2] - 32.22599833278407 x[12,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 28.28568385760881 x[0,3]
   - 28.28568385760881 x[1,3] - 58.29073154701435 x[2,3]
   - 28.28568385760881 x[3,3] - 144.4063654426956 x[4,3]
   - 90.91620549209975 x[5,3] - 151.7229063850928 x[6,3]
   - 58.29073154701435 x[7,3] - 28.28568385760881 x[8,3]
   - 144.4063654426956 x[9,3] - 90.91620549209975 x[10,3]
   - 151.7229063850928 x[11,3] - 28.28568385760881 x[12,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 84.64578622351341 x[0,4]
   - 84.64578622351341 x[1,4] - 114.0477732773782 x[2,4]
   - 88.04626307679101 x[3,4] - 84.64578622351341 x[4,4]
   - 125.1482262535336 x[5,4] - 99.32051700158165 x[6,4]
   - 114.0477732773782 x[7,4] - 88.04626307679101 x[8,4]
   - 84.64578622351341 x[9,4] - 125.1482262535336 x[10,4]
   - 99.32051700158165 x[11,4] - 84.64578622351341 x[12,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 55.99604681158743 x[0,5]
   - 55.99604681158743 x[1,5] - 60.02943886035965 x[2,5]
   - 63.20584253812113 x[3,5] - 153.7979656654596 x[4,5]
   - 55.99604681158743 x[5,5] - 155.6680668605786 x[6,5]
   - 60.02943886035965 x[7,5] - 63.20584253812113 x[8,5]
   - 153.7979656654596 x[9,5] - 55.99604681158743 x[10,5]
   - 155.6680668605786 x[11,5] - 55.99604681158743 x[12,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 89.07721132608947 x[0,6]
   - 89.07721132608947 x[1,6] - 114.7407072641829 x[2,6]
   - 90.93137891661215 x[3,6] - 94.88909189900559 x[4,6]
   - 122.5869023460765 x[5,6] - 89.07721132608947 x[6,6]
   - 114.7407072641829 x[7,6] - 90.93137891661215 x[8,6]
   - 94.88909189900559 x[9,6] - 122.5869023460765 x[10,6]
   - 89.07721132608947 x[11,6] - 89.07721132608947 x[12,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 32.22599833278407 x[0,7]
   - 32.22599833278407 x[1,7] - 32.22599833278407 x[2,7]
   - 54.35041707183908 x[3,7] - 166.4675611681076 x[4,7]
   - 83.79948733916301 x[5,7] - 171.5919202574883 x[6,7]
   - 32.22599833278407 x[7,7] - 54.35041707183908 x[8,7]
   - 166.4675611681076 x[9,7] - 83.79948733916301 x[10,7]
   - 171.5919202574883 x[11,7] - 32.22599833278407 x[12,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 28.28568385760881 x[0,8]
   - 28.28568385760881 x[1,8] - 58.29073154701435 x[2,8]
   - 28.28568385760881 x[3,8] - 144.4063654426956 x[4,8]
   - 90.91620549209975 x[5,8] - 151.7229063850928 x[6,8]
   - 58.29073154701435 x[7,8] - 28.28568385760881 x[8,8]
   - 144.4063654426956 x[9,8] - 90.91620549209975 x[10,8]
   - 151.7229063850928 x[11,8] - 28.28568385760881 x[12,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 84.64578622351341 x[0,9]
   - 84.64578622351341 x[1,9] - 114.0477732773782 x[2,9]
   - 88.04626307679101 x[3,9] - 84.64578622351341 x[4,9]
   - 125.1482262535336 x[5,9] - 99.32051700158165 x[6,9]
   - 114.0477732773782 x[7,9] - 88.04626307679101 x[8,9]
   - 84.64578622351341 x[9,9] - 125.1482262535336 x[10,9]
   - 99.32051700158165 x[11,9] - 84.64578622351341 x[12,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 55.99604681158743 x[0,10]
   - 55.99604681158743 x[1,10] - 60.02943886035965 x[2,10]
   - 63.20584253812113 x[3,10] - 153.7979656654596 x[4,10]
   - 55.99604681158743 x[5,10] - 155.6680668605786 x[6,10]
   - 60.02943886035965 x[7,10] - 63.20584253812113 x[8,10]
   - 153.7979656654596 x[9,10] - 55.99604681158743 x[10,10]
   - 155.6680668605786 x[11,10] - 55.99604681158743 x[12,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 89.07721132608947 x[0,11]
   - 89.07721132608947 x[1,11] - 114.7407072641829 x[2,11]
   - 90.93137891661215 x[3,11] - 94.88909189900559 x[4,11]
   - 122.5869023460765 x[5,11] - 89.07721132608947 x[6,11]
   - 114.7407072641829 x[7,11] - 90.93137891661215 x[8,11]
   - 94.88909189900559 x[9,11] - 122.5869023460765 x[10,11]
   - 89.07721132608947 x[11,11] - 89.07721132608947 x[12,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 32.22599833278407 x[2,12]
   - 28.28568385760881 x[3,12] - 84.64578622351341 x[4,12]
   - 55.99604681158743 x[5,12] - 89.07721132608947 x[6,12]
   - 32.22599833278407 x[7,12] - 28.28568385760881 x[8,12]
   - 84.64578622351341 x[9,12] - 55.99604681158743 x[10,12]
   - 89.07721132608947 x[11,12] - start_service[0] + start_service[12]
   >= 0
 LowerBoundArrivalVehicle[13]: - 32.22599833278407 x[2,13]
   - 28.28568385760881 x[3,13] - 84.64578622351341 x[4,13]
   - 55.99604681158743 x[5,13] - 89.07721132608947 x[6,13]
   - 32.22599833278407 x[7,13] - 28.28568385760881 x[8,13]
   - 84.64578622351341 x[9,13] - 55.99604681158743 x[10,13]
   - 89.07721132608947 x[11,13] - start_service[0] + start_service[13]
   >= 0
 VisitArrivalNodesNotEmpty[2]: w[0,2] + w[1,2] + w[2,2] + w[3,2] + w[4,2]
   + w[5,2] + w[6,2] + w[7,2] + w[8,2] + w[9,2] + w[10,2] + w[11,2]
   + w[12,2] + w[13,2] >= 1
 VisitArrivalNodesNotEmpty[3]: w[0,3] + w[1,3] + w[2,3] + w[3,3] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] + w[13,3] >= 1
 VisitArrivalNodesNotEmpty[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] + w[4,4]
   + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4]
   + w[12,4] + w[13,4] >= 1
 VisitArrivalNodesNotEmpty[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   + w[5,5] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5]
   + w[12,5] + w[13,5] >= 1
 VisitArrivalNodesNotEmpty[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] + w[6,6] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6]
   + w[12,6] + w[13,6] >= 1
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
 LeaveDepartureNodesNotEmpty[7]: w[7,0] + w[7,1] + w[7,2] + w[7,3] + w[7,4]
   + w[7,5] + w[7,6] + w[7,7] + w[7,8] + w[7,9] + w[7,10] + w[7,11]
   + w[7,12] + w[7,13] >= 1
 LeaveDepartureNodesNotEmpty[8]: w[8,0] + w[8,1] + w[8,2] + w[8,3] + w[8,4]
   + w[8,5] + w[8,6] + w[8,7] + w[8,8] + w[8,9] + w[8,10] + w[8,11]
   + w[8,12] + w[8,13] >= 1
 LeaveDepartureNodesNotEmpty[9]: w[9,0] + w[9,1] + w[9,2] + w[9,3] + w[9,4]
   + w[9,5] + w[9,6] + w[9,7] + w[9,8] + w[9,9] + w[9,10] + w[9,11]
   + w[9,12] + w[9,13] >= 1
 LeaveDepartureNodesNotEmpty[10]: w[10,0] + w[10,1] + w[10,2] + w[10,3]
   + w[10,4] + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,10]
   + w[10,11] + w[10,12] + w[10,13] >= 1
 LeaveDepartureNodesNotEmpty[11]: w[11,0] + w[11,1] + w[11,2] + w[11,3]
   + w[11,4] + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10]
   + w[11,11] + w[11,12] + w[11,13] >= 1
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
