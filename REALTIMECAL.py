###Start with CalSamples###

###find mean and standard deviation of each dimension and sensor type###

###the mean turns into our acceleration offset and the magnitude is used to set our sensor weight###

###For each new sample we check if the acceleration in the body frame fall within one standar deviation of the previous measurment###

### this will tell us if this direction is stationary and stable###

### if the acceleration does fall within the bounds we add one to a ticker###

### if the acceleration does not fall within the bounds we set the ticker back to zero###

### there needs to be one ticker for each dimension###

### if the ticker reaches the number we predetermined as the number of calibration samples we can check to see if we should recalibrate this axis###

### to do this we will average all of the accelerations that helped contribute to the ticker going up###

### if this average falls outside of the standard deviation of the privious calibration samples we can recalibrate###

### if this average falls within the bounds then there is no need to recalibrate and the ticker is set back to zero###

### there should be some other check to make sure it is not just accelerating very slowley###

### like the next acceleration has to be within one standard deviation of the current one and must also be within two standard deviations of the original offset###

### something like that but i still have to think about that a little. For now we dont need it###

j = 0
if a[t] is in range(a[t - 1] -s.d., a[t -1] +s.d.):
    j = j + 1
else:
    j = 0

if j == number of cal_samples:
    x = sum(a_body[t - number of cal_samples] to a_body[number of cal_samples])/number of cal_samples
    if x is in range(mean-s.d.,mean+s.d.):
        x is the new offset
    else:
        j = 0

###this can also be done for gyroscope samples too###

### it might be smart to write the code genaricaly such that we can apply it to any sample###
        
