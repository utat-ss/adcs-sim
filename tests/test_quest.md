Disclaimer: I am definitely going to use AI to reformat this segment and make it more formal. Please bear that in mind. I don't like writing markdowns too often, so I'm going to do this to save time.

Anyways 
make_case: I saw the methods used in Q_method.py and thought that it might be a good idea to use truly random vectors. So the way that it was done is to create a random number using a seed (so that it is predictable) using that a random 3 vector is generated and turned into a quaternion where it is normalized (Gaussian generation).
We then add some noise to the system to simulate what real systems are like. The added Gaussian term simulates sensor error. Noise = 0 imitates perfect sensors.

load_q_method: I loved the individualized test cases written and I thought we can test against both, so I decided to incorporate that into the testbench as well as a method of cross validation.

test_helpers_match_hand_derived_rotation: We assume passive rotation as the main method instead of active rotation, once the test is complete I'll assert whether this is true. 

test_random_round_trip: This just tests whether the algorirthm can recover the initial vectors given 0 noise.

test_identity_rotation: This just tests a property of QUEST which gives Identity matrix on using the same vector. QUEST tests whether a vector is aligned with another vector and by how much, the same vector means we don't rotate. That's a quaternion [0,0,0,1] but we write that as a matrix. Which is just the identity.

test_angle_sweep: The issue last time was that when the angle rotated to 180 degrees it blew the error up. It's an expected failure. To be taken into account.

It was at this time I realized that we're using a different rotation convention because I was seeing this
guous. Use a.any() or a.all()
========================================= 16 failed, 1 passed in 0.19s ============

Also I took a look at the git tag 3b6a6a3
Something's definitely up but I'm pushing this code to my branch anyway