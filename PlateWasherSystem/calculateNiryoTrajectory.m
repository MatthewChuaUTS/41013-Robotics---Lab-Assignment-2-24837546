% function [niryoTrajectoryQmatrix] = calculateNiryoTrajectory(myNiryoOne, totalSteps)
% end

niryoOneInitialJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
myNiryoOne.model.animate(niryoOneCurrentJointPosition);  
myNiryoOne.model.teach(niryoOneCurrentJointPosition);