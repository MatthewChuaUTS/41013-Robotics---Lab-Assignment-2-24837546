%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses(q, myNiryoOne)
    links = myNiryoOne.model.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = myNiryoOne.model.base;

    for i = 1:length(links)
        L = links(1,i);

        current_transform = transforms(:,:, i);

        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end