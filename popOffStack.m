% helper function to remove top element of a stack
function [outElement, newStack] = popOffStack(originalStack)
    [n,m] = size(originalStack);
    outElement = originalStack(1,:);
    newStack = originalStack(2:n,:);