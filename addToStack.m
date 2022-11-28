% adds newElement (1xM row vector) to the originalStack (N x M array)
function outStack = addToStack(newElement, originalStack)
    outStack = [newElement; originalStack];
