### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3

# This file generates the four types of problems: LOR, POR, LTR, POR
import random

'''
Return a randomized array with n entries containing integers from 
0 to n - 1, each appearing once
'''
def get_lor_instance(n):
    # Create a random list of integers from 0 to n - 1
    rl = []
    for i in range(0, n):
        rl.insert(random.randint(0,i), i)
        
    return rl


def get_lor_instance_2(n, m):
    # Create a random list of integers from 0 to n - 1
    rl = []
    for mi in range(0, m):
        for i in range(0, n):
            rl.insert(random.randint(0,i), m*n - mi*n - i - 1)
        
    return rl

def get_lor_instance_3(n, m):
    # Create a random list of integers from 0 to n - 1
    rl = []
    for mi in range(0, m):
        for i in range(0, n):
            rl.insert(random.randint(0,i + (n//2 if mi > 0 else 0)), m*n - mi*n - i - 1)
        
    return rl

'''
Return a randomized array with length n*k entries containing 
integers from 0 to n - 1, each appearing k times.
'''
def get_por_instance(n, k):
    # Create a random list of integers from 0 to n*k - 1
    rl = get_lor_instance(n*k)
    
    # Update list so that i -> i/k
    for i in range(0, n*k):
        rl[i] = rl[i]//n

    return rl

def get_por_instance_pl(n, k):
    # Create a random list of integers from 0 to n*k - 1
    lor = get_lor_instance(n*k)
    por = get_lor_instance(n*k)
    
    # Update list so that i -> i/k
    for i in range(0, n*k):
        por[i] = lor[i]//n

    return (lor, por)

def get_por_instance_2(n, k):
    # Create a random list of integers from 0 to n*k - 1
    rl = get_lor_instance_2(n, k)
    
    # Update list so that i -> i/k
    for i in range(0, n*k):
        rl[i] = rl[i]//n

    return rl

def get_por_instance_3(n, k, nn, kk): 
    # Create a random list of integers from 0 to n*k - 1
    rl = get_lor_instance_2(nn, kk)
    
    # Update list so that i -> i/k
    for i in range(0, n*k):
        rl[i] = rl[i]//n

    return rl


'''
Return a randomized 2D array with length n^2 entries containing 
integers from 0 to n^2 - 1, each appearing once.
'''
def get_ltr_instance(n):
    # Decleare n*n array 
    arr = [[0]*n for i in range(n)]

    # Get a random list with numbers from 0 to n^2
    rl = get_lor_instance(n*n)

    for i in range(0, n):
        for j in range(0, n):
            arr[i][j] = rl[i*n + j]

    return (rl, arr)

def get_ltr_instance_block(n, sqrtn):
    # Decleare n*n array 
    arr = [[0]*n for i in range(n)]

    rl = [0 for i in range(n*n)]
    # Get a random list with numbers from 0 to n^2
    for i in range(sqrtn):
        for j in range(sqrtn):
            nrl = get_lor_instance(n)
            for ii in range(sqrtn):
                for jj in range(sqrtn):
                    idx = ii*sqrtn + jj
                    iii = nrl[idx] // sqrtn
                    jjj = nrl[idx] % sqrtn
                    rl[i*n*sqrtn + j*sqrtn + ii*n + jj] = i*n*sqrtn + j*sqrtn + iii*n + jjj

    for i in range(0, n):
        for j in range(0, n):
            arr[i][j] = rl[i*n + j]

    return (rl, arr)

def get_ltr_instance_column(n):
    # Decleare n*n array 
    arr = [[0]*n for i in range(n)]

    # Get a random list with numbers from 0 to n^2
    rl = get_lor_instance_2(n, n)

    for i in range(0, n):
        for j in range(0, n):
            arr[i][j] = rl[i*n + j]

    return (rl, arr)


'''
Return a randomized 2D array with length n^2 entries containing 
integers from 0 to n - 1, each appearing n times.
'''
def get_ptr_instance(n):
    # Decleare n*n array 
    arr = [[0]*n for i in range(n)]

    # Get a random list with numbers from 0 to n^2
    rl = get_lor_instance(n*n)
    # rl = get_lor_instance_2(n, n)

    for i in range(0, n):
        for j in range(0, n):
            arr[i][j] = rl[i*n + j]//n

    return (rl, arr)


#print(get_por_instance(4, 3))
#print (get_lor_instance(16))
#print(get_lor_instance_2(10,10))

#print(get_ltr_instance_block(4,2))