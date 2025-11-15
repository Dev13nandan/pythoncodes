# *****************************************************************
# Kevin and Stuart want to play the 'The Minion Game'.

# Game Rules

# Both players are given the same string, .
# Both players have to make substrings using the letters of the string .
# Stuart has to make words starting with consonants.
# Kevin has to make words starting with vowels.
# The game ends when both players have made all possible substrings.

# Scoring
# A player gets +1 point for each occurrence of the substring in the string .

# For Example:
# String  = BANANA
# Kevin's vowel beginning word = ANA
# Here, ANA occurs twice in BANANA. Hence, Kevin will get 2 Points.
# *****************************************************************
def minion_game(s):
    s = s.upper()
    vowels = set("AEIOU")
    n = len(s)
    kevin = stuart = 0
    for i, ch in enumerate(s):
        if ch in vowels:
            kevin += n - i
        else:
            stuart += n - i
    if kevin > stuart:
        print("Kevin", kevin)
    elif stuart > kevin:
        print("Stuart", stuart)
    else:
        print("Draw")


if __name__ == '__main__':
# *****************************************************************
# Consider the following:

# A string, , of length  where .
# An integer, , where  is a factor of .
# We can split  into  substrings where each subtring, , consists of a contiguous block of  characters in . Then, use each  to create string  such that:

# The characters in  are a subsequence of the characters in .
# Any repeat occurrence of a character is removed from the string such that each character in  occurs exactly once. In other words, if the character at some index  in  occurs at a previous index  in , then do not include the character in string .
# Given  and , print  lines where each line  denotes string
# *****************************************************************

def merge_the_tools(string, k):
    n=len(string)
    for i in range(0,n,k):
        t=string[i:k+i]
        u=""
        for ch in t:
            if ch not in u:
                u+=ch
        print (u)
        
        
if __name__ == '__main__':
# *****************************************************************
# Particulate matters are the biggest contributors to Delhi pollution. The main reason behind the increase in the concentration of PMs include vehicle emission by applying Odd Even concept for all types of vehicles. The vehicles with the odd last digit in the registration number will be allowed on roads on odd dates and those with even last digit will on even dates. Given an integer array a[], contains the last digit of the registration number of N vehicles traveling on date D(a positive integer). The task is to calculate the total fine collected by the traffic police department from the vehicles violating the rules. Note : For violating the rule, vehicles would be fined as X Rs. Input Format: The candidate has to write the code to accept 4 input(s). First input – Accept for N(Positive integer) values (a[]), where each value is separated by a new line. Third input – Accept value for D(Positive integer) Fourth input – Accept value for X(Positive integer ) Output Format: The output should be a positive integer number if no fine is collected then print ”0”. Sample Input: 4 -> Value of N {5,2,3,7} -> a[], Elements a[0] to a[N-1], during input each element is separated by a new line 12 -> Value of D, i.e. date 200 -> Value of x i.e. fine Output: 600 -> total fine collected Explanation: Date D=12 means , only an even number of vehicles are allowed. Fine will be collected from 5,3 and 7 with an amount of 200 each. Hence, the output = 600. Input Format 4 5 2 3 7 12 200 Constraints 0 Output Format 600
# *****************************************************************
import sys   

data = list(map(int, sys.stdin.read().split()))

n = data[0]
date = data[-2]
fine = data[-1]
x = 0

for i in range(n):
    num = data[i + 1]
    if date % 2 == 0:        
        if num % 2 != 0:     
            x += 1
    else:                    
        if num % 2 == 0:     
            x += 1

print(fine * x)
# *****************************************************************
# Before the outbreak of corona virus to the world, a meeting happened in a room in Wuhan. A person who attended that meeting had COVID-19 and no one in the room knew about it! So everyone started shaking hands with everyone else in the room as a gesture of respect and after meeting unfortunately every one got infected! Given the fact that any two persons shake hand exactly once, Can you tell the total count of handshakes happened in that meeting?

# Input Format: The first line contains the number of test cases T, T lines follow.
# Each line then contains an integer N, the total number of people attended that meeting.

# Output Format: Print the number of handshakes for each test-case in a new line.

# Sample Input: 2 1 2

# Output: 0 1

# Explanation: Case 1 : The lonely board member shakes no hands, hence 0.
# Case 2 : There are 2 board members, 1 handshake takes place.
# *****************************************************************
data = list(map(int, input().split()))

t = data[0]     
p = 1           
result = []    

for _ in range(t):
    if p < len(data):        
        n = data[p]
        p += 1
        h = (n * (n - 1)) // 2
        result.append(str(h))
    else:
        break              
print(" ".join(result))
# *****************************************************************
# An automobile company manufactures both a two wheeler (TW) and a four wheeler (FW). A company manager wants to make the production of both types of vehicle according to the given data below: 1st data, Total number of vehicle (two-wheeler + four-wheeler)=v 2nd data, Total number of wheels = W The task is to find how many two-wheelers as well as four-wheelers need to manufacture as per the given data Example: Input: 200 -> Value of V 540 -> Value of W Output: TW =130 FW=70 Explanation: 130+70 = 200 vehicles (70*4)+(130*2)= 540 wheels Input Format 200 540 Constraints 2<=W W%2=0 V Output Format TW =130 FW=70 Sample Input 0 200 540 Sample Output 0 TW =130 FW=70 Sample Input 1 150 440 Sample Output 1 Tw=80 Fw=70
#*****************************************************************
vehicle = int(input())
wheel = int(input())

for i in range(vehicle + 1):
    TW = vehicle - i
    if (i * 4) + (TW * 2) == wheel:
        print("TW =", TW)
        print("FW=", i, sep="")
        break





