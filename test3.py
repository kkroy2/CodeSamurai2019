import math


class node:
    def __init__(self,d):
        self.d = d

class heap:
    def __init__(self):
        self.arr = [None for i in range(10000000)]
        self.size = 0

    def pr(self):
        for i in range(self.size):
            print(self.arr[i],end=" ")

    def insert(self, n):
        self.size += 1
        i = self.size -1
        self.arr[i] = n

        while i != 0 and self.arr[self.parent(i)] > self.arr[i]:
            temp = self.arr[self.parent(i)]
            self.arr[self.parent(i)] = self.arr[i]
            self.arr[i] = temp
            i = self.parent(i)

    def pop(self):
        if self.size ==0:
            return None
        if self.size == 1:
            self.size -= 1
            return self.arr[0]

        root = self.arr[0]
        self.arr[0] = self.arr[self.size - 1]
        self.size -= 1
        self.minHeapify(0)
        return root

    def minHeapify(self,i):
        l = 2*i+1
        r = 2*i+2
        smallest = i
        if l < self.size and self.arr[l] < self.arr[i]:
            smallest = l
        if r < self.size and self.arr[r] < self.arr[smallest]:
            smallest = r
        if smallest != i:
            temp = self.arr[i]
            self.arr[i] = self.arr[smallest]
            self.arr[smallest] = temp
            self.minHeapify(smallest)


    def parent(self, i):
        return int(math.floor((i-1)/2))

h = heap()
h.insert(3)
h.insert(30)
h.insert(23)
h.insert(34)

while True:
    head = h.pop()
    if head is None:
        break
    print(head)
