class Cell:
    def __init__(self, row, col, content, visited):
        self.row = row
        self.col = col
        self.content = content
        self.visited = visited

        self.m_c = 0
        self.nw_c = 0
        self.ne_c = 0
        self.sw_c = 0
        self.se_c = 0

        self.w_w = 0
        self.e_w = 0
        self.s_w = 0
        self.n_w = 0
        


class World:
    def __init__(self, row_num, col_num):
        self.grid = []
        for row in range(row_num):
            gridRow = []
            for col in range(col_num):
                gridRow.append(Cell(row,col,0,False))
            self.grid.append(row)
        
        self.x = row_num/2
        self.y = col_num/2
        self.mypos = self.getCell(self.x, self.y)

    def updatePos(self, dir):
        if dir=='north':
            self.x = self.x
            self.y = self.y+1
        elif dir=='south':
            self.x = self.x
            self.y = self.y-1
        elif dir=='west':
            self.x = self.x-1
            self.y = self.y
        elif dir=='east':
            self.x = self.x+1
            self.y = self.y

    def getCell(self, row, col):
        res = None
        for i in range(len(self.grid)):
            if row == self.grid[i].row and col == self.grid[i].col:
                res = self.grid[i]
        return res
    