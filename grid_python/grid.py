from tkinter import *
from tkinter import filedialog as fd
import pandas as pd
import numpy as np

class Cell():
    FILLED_COLOR_BG = "green"
    EMPTY_COLOR_BG = "white"
    FILLED_COLOR_BORDER = "green"
    EMPTY_COLOR_BORDER = "black"

    def __init__(self, master, x, y, size):
        """ Constructor of the object called by Cell(...) """
        self.master = master
        self.abs = x
        self.ord = y
        self.size= size
        self.fill= False

    def _switch(self):
        """ Switch if the cell is filled or not. """
        self.fill= not self.fill

    def draw(self):
        """ order to the cell to draw its representation on the canvas """
        if self.master != None :
            fill = Cell.FILLED_COLOR_BG
            outline = Cell.FILLED_COLOR_BORDER

            if not self.fill:
                fill = Cell.EMPTY_COLOR_BG
                outline = Cell.EMPTY_COLOR_BORDER

            xmin = self.abs * self.size
            xmax = xmin + self.size
            ymin = self.ord * self.size
            ymax = ymin + self.size

            self.master.create_rectangle(xmin, ymin, xmax, ymax, fill = fill, outline = outline)

    def set_state(self, value):
        if value == 0:
            self.fill = False
        else:
            self.fill = True

    def get_state(self):
        if self.fill is False:
            return 0
        else:
            return 1

class CellGrid(Canvas):
    def __init__(self,master, rowNumber, columnNumber, cellSize, *args, **kwargs):
        Canvas.__init__(self, master, width = cellSize * columnNumber , height = cellSize * rowNumber, *args, **kwargs)

        frame = Frame(master)
        frame.pack()
        self.button_load = Button(frame, 
                             text="LOAD Grid", fg="green",
                             command=self.filename_callback)
        self.button_load.pack(side=LEFT)

        self.button_save = Button(frame, 
                             text="SAVE Grid", fg="red",
                             command=self.save)
        self.button_save.pack(side=RIGHT)


        self.cellSize = cellSize
        self.columnNumber = columnNumber
        self.rowNumber = rowNumber

        self.grid = []
        for row in range(rowNumber):

            line = []
            for column in range(columnNumber):
                line.append(Cell(self, column, row, cellSize))

            self.grid.append(line)

        #memorize the cells that have been modified to avoid many switching of state during mouse motion.
        self.switched = []

        #bind click action
        self.bind("<Button-1>", self.handleMouseClick)  
        #bind moving while clicking
        self.bind("<B1-Motion>", self.handleMouseMotion)
        #bind release button action - clear the memory of midified cells.
        self.bind("<ButtonRelease-1>", lambda event: self.switched.clear())

        self.draw()

    def draw(self):
        for row in self.grid:
            for cell in row:
                cell.draw()

    def _eventCoords(self, event):
        row = int(event.y / self.cellSize)
        column = int(event.x / self.cellSize)
        return row, column

    def handleMouseClick(self, event):
        row, column = self._eventCoords(event)
        cell = self.grid[row][column]
        cell._switch()
        cell.draw()
        #add the cell to the list of cell switched during the click
        self.switched.append(cell)

    def handleMouseMotion(self, event):
        row, column = self._eventCoords(event)
        cell = self.grid[row][column]

        if cell not in self.switched:
            cell._switch()
            cell.draw()
            self.switched.append(cell)

    def save(self):
        matrix = np.zeros((self.rowNumber, self.columnNumber))
        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                matrix[i,j] = cell.get_state()

        df = pd.DataFrame(data=matrix.astype(int))
        df.to_csv('grid.csv', header=False, index=False)
        print('saved grid sucessfuly!')


    def load(self, filename):
        matrix = pd.read_csv(filename, header=None).to_numpy()

        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                cell.set_state(matrix[i,j])
                cell.draw()

        print('loaded matrix sucessfuly!')

    def filename_callback(self):
        filename = fd.askopenfilename() 
        self.load(filename)



if __name__ == "__main__" :
    app = Tk()

    grid = CellGrid(app, 60, 100, 15)
    grid.pack()

    app.mainloop()
