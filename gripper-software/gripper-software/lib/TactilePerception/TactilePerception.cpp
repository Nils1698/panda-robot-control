

#include "TactilePerception.h"

#include <vector>

namespace TactilePerception {


// https://www.geeksforgeeks.org/find-number-of-islands/

// A utility function to do DFS for a 2D
//  boolean matrix. It only considers
// the 8 neighbours as adjacent vertices
void DFS(std::vector<std::vector<int>> &M, int i, int j, int ROW, int COL){
    
    //Base condition
    //if i less than 0 or j less than 0 or i greater than ROW-1 or j greater than COL-  or if M[i][j] != 1 then we will simply return
    if(i < 0 || j < 0 || i > (ROW - 1) || j > (COL - 1) || M[i][j] != 1){
        return;
    }
 
    if (M[i][j] == 1){
        M[i][j] = 0;
        DFS(M, i + 1, j, ROW, COL);     //right side traversal
        DFS(M, i - 1, j, ROW, COL);     //left side traversal
        DFS(M, i, j + 1, ROW, COL);     //upward side traversal
        DFS(M, i, j - 1, ROW, COL);     //downward side traversal
        // If using 8-neighbours include diagonals:
        // DFS(M, i + 1, j + 1, ROW, COL); //upward-right side traversal
        // DFS(M, i - 1, j - 1, ROW, COL); //downward-left side traversal
        // DFS(M, i + 1, j - 1, ROW, COL); //downward-right side traversal
        // DFS(M, i - 1, j + 1, ROW, COL); //upward-left side traversal
    }
}
 
int countIslands(std::vector<std::vector<int>> &M)
{
    int ROW = M.size();
    int COL = M[0].size();
    int count = 0;
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            if (M[i][j] == 1)
            {
                M[i][j] = 0;
                count++;
                DFS(M, i + 1, j, ROW, COL);     //right side traversal
                DFS(M, i - 1, j, ROW, COL);     //left side traversal
                DFS(M, i, j + 1, ROW, COL);     //upward side traversal
                DFS(M, i, j - 1, ROW, COL);     //downward side traversal
                // If using 8-neighbours include diagonals:
                // DFS(M, i + 1, j + 1, ROW, COL); //upward-right side traversal
                // DFS(M, i - 1, j - 1, ROW, COL); //downward-left side traversal
                // DFS(M, i + 1, j - 1, ROW, COL); //downward-right side traversal
                // DFS(M, i - 1, j + 1, ROW, COL); //upward-left side traversal
            }
        }
    }
    return count;
}

int compute_contact_points(hw::Fingers::finger_t *finger){
    auto &S = finger->TactileSensor;
    std::vector<std::vector<int>> M =   {{S.force(0)>0?1:0, S.force(1)>0?1:0},
                                         {S.force(2)>0?1:0, S.force(3)>0?1:0},
                                         {S.force(4)>0?1:0, S.force(5)>0?1:0},
                                         {S.force(6)>0?1:0, S.force(6)>0?1:0}};
    return countIslands(M);
}


}//TactilePerception
