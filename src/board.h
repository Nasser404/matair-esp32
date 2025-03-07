#include <Arduino.h>
#include <piece.h>
class Board {

    public:
        Piece* grid[8][8];  // Using raw pointers instead of vectors to save ESP32 memory
        Board();
        ~Board();

        void addPiece(PieceType type, PieceColor color, std::pair<int, int> pos);
        bool movePiece(std::pair<int, int> from, std::pair<int, int> to);
        void printBoard() const;
        void initializeBoard();
        String getBoardString();
        String getSquareString(std::pair<int, int> pos);
        int getSquareNextionId(std::pair<int, int> pos);
        void resetBoard();
    };
    