#include <board.h>
Board::Board() {
    initializeBoard();
}

Board::~Board() {
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            delete grid[i][j];
}

void Board::addPiece(PieceType type, PieceColor color, std::pair<int, int> pos) {
    int x = pos.first, y = pos.second;
    if (grid[x][y]) return;

    switch (type) {
        case PieceType::PAWN:
            grid[x][y] = new Pawn(pos, color);
            break;
        case PieceType::KNIGHT:
            grid[x][y] = new Knight(pos, color);
            break;
        case PieceType::ROOK:
            grid[x][y] = new Rook(pos, color);
            break;
        case PieceType::BISHOP:
            grid[x][y] = new Bishop(pos, color);
            break;
        case PieceType::KING:
            grid[x][y] = new King(pos, color);
            break;
        case PieceType::QUEEN:
            grid[x][y] = new Queen(pos, color);
            break;
    }
};
bool Board::movePiece(std::pair<int, int> from, std::pair<int, int> to) {
    int fromX = from.first, fromY = from.second;
    int toX = to.first, toY = to.second;

    if (!grid[fromX][fromY]) {
        Serial.print("No piece at the given 'from' position.\n");
        return false;
    }

    if (grid[toX][toY]) {
        delete grid[toX][toY];  // Capture the piece at the 'to' position
    }

    grid[toX][toY] = grid[fromX][fromY];
    grid[toX][toY]->setPosition(to);
    grid[fromX][fromY] = nullptr;

    return true;
};

void Board::printBoard() const {
    for (int j = 0; j < 8; ++j) {
        for (int i = 0; i < 8; ++i) {
            if (grid[i][j]) {
                Serial.print(grid[i][j]->getSymbol());
    
            } else {
                Serial.print("*");
            }
        }
        Serial.print("\n");
    }
};

void Board::initializeBoard() {
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            delete grid[i][j];
            grid[i][j] = nullptr;
        }
    }

    for (int i = 0; i < 8; ++i) {
        addPiece(PieceType::PAWN, PieceColor::BLACK, {i, 1});
        addPiece(PieceType::PAWN, PieceColor::WHITE, {i, 6});
    }
    addPiece(PieceType::ROOK, PieceColor::BLACK, {0, 0});
    addPiece(PieceType::ROOK, PieceColor::BLACK, {7, 0});
    addPiece(PieceType::ROOK, PieceColor::WHITE, {0, 7});
    addPiece(PieceType::ROOK, PieceColor::WHITE, {7, 7});
    addPiece(PieceType::KNIGHT, PieceColor::BLACK, {1, 0});
    addPiece(PieceType::KNIGHT, PieceColor::BLACK, {6, 0});
    addPiece(PieceType::KNIGHT, PieceColor::WHITE, {1, 7});
    addPiece(PieceType::KNIGHT, PieceColor::WHITE, {6, 7});
    addPiece(PieceType::BISHOP, PieceColor::BLACK, {2, 0});
    addPiece(PieceType::BISHOP, PieceColor::BLACK, {5, 0});
    addPiece(PieceType::BISHOP, PieceColor::WHITE, {2, 7});
    addPiece(PieceType::BISHOP, PieceColor::WHITE, {5, 7});
    addPiece(PieceType::QUEEN, PieceColor::BLACK, {3, 0});
    addPiece(PieceType::QUEEN, PieceColor::WHITE, {3, 7});
    addPiece(PieceType::KING, PieceColor::BLACK, {4, 0});
    addPiece(PieceType::KING, PieceColor::WHITE, {4, 7});
}
String Board::getBoardString() {
    String boardString;
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            if (grid[j][i] == nullptr) {
                boardString += "*";
            } else {
                boardString += grid[j][i]->getSymbol();
            }
        }
    }
    return boardString;
};
String Board::getSquareString(std::pair<int, int> pos) {
    char row[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    int x = pos.first, y = (8-pos.second);
    return String(row[x] + String(y));

}

int Board::getSquareNextionId(std::pair<int, int> pos) {

    int x = pos.first, y = pos.second;

    if (grid[x][y] == nullptr) return 1;
    else return grid[x][y]->getNextionId();

}

void Board::resetBoard() {
    initializeBoard();
}

bool Board::isAtStartingPosition() const {
    // Create a temporary standard board to compare against
    Board standardBoard; // This calls the constructor, which calls initializeBoard()

    // Iterate through every square
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            Piece* currentPiece = this->grid[i][j]; // The piece on our actual board
            Piece* standardPiece = standardBoard.grid[i][j]; // The piece that SHOULD be here

            // Case 1: Both squares are empty
            if (currentPiece == nullptr && standardPiece == nullptr) {
                continue; // This square is correct, check the next one
            }

            // Case 2: One square is empty and the other is not
            if ((currentPiece == nullptr && standardPiece != nullptr) ||
                (currentPiece != nullptr && standardPiece == nullptr)) {
                Serial.print("Board mismatch: Square ("); Serial.print(i); Serial.print(","); Serial.print(j);
                Serial.println(") has a piece when it should be empty, or vice-versa.");
                // No need to delete standardBoard pieces, its destructor will handle it when it goes out of scope.
                return false; // Mismatch found
            }

            // Case 3: Both squares have pieces, check if they are the same type and color
            if (currentPiece->getType() != standardPiece->getType() ||
                currentPiece->getColor() != standardPiece->getColor()) {
                Serial.print("Board mismatch: Square ("); Serial.print(i); Serial.print(","); Serial.print(j);
                Serial.print(") has piece "); Serial.print(currentPiece->getSymbol());
                Serial.print(" but should have "); Serial.println(standardPiece->getSymbol());
                // No need to delete standardBoard pieces, its destructor will handle it when it goes out of scope.
                return false; // Mismatch found
            }

            // If we get here, the pieces on this square match. Continue to next square.
        }
    }

    // If the loop completes without finding any mismatches, the board is in the starting position
    return true;
}