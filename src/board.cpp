#include <board.h>
Board::Board() {
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            grid[i][j] = nullptr;
        }
    }
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

#include <cctype> // Required for the tolower() function

bool Board::isAtStartingPosition() {
    // Represents the standard chess starting board layout.
    // Uppercase for BLACK, lowercase for WHITE. '*' for empty.
    // This is indexed as [row][column] for easy visualization.
    const char initialBoardLayout[8][8] = {
        {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'}, // Row 0 (Black's back rank)
        {'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'}, // Row 1 (Black's pawns)
        {'*', '*', '*', '*', '*', '*', '*', '*'}, // Row 2
        {'*', '*', '*', '*', '*', '*', '*', '*'}, // Row 3
        {'*', '*', '*', '*', '*', '*', '*', '*'}, // Row 4
        {'*', '*', '*', '*', '*', '*', '*', '*'}, // Row 5
        {'p', 'p', 'p', 'p', 'p', 'p', 'p', 'p'}, // Row 6 (White's pawns)
        {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'}  // Row 7 (White's back rank)
    };

    // Iterate through your board, which is indexed as grid[column][row]
    for (int col = 0; col < 8; ++col) {
        for (int row = 0; row < 8; ++row) {
            Piece* currentPiece = this->grid[col][row];
            char expectedPieceChar = initialBoardLayout[row][col]; // Compare against layout[row][col]

            // Case 1: The square should be empty
            if (expectedPieceChar == '*') {
                if (currentPiece != nullptr) {
                    // This square should be empty, but it has a piece.
                    return false;
                }
            }
            // Case 2: The square should have a piece
            else {
                if (currentPiece == nullptr) {
                    // This square should have a piece, but it's empty.
                    return false;
                }

                // Check if the piece color is correct
                PieceColor expectedColor = (expectedPieceChar >= 'a' && expectedPieceChar <= 'z') ? PieceColor::WHITE : PieceColor::BLACK;
                if (currentPiece->getColor() != expectedColor) {
                    return false; // Mismatch in piece color
                }

                // Check if the piece type is correct
                PieceType expectedType;
                switch (tolower(expectedPieceChar)) {
                    case 'r': expectedType = PieceType::ROOK;   break;
                    case 'n': expectedType = PieceType::KNIGHT; break;
                    case 'b': expectedType = PieceType::BISHOP; break;
                    case 'q': expectedType = PieceType::QUEEN;  break;
                    case 'k': expectedType = PieceType::KING;   break;
                    case 'p': expectedType = PieceType::PAWN;   break;
                    default:  return false; // Should never happen
                }

                if (currentPiece->getType() != expectedType) {
                    return false; // Mismatch in piece type
                }
            }
        }
    }

    // If all squares have been checked and no mismatches were found, the board is correct.
    return true;
}