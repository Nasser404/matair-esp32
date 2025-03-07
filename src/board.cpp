#include <board.h>
Board::Board() {
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            grid[i][j] = nullptr;
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

};
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
    // Delete existing pieces
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            delete grid[i][j];  // Free memory
            grid[i][j] = nullptr;
        }
    }

    // Reinitialize pieces
    initializeBoard();
}