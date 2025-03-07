#include <Arduino.h>
#include <iostream>
#include <vector>
#include <memory>


enum class PieceType { PAWN, KNIGHT, ROOK, BISHOP, KING, QUEEN };
enum class PieceColor { BLACK, WHITE };

class Piece {
protected:
    String name;
    std::pair<int, int> pos;
    PieceType type;
    PieceColor color;

public:
    Piece(String name, std::pair<int, int> pos, PieceType type, PieceColor color)
        : name(name), pos(pos), type(type), color(color) {}

    virtual ~Piece() = default;

    virtual char getSymbol() const = 0;
    std::pair<int, int> getPosition() const { return pos; }
    void setPosition(std::pair<int, int> newPos) { pos = newPos; }
    int getNextionId() const {
        switch (type) {
            case PieceType::PAWN: return (color == PieceColor::WHITE) ? 5 : 11;
            case PieceType::KNIGHT: return (color == PieceColor::WHITE) ? 6 : 12;
            case PieceType::BISHOP: return (color == PieceColor::WHITE) ? 7 : 13;
            case PieceType::ROOK: return (color == PieceColor::WHITE) ? 8 : 14;
            case PieceType::QUEEN: return (color == PieceColor::WHITE) ? 9 : 15;
            case PieceType::KING: return (color == PieceColor::WHITE) ? 10 : 16;
            default: return -1;
        }
    };
};

class Pawn : public Piece {
public:
    Pawn(std::pair<int, int> pos, PieceColor color)
        : Piece("Pawn", pos, PieceType::PAWN, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'P' : 'p'; }
};

class Knight : public Piece {
public:
    Knight(std::pair<int, int> pos, PieceColor color)
        : Piece("Knight", pos, PieceType::KNIGHT, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'N' : 'n'; }
};

class Rook : public Piece {
public:
    Rook(std::pair<int, int> pos, PieceColor color)
        : Piece("Rook", pos, PieceType::ROOK, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'R' : 'r'; }
};

class Bishop : public Piece {
public:
    Bishop(std::pair<int, int> pos, PieceColor color)
        : Piece("Bishop", pos, PieceType::BISHOP, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'B' : 'b'; }
};

class King : public Piece {
public:
    King(std::pair<int, int> pos, PieceColor color)
        : Piece("King", pos, PieceType::KING, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'K' : 'k'; }
};

class Queen : public Piece {
public:
    Queen(std::pair<int, int> pos, PieceColor color)
        : Piece("Queen", pos, PieceType::QUEEN, color) {}
    char getSymbol() const override { return (color == PieceColor::BLACK) ? 'Q' : 'q'; }
};


