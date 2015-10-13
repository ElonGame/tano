#pragma once
#include "parser/input_buffer.hpp"

namespace eval
{
  //------------------------------------------------------------------------------
  struct Token
  {
    enum class Type
    {
      Constant,
      Var,
      BinOp,
      LeftParen,
      RightParen,
      FuncCall,
      Comma,
      UnaryMinus,
      UnaryPlus,
    };

    enum BinOp
    {
      BinOpAdd, BinOpSub, BinOpMul, BinOpDiv
    };

    Token() {}
    Token(Type type) : type(type) {}
    Token(float constant) : type(Type::Constant), constant(constant) {}
    Token(Type type, const std::string& name) : type(type), name(name) {}
    Token(BinOp binOp) : type(Type::BinOp), binOp(binOp) {}

    Type type;
    BinOp binOp;
    std::string name;
    float constant;
  };

  struct Evaluator;
  typedef std::function<void(Evaluator* eval)> fnFunction;

  //------------------------------------------------------------------------------
  struct Environment
  {
    std::unordered_map<std::string, fnFunction> functions;
    std::unordered_map<std::string, float> constants;
  };

  //------------------------------------------------------------------------------
  bool Parse(const char* str, std::vector<Token>* expression);

  //------------------------------------------------------------------------------
  struct Evaluator
  {
    Evaluator();

    float EvaluateFromString(const char* str, const Environment* env);
    float Evaluate(const std::vector<Token>& expression, const Environment* env);

    void RegisterFunction(const std::string& name, const fnFunction& fn);
    void SetConstant(const std::string& name, float value);

    float PopValue();
    void PushValue(float value);
    Token PopOperator();
    void ApplyBinOp(Token::BinOp op);
    void LookupVar(const Token& t, const Environment* env);
    void InvokeFunction(const Token& t, const Environment* env);
    void ApplyUntilLeftParen(bool discardParen);

    std::deque<Token> operandStack;
    std::deque<Token> operatorStack;

    std::unordered_map<std::string, fnFunction> functions;
    std::unordered_map<std::string, float> constants;
  };
}
