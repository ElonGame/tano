#include "evaluator.hpp"

using namespace std;
using namespace parser;

namespace eval
{
  static int BINOP_PRIO[4] = { 1, 1, 2, 2 };

  //------------------------------------------------------------------------------
  bool Parse(const char* str, vector<Token>* expression)
  {
    InputBuffer buf(str, strlen(str));

    while (!buf.Eof())
    {
      buf.SkipWhitespace();

      //char ch;
      float value;
      bool res;
      int idx;
      string fn;
      // woot, comma operator!
      if (value = parser::ParseFloat(buf, &res), res)
      {
        expression->push_back(Token(value));
      }
      else if (buf.IsOneOfIdx("+-*/", 4, &idx) && idx != -1)
      {
        expression->push_back(Token(Token::BinOp(idx)));
      }
      else if (buf.ConsumeIf('(') )
      {
        expression->push_back(Token(Token::Type::LeftParen));
      }
      else if (buf.ConsumeIf(')'))
      {
        expression->push_back(Token(Token::Type::RightParen));
      }
      else if (buf.ConsumeIf(','))
      {
        expression->push_back(Token(Token::Type::Comma));
      }
      else if (fn = parser::ParseIdentifier(buf, false, &res), res)
      {
        // differentiate between function calls and vars
        if (!buf.Eof() &&  buf.Peek() == '(')
          expression->push_back(Token(Token::Type::FuncCall, fn));
        else
          expression->push_back(Token(Token::Type::Var, fn));
      }
      else
      {
        LOG_WARN("Error tokenizing string", str);
        return false;
      }
    }
    return true;
  }

  //------------------------------------------------------------------------------
  Evaluator::Evaluator()
  {
    RegisterFunction("min", [](eval::Evaluator* eval) {
      eval->PushValue(min(eval->PopValue(), eval->PopValue()));
    });

    RegisterFunction("max", [](eval::Evaluator* eval) {
      eval->PushValue(max(eval->PopValue(), eval->PopValue()));
    });

    RegisterFunction("sin", [](eval::Evaluator* eval) {
      eval->PushValue(sin(eval->PopValue()));
    });

    RegisterFunction("cos", [](eval::Evaluator* eval) {
      eval->PushValue(cos(eval->PopValue()));
    });
  }

  //------------------------------------------------------------------------------
  void Evaluator::RegisterFunction(const string& name, const fnFunction& fn)
  {
    functions[name] = fn;
  }

  //------------------------------------------------------------------------------
  void Evaluator::SetConstant(const string& name, float value)
  {
    constants[name] = value;
  }

  //------------------------------------------------------------------------------
  float Evaluator::PopValue()
  {
    float v = operandStack.back().constant;
    operandStack.pop_back();
    return v;
  }

  //------------------------------------------------------------------------------
  void Evaluator::PushValue(float value)
  {
    operandStack.push_back(Token{ value });
  }

  //------------------------------------------------------------------------------
  Token Evaluator::PopOperator()
  {
    Token t = operatorStack.back();
    operatorStack.pop_back();
    return t;
  }

  //------------------------------------------------------------------------------
  void Evaluator::ApplyBinOp(Token::BinOp op)
  {
    float b = PopValue();
    float a = PopValue();
    switch (op)
    {
    case Token::BinOpAdd: PushValue(a + b); break;
    case Token::BinOpSub: PushValue(a - b); break;
    case Token::BinOpMul: PushValue(a * b); break;
    case Token::BinOpDiv: PushValue(a / b); break;
    default: LOG_WARN("Unknown bin-op!");
    }
  }

  //------------------------------------------------------------------------------
  void Evaluator::LookupVar(const Token& t, const Environment* env)
  {
    const string& name = t.name;
    if (env)
    {
      auto it = env->constants.find(name);
      if (it != env->constants.end())
      {
        PushValue(it->second);
        return;
      }
    }
    assert(constants.count(name));
    PushValue(constants[name]);
  }

  //------------------------------------------------------------------------------
  void Evaluator::InvokeFunction(const Token& t, const Environment* env)
  {
    const string& name = t.name;
    if (env)
    {
      auto it = env->functions.find(name);
      if (it != env->functions.end())
      {
        it->second(this);
        return;
      }
    }

    assert(functions.count(name));
    functions[name](this);
  }

  //------------------------------------------------------------------------------
  void Evaluator::ApplyUntilLeftParen(bool discardParen)
  {
    while (!operatorStack.empty())
    {
      Token token = PopOperator();
      if (token.type == Token::Type::LeftParen)
      {
        if (!discardParen)
          operatorStack.push_back(token);
        break;
      }
      else
      {
        ApplyBinOp(token.binOp);
      }
    }
  }

  //------------------------------------------------------------------------------
  float Evaluator::EvaluateFromString(const char* str, const Environment* env)
  {
    vector<Token> expr;
    Parse(str, &expr);
    return Evaluate(expr, env);
  }

  //------------------------------------------------------------------------------
  float Evaluator::Evaluate(const vector<Token>& expression, const Environment* env)
  {

    // Now perform the actual shunting :)
    for (size_t i = 0; i < expression.size(); ++i)
    {
      const Token& t = expression[i];

      if (t.type == Token::Type::Constant)
      {
        operandStack.push_back(t);
      }
      else if (t.type == Token::Type::BinOp)
      {
        // Apply any higher priority operators
        int prio = BINOP_PRIO[t.binOp];
        while (!operatorStack.empty())
        {
          const Token& op = operatorStack.back();
          if (op.type == Token::Type::BinOp && BINOP_PRIO[op.binOp] >= prio)
          {
            ApplyBinOp(op.binOp);
            operatorStack.pop_back();
          }
          else
            break;
        }

        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::FuncCall)
      {
        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::LeftParen)
      {
        operatorStack.push_back(t);
      }
      else if (t.type == Token::Type::Comma)
      {
        // apply all the operators until the left paren
        ApplyUntilLeftParen(false);
      }
      else if (t.type == Token::Type::RightParen)
      {
        ApplyUntilLeftParen(true);
        if (!operatorStack.empty())
        {
          // if the token at the top of the operator stack is a function call,
          // then invoke it
          Token t = operatorStack.back();
          if (t.type == Token::Type::FuncCall)
          {
            InvokeFunction(t, env);
            operatorStack.pop_back();
          }
        }
      }
      else if (t.type == Token::Type::Var)
      {
        LookupVar(t, env);
      }
    }

    // apply all the remaining operators
    while (!operatorStack.empty())
    {
      ApplyBinOp(PopOperator().binOp);
    }

    return PopValue();
  }
}
