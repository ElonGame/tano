import sys

# note, stacks push/pop from the back
operator_stack = []
operand_stack = []


def push_operand(v):
    global operand_stack
    operand_stack += [v]


def push_operator(v):
    global operator_stack
    operator_stack += [v]


def peek_operator():
    global operator_stack
    if not operator_stack:
        return None
    return operator_stack[-1]


def pop_operand():
    return operand_stack.pop()


def pop_operator():
    return operator_stack.pop()


def op_plus():
    push_operand(pop_operand() + pop_operand())


def op_sub():
    b = pop_operand()
    a = pop_operand()
    push_operand(a - b)


def op_mul():
    push_operand(pop_operand() * pop_operand())


def op_div():
    b = pop_operand()
    a = pop_operand()
    push_operand(a / b)


ops = {
    '+': {'prio': 1, 'fn': op_plus},
    '-': {'prio': 1, 'fn': op_sub},
    '*': {'prio': 2, 'fn': op_mul},
    '/': {'prio': 2, 'fn': op_div},
}


def apply_op(op):
    ops[op]['fn']()


def funky():
    push_operand(pop_operand() + pop_operand())


def shunting(input):
    skip = 0
    for x in input:
        if skip > 0:
            skip -= 1
            continue
        try:
            operand = int(x)
            push_operand(operand)
        except ValueError:
            if x == '(':
                # push paren
                push_operator(x)
            elif x == ')':
                # right paren. apply ops until we hit left paren
                while True:
                    op = pop_operator()
                    if op == '(':
                        break
                    elif op == '[':
                        # start of function call
                        funky()
                        break
                    apply_op(op)
            elif x in ops:
                # apply any operators with higher prio
                prio = ops[x]['prio']
                while True:
                    op = peek_operator()
                    if op in ops and ops[op]['prio'] >= prio:
                        apply_op(pop_operator())
                    else:
                        break

                push_operator(x)
            elif x == ',':
                while True:
                    op = pop_operator()
                    if op == '[':
                        break
                    apply_op(op)
            else:
                # function call
                # push_operator(x)
                push_operator('[')
                skip = 1

        # print 'x: %r, operand: %r, operators: %r' % (
        #     x, operand_stack, operator_stack)

input = sys.argv[1]
shunting(input)
while peek_operator():
    apply_op(pop_operator())
print pop_operand()
