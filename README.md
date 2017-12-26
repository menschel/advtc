# AdvTC (Advanced Traction Controller)

This project describes a fuzzy logic controller for traction control of a ground based vehicle.

AdvTC uses fuzzy logic with just 2 sensors, 4 rules and 8bit width for peak slip targetting regardless of tires and road conditions.

The project originates from my work on the 2009's formula student car of the University of Paderborn, Germany.

The provided assembly file is written for freescale HCS12 processor and integrates into megasquire 2 code of that time.


## Why fuzzy logic?
Fuzzy logic ...
1. doesn't require a precise model for the system it is to be used on,
2. is largely immune to changing condition, e.g. changing road surface condition,
3. is stable by default, e.g. no output oscillations.

Fuzzy logic does a pretty good job in unpredictable systems.

## Some background on fuzzy logic

Fuzzy logic is a way to set up rules by math representing logic dependencies with math operations.

### getting fuzzy inputs

Input conditions are truth based values that form a condition.

In our case the speed difference in between two wheels is the physical value and we aquire it by subtracting two 8bit ADC values leaving us with a signed 8bit integer.

We convert it to unsigned 8 bit integer to get a truth value for that condition. If the wheels are slipping, the difference is high and so is the truth value.

### setting fuzzy rules

Fuzzy rules are combination of fuzzy inputs. If condition A and B are met, we want controller step C. A and B operation is nothing but min(A,B) in math which results in min(A,B)*C for that rule. The min(A,B) is the truth that this rule is to be applied.

### rule evaluation and output

Nothing but the normalized sum of all evaluated rules, i.e. sum(Rule1,...,RuleN)/sum(truth(Rule1,...,RuleN).


