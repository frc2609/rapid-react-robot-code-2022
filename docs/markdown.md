# .md file instructions

Press the book with a magnifying glass beside the WPILib button to view formatted text (top right of the screen)

Plain text is displayed as plain text.

Text with a # before it is displayed as a heading:

# This Is A Heading

Adding repeated # signs makes the heading smaller

## Subheading

### Really Small Heading

**Bold** text is surrounded by two asterisks *

*Italic* text is surrounded by one asterisk *

A single line code block is surrounded in one backtick: 

`single line code block`

Multi line code blocks are surrounded by three backticks:

```
System.out.println("Hello World!");
```

Adding a language name (e.g. java) in front of the first set of backticks (no space) colours the code for that language:

```java
System.out.println("Hello World!");
```

Sort of. Visual Studio correctly recognizes the markup colours for java but the markdown previewer is a different story.

Typing a number followed by a dot will create an ordered list.

1. Item one
1. Item two
1. Item three

Only the starting number matters, so you can use "1" for each number:

1. Item one
53234. Item two
1. Item three

Different starting number:

53. Item one
1. Item two
1. Item three

Use a dash for unordered lists:

- Hello there
- Another interesting line

Links are automatically recognized:

https://www.google.com

To insert special characters as normal ones, use a backwards slash:

\# \*

Text followed by one newline will be counted as the same line:
This will appear on the same line as the above sentence.

Text followed by two newlines will be placed on a seperate line:

(Seperated by a line from above sentence.)