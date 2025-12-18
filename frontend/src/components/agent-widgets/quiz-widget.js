import React, { useState } from 'react';
import ApiService from '../../services/api';
import { trackQuizCreation, trackError } from '../../services/analytics';
import './quiz-widget.css';

const QuizWidget = ({ content }) => {
  const [quiz, setQuiz] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [userAnswers, setUserAnswers] = useState([]);

  const generateQuiz = async (quizContent = content) => {
    if (!quizContent) {
      setError('No content provided for quiz generation');
      trackError(new Error('No content provided for quiz generation'), {
        component: 'QuizWidget',
        action: 'generateQuiz'
      });
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Track the quiz creation request
      trackQuizCreation({
        content_length: quizContent.length,
        trigger: 'widget_button_click'
      });

      const response = await ApiService.createQuiz(quizContent);

      // In a real implementation, we would get the quiz data from the response
      // For now, we'll simulate polling for the result
      const requestId = response.request_id;

      // Simulate polling for the result
      const pollForResult = async () => {
        try {
          const statusResponse = await ApiService.getRequestStatus(requestId);
          if (statusResponse.status === 'COMPLETED') {
            // In a real implementation, we would fetch the actual generated content
            // For now, we'll simulate a response
            const mockQuiz = {
              title: "Generated Quiz",
              metadata: { difficulty: "MEDIUM", questions_count: 3 },
              questions: {
                multiple_choice: [
                  {
                    id: "mcq_1",
                    question: "What is the primary purpose of this concept?",
                    options: ["Option A", "Option B", "Option C", "Option D"],
                    correct_answer_index: 1
                  },
                  {
                    id: "mcq_2",
                    question: "Which of the following is a key characteristic?",
                    options: ["Option A", "Option B", "Option C", "Option D"],
                    correct_answer_index: 2
                  }
                ],
                short_answer: [
                  {
                    id: "sa_1",
                    question: "Explain the main principles in your own words.",
                    expected_length: "medium"
                  }
                ]
              }
            };
            setQuiz(mockQuiz);
            setLoading(false);
            setCurrentQuestion(0);
            setUserAnswers(new Array(mockQuiz.questions.multiple_choice.length, null));

            // Track successful completion
            trackQuizCreation({
              status: 'success',
              request_id: requestId,
              questions_count: mockQuiz.questions.multiple_choice.length + mockQuiz.questions.short_answer.length
            });
          } else if (statusResponse.status === 'FAILED') {
            setError('Quiz generation failed');
            setLoading(false);

            // Track failure
            trackQuizCreation({
              status: 'failed',
              request_id: requestId
            });
          } else {
            // Still processing, check again in 2 seconds
            setTimeout(pollForResult, 2000);
          }
        } catch (err) {
          setError('Failed to check quiz generation status');
          setLoading(false);

          // Track error
          trackError(err, {
            component: 'QuizWidget',
            action: 'pollForResult',
            request_id: requestId
          });
        }
      };

      // Start polling for the result
      setTimeout(pollForResult, 1000);
    } catch (err) {
      setError(err.message || 'Failed to generate quiz');
      setLoading(false);

      // Track error
      trackError(err, {
        component: 'QuizWidget',
        action: 'generateQuiz'
      });
    }
  };

  const handleAnswerSelect = (questionIndex, answerIndex) => {
    const newAnswers = [...userAnswers];
    newAnswers[questionIndex] = answerIndex;
    setUserAnswers(newAnswers);
  };

  const nextQuestion = () => {
    if (currentQuestion < getTotalQuestions() - 1) {
      setCurrentQuestion(currentQuestion + 1);
    }
  };

  const prevQuestion = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
    }
  };

  const getTotalQuestions = () => {
    if (!quiz) return 0;
    return quiz.questions.multiple_choice.length + quiz.questions.short_answer.length;
  };

  const getQuestion = () => {
    if (!quiz) return null;

    const mcqCount = quiz.questions.multiple_choice.length;

    if (currentQuestion < mcqCount) {
      return {
        type: 'multiple_choice',
        ...quiz.questions.multiple_choice[currentQuestion]
      };
    } else {
      const saIndex = currentQuestion - mcqCount;
      return {
        type: 'short_answer',
        ...quiz.questions.short_answer[saIndex]
      };
    }
  };

  const currentQuestionData = getQuestion();

  return (
    <div className="quiz-widget">
      <h3>Quiz Creator</h3>
      {!quiz ? (
        <button
          onClick={generateQuiz}
          disabled={loading}
          className="generate-btn"
        >
          {loading ? 'Generating...' : 'Generate Quiz'}
        </button>
      ) : (
        <div className="quiz-container">
          <div className="quiz-header">
            <h4>{quiz.title}</h4>
            <p>Question {currentQuestion + 1} of {getTotalQuestions()}</p>
          </div>

          {error && <div className="error">{error}</div>}

          {currentQuestionData && (
            <div className="question-container">
              <h5>{currentQuestionData.question}</h5>

              {currentQuestionData.type === 'multiple_choice' ? (
                <div className="options">
                  {currentQuestionData.options.map((option, index) => (
                    <div key={index} className="option">
                      <input
                        type="radio"
                        id={`option-${currentQuestion}-${index}`}
                        name={`question-${currentQuestion}`}
                        checked={userAnswers[currentQuestion] === index}
                        onChange={() => handleAnswerSelect(currentQuestion, index)}
                      />
                      <label htmlFor={`option-${currentQuestion}-${index}`}>
                        {option}
                      </label>
                    </div>
                  ))}
                </div>
              ) : (
                <textarea
                  className="short-answer"
                  placeholder="Type your answer here..."
                  rows="4"
                />
              )}
            </div>
          )}

          <div className="navigation">
            <button
              onClick={prevQuestion}
              disabled={currentQuestion === 0}
              className="nav-btn"
            >
              Previous
            </button>
            {currentQuestion < getTotalQuestions() - 1 ? (
              <button
                onClick={nextQuestion}
                className="nav-btn next-btn"
              >
                Next
              </button>
            ) : (
              <button className="nav-btn submit-btn">
                Submit Quiz
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default QuizWidget;