import { SessionManager } from '../Utils/SessionManager';

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};

// Mock window and localStorage
Object.defineProperty(window, 'localStorage', {
  value: localStorageMock,
});

describe('Session Persistence and Conversation History', () => {
  beforeEach(() => {
    // Clear all mocks before each test
    jest.clearAllMocks();

    // Reset localStorage mock
    localStorageMock.getItem.mockClear();
    localStorageMock.setItem.mockClear();
    localStorageMock.removeItem.mockClear();
  });

  test('creates a new session with unique ID', () => {
    const session = SessionManager.createSession();

    expect(session).toHaveProperty('id');
    expect(session).toHaveProperty('createdAt');
    expect(session).toHaveProperty('lastActive');
    expect(session).toHaveProperty('questions');
    expect(Array.isArray(session.questions)).toBe(true);
    expect(session.questions.length).toBe(0);

    // Check that the ID is in the correct format
    expect(session.id).toMatch(/^session_\d+_[a-z0-9]+$/);
  });

  test('stores session in localStorage', () => {
    const session = SessionManager.createSession();

    // Check that localStorage was called with the correct key and value
    expect(localStorageMock.setItem).toHaveBeenCalledWith(
      'ai-question-sessions',
      expect.any(String) // JSON string
    );

    // Parse the stored value to verify it contains our session
    const calls = localStorageMock.setItem.mock.calls;
    const storedValue = calls.find(call => call[0] === 'ai-question-sessions')?.[1];
    const storedSessions = JSON.parse(storedValue);

    expect(storedSessions).toContainEqual(session);
  });

  test('retrieves current session', () => {
    const session = SessionManager.createSession();

    const retrievedSession = SessionManager.getCurrentSession();

    expect(retrievedSession).toEqual(session);
  });

  test('adds question-response pair to session', () => {
    const session = SessionManager.createSession();
    const question = 'Test question';
    const response = { answer: 'Test response', sources: [] };

    SessionManager.addQuestionToSession(question, response);

    const updatedSession = SessionManager.getCurrentSession();

    expect(updatedSession.questions.length).toBe(1);
    expect(updatedSession.questions[0].question).toBe(question);
    expect(updatedSession.questions[0].response).toEqual(response);
    expect(updatedSession.questions[0].timestamp).toBeInstanceOf(Number);
  });

  test('gets session history', () => {
    const session = SessionManager.createSession();
    const question1 = 'First question';
    const response1 = { answer: 'First response', sources: [] };
    const question2 = 'Second question';
    const response2 = { answer: 'Second response', sources: [] };

    SessionManager.addQuestionToSession(question1, response1);
    SessionManager.addQuestionToSession(question2, response2);

    const history = SessionManager.getSessionHistory();

    expect(history.length).toBe(2);
    expect(history[0].question).toBe(question1);
    expect(history[1].question).toBe(question2);
  });

  test('session expires after timeout', () => {
    // Mock Date.now to return a fixed time
    const originalDateNow = Date.now;
    const mockTime = 1000000;
    Date.now = jest.fn().mockReturnValue(mockTime);

    const session = SessionManager.createSession();

    // Mock Date.now to return a time after the timeout period (30 minutes = 1800000 ms)
    (Date.now as jest.Mock).mockReturnValue(mockTime + 1800000 + 1); // Add 1ms to ensure it's over the timeout

    const expiredSession = SessionManager.getCurrentSession();

    expect(expiredSession).toBeNull();

    // Restore original Date.now
    Date.now = originalDateNow;
  });

  test('clears current session', () => {
    const session = SessionManager.createSession();

    SessionManager.clearCurrentSession();

    const currentSession = SessionManager.getCurrentSession();

    expect(currentSession).toBeNull();
  });

  test('persists multiple sessions', () => {
    const session1 = SessionManager.createSession();
    const session2 = SessionManager.createSession();

    const allSessions = SessionManager.getAllSessions();

    expect(allSessions.length).toBeGreaterThanOrEqual(2);
  });

  test('switches between sessions', () => {
    const session1 = SessionManager.createSession();
    const session1Id = session1.id;

    const session2 = SessionManager.createSession();
    const session2Id = session2.id;

    // Add a question to session1
    SessionManager.switchSession(session1Id);
    SessionManager.addQuestionToSession('Question for session 1', { answer: 'Response 1' });

    // Add a question to session2
    SessionManager.switchSession(session2Id);
    SessionManager.addQuestionToSession('Question for session 2', { answer: 'Response 2' });

    // Switch back to session1 and verify it has the correct history
    SessionManager.switchSession(session1Id);
    const session1History = SessionManager.getSessionHistory();
    expect(session1History.length).toBe(1);
    expect(session1History[0].question).toBe('Question for session 1');

    // Switch to session2 and verify it has the correct history
    SessionManager.switchSession(session2Id);
    const session2History = SessionManager.getSessionHistory();
    expect(session2History.length).toBe(1);
    expect(session2History[0].question).toBe('Question for session 2');
  });

  test('handles localStorage parsing errors gracefully', () => {
    // Mock localStorage to return invalid JSON
    localStorageMock.getItem.mockReturnValueOnce('{ invalid json');

    // This should not throw an error
    const sessions = SessionManager.getAllSessions();

    expect(sessions).toEqual([]);
  });

  test('handles localStorage storage errors gracefully', () => {
    // Mock localStorage to throw an error
    localStorageMock.setItem.mockImplementation(() => {
      throw new Error('Storage error');
    });

    // This should not throw an error
    expect(() => {
      SessionManager.createSession();
    }).not.toThrow();
  });
});