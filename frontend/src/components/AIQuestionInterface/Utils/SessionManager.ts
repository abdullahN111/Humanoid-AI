export interface Session {
  id: string;
  createdAt: number;
  lastActive: number;
  questions: QuestionResponsePair[];
}

export interface QuestionResponsePair {
  question: string;
  response: any; // Using any to accommodate the AIResponseModel structure
  timestamp: number;
}

export class SessionManager {
  private static readonly STORAGE_KEY = 'ai-question-sessions';
  private static readonly CURRENT_SESSION_KEY = 'ai-current-session-id';
  private static readonly SESSION_TIMEOUT = 30 * 60 * 1000; // 30 minutes in milliseconds

  /**
   * Creates a new session
   * @returns The new session object
   */
  static createSession(): Session {
    const sessionId = this.generateSessionId();
    const now = Date.now();

    const newSession: Session = {
      id: sessionId,
      createdAt: now,
      lastActive: now,
      questions: [],
    };

    // Save the new session
    this.saveSession(newSession);

    // Set as current session
    this.setCurrentSessionId(sessionId);

    return newSession;
  }

  /**
   * Gets the current active session
   * @returns The current session or null if no session exists
   */
  static getCurrentSession(): Session | null {
    const currentSessionId = this.getCurrentSessionId();
    if (!currentSessionId) {
      return null;
    }

    const session = this.getSessionById(currentSessionId);

    // Check if session has expired
    if (session && Date.now() - session.lastActive > this.SESSION_TIMEOUT) {
      this.deleteSession(currentSessionId);
      this.setCurrentSessionId(null);
      return null;
    }

    return session;
  }

  /**
   * Adds a question-response pair to the current session
   * @param question The question text
   * @param response The AI response
   */
  static addQuestionToSession(question: string, response: any): void {
    const session = this.getCurrentSession();
    if (!session) {
      // Create a new session if none exists
      const newSession = this.createSession();
      this.addQuestionToSession(question, response);
      return;
    }

    const questionResponsePair: QuestionResponsePair = {
      question,
      response,
      timestamp: Date.now(),
    };

    session.questions.push(questionResponsePair);
    session.lastActive = Date.now();

    this.updateSession(session);
  }

  /**
   * Gets all questions and responses from the current session
   * @returns Array of question-response pairs
   */
  static getSessionHistory(): QuestionResponsePair[] {
    const session = this.getCurrentSession();
    return session ? [...session.questions] : [];
  }

  /**
   * Clears the current session
   */
  static clearCurrentSession(): void {
    const currentSessionId = this.getCurrentSessionId();
    if (currentSessionId) {
      this.deleteSession(currentSessionId);
      this.setCurrentSessionId(null);
    }
  }

  /**
   * Gets all available sessions
   * @returns Array of all sessions
   */
  static getAllSessions(): Session[] {
    const sessionsData = localStorage.getItem(this.STORAGE_KEY);
    if (!sessionsData) {
      return [];
    }

    try {
      const sessions = JSON.parse(sessionsData);
      // Filter out expired sessions
      return sessions.filter((session: Session) => {
        return Date.now() - session.lastActive <= this.SESSION_TIMEOUT;
      });
    } catch (error) {
      console.error('Error parsing sessions from localStorage:', error);
      return [];
    }
  }

  /**
   * Switches to a different session by ID
   * @param sessionId The ID of the session to switch to
   * @returns True if the switch was successful, false otherwise
   */
  static switchSession(sessionId: string): boolean {
    const session = this.getSessionById(sessionId);
    if (session) {
      this.setCurrentSessionId(sessionId);
      return true;
    }
    return false;
  }

  /**
   * Generates a unique session ID
   * @returns A unique session ID string
   */
  private static generateSessionId(): string {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Gets a session by its ID
   * @param sessionId The ID of the session to retrieve
   * @returns The session object or null if not found
   */
  private static getSessionById(sessionId: string): Session | null {
    const sessionsData = localStorage.getItem(this.STORAGE_KEY);
    if (!sessionsData) {
      return null;
    }

    try {
      const sessions: Session[] = JSON.parse(sessionsData);
      return sessions.find(session => session.id === sessionId) || null;
    } catch (error) {
      console.error('Error parsing session from localStorage:', error);
      return null;
    }
  }

  /**
   * Saves a session to localStorage
   * @param session The session to save
   */
  private static saveSession(session: Session): void {
    const sessions = this.getAllSessions();
    // Remove existing session with same ID if it exists
    const filteredSessions = sessions.filter(s => s.id !== session.id);
    // Add the updated session
    filteredSessions.push(session);

    try {
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(filteredSessions));
    } catch (error) {
      console.error('Error saving session to localStorage:', error);
    }
  }

  /**
   * Updates an existing session
   * @param session The updated session object
   */
  private static updateSession(session: Session): void {
    this.saveSession(session);
  }

  /**
   * Deletes a session by its ID
   * @param sessionId The ID of the session to delete
   */
  private static deleteSession(sessionId: string): void {
    const sessionsData = localStorage.getItem(this.STORAGE_KEY);
    if (!sessionsData) {
      return;
    }

    try {
      const sessions: Session[] = JSON.parse(sessionsData);
      const filteredSessions = sessions.filter(session => session.id !== sessionId);
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(filteredSessions));
    } catch (error) {
      console.error('Error deleting session from localStorage:', error);
    }
  }

  /**
   * Gets the current session ID from localStorage
   * @returns The current session ID or null if none exists
   */
  private static getCurrentSessionId(): string | null {
    if (typeof localStorage === 'undefined') {
      return null;
    }

    return localStorage.getItem(this.CURRENT_SESSION_KEY);
  }

  /**
   * Sets the current session ID in localStorage
   * @param sessionId The session ID to set as current, or null to clear
   */
  private static setCurrentSessionId(sessionId: string | null): void {
    if (typeof localStorage === 'undefined') {
      return;
    }

    if (sessionId) {
      localStorage.setItem(this.CURRENT_SESSION_KEY, sessionId);
    } else {
      localStorage.removeItem(this.CURRENT_SESSION_KEY);
    }
  }
}